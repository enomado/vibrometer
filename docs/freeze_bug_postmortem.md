# Postmortem: зависание firmware при SetPga / SetDataRate

## Симптомы

При попытке сменить PGA или sample rate в рантайме (команды `SetPga`, `SetDataRate`)
прошивка зависала намертво. Логи показывали:

```
adc_command_task: got cmd SetPga(32), taking peripherals...
take_peripherals: before cs
```

После этого — тишина. Ни `take_peripherals: done`, ни дальнейшего прогресса.
WiFi жил, ISR продолжал захватывать сэмплы, но `adc_command_task` был заморожен.
Перезагрузка по WDT не срабатывала — задача просто не возвращала управление.

---

## Архитектура (контекст)

```
Priority2 ISR (drdy_isr)
  └─ GPIO DRDY↓ → SPI RDATA (~25µs) → Sample → SPSC queue

Priority1 embassy executor
  ├─ stream_loop   — дренирует SPSC, отправляет по TCP
  ├─ adc_command_task — обрабатывает SetPga/SetDataRate:
  │    take_peripherals() → reconfig ADS1256 → give_back()
  └─ keyphasor_task, net_task, ...
```

`take_peripherals()` должна атомарно забрать SPI/CS/DRDY из ISR и выключить прерывание,
чтобы `adc_command_task` мог блокирующе переконфигурировать ADS1256 (SELFCAL, drain).

---

## Причина #1 (первичная): зависание в `take_peripherals` — `unlisten()` под GPIO_LOCK

### Что делает `unlisten()`

```rust
// esp-hal/src/gpio/mod.rs
pub fn unlisten(&mut self) {
    GPIO_LOCK.lock(|| { set_int_enable(self.pin, false); });
}
```

`GPIO_LOCK` — это `SingleCoreInterruptLock` (отключает прерывания) + `Cell<bool>`
для отслеживания реентерабельности.

### Что делает GPIO dispatcher esp-hal

```rust
// esp-hal/src/gpio/interrupt.rs
fn user_gpio_interrupt_handler() {
    GPIO_LOCK.lock(|| {           // ← берёт GPIO_LOCK
        USER_INTERRUPT_HANDLER.call();  // ← вызывает drdy_isr
    });
    // ← отпускает GPIO_LOCK
}
```

### Цепочка событий, приводящая к зависанию

1. DRDY↓ → CPU переходит на `user_gpio_interrupt_handler`.
2. Handler берёт `GPIO_LOCK.lock()` — MIE=0, `is_reentry=false → true`.
3. Вызывает `drdy_isr` → SPI read (~25µs).
4. **В это время** embassy executor успевает начать `take_peripherals()`.
   *(На single-core RISC-V это невозможно если ISR не возвращается, но...)*

**Реальная проблема**: в старом коде `unlisten()` вызывался **вне** `critical_section::with`,
уже после того как `ISR_STATE` был обнулён. В момент вызова `unlisten()`:

- ISR мог находиться в процессе выполнения (на SPI транзакции).
- `GPIO_LOCK` в esp-hal использует `Cell<bool>` (не атомик) для `is_reentry`.
- `unlisten()` → `GPIO_LOCK.lock()` → видит `is_reentry=true` (ISR в полёте) →
  отмечает reentry, на выходе **не восстанавливает MIE** (считает что он и не был выключен).
- Но MIE-то был выключен внешним кодом! Итог: MIE остаётся=0 навсегда.
  Прерывания замолчали. Embassy не получает таймерных тиков. Задача зависла.

> Точная механика зависит от версии esp-hal, но эффект один:
> вызов `unlisten()` вне критической секции создаёт гонку с GPIO_LOCK,
> которая при определённом тайминге ломает состояние прерываний.

### Фикс

Перенести `unlisten()` **внутрь** `critical_section::with`:

```rust
// isr_capture.rs — take_peripherals()
pub fn take_peripherals() -> (...) {
    let st = critical_section::with(|cs| {
        let mut st = ISR_STATE.borrow(cs).borrow_mut().take().unwrap();
        // unlisten внутри cs — пока MIE отключён, GPIO ISR не может быть в полёте.
        // GPIO_LOCK не будет "захвачен" ISR контекстом → unlisten безопасен.
        st.drdy.unlisten();
        st
    });
    (st.spi, st.cs_pin, st.drdy, st.tx)
}
```

Пока `critical_section::with` держит MIE=0, никакой ISR не выполняется.
`GPIO_LOCK.is_reentry` = false. `unlisten()` отрабатывает чисто.

---

## Причина #2 (вторичная): дедлок `stream_loop` ↔ `adc_command_task`

Даже после фикса #1 система могла зависать по второй причине.

### Embassy — кооперативный планировщик

Embassy на ESP32-C3 — **cooperative** (не preemptive). Задача держит CPU до первого `.await`.
Если задача крутится в цикле без yield — остальные задачи не получают управление.

### Старый код `stream_loop`

```rust
// БЫЛО (stream.rs)
if RECONFIG_IN_PROGRESS.load(Ordering::Acquire) {
    continue;  // ← нет .await! spin-loop без yield
}
```

### Что происходило

1. `adc_command_task` ставит `RECONFIG_IN_PROGRESS=true`, вызывает `take_peripherals()`.
2. `take_peripherals()` отрабатывает (ISR забран).
3. `adc_command_task` должен начать `set_pga()` → `wait_drdy_low()` → poll-loop.
4. **Но**: `stream_loop` в этот момент уже вошёл в `RECONFIG_IN_PROGRESS` spin-loop.
5. `stream_loop` крутится без единого `.await` — embassy не может переключиться на `adc_command_task`.
6. `adc_command_task` никогда не получает CPU. `RECONFIG_IN_PROGRESS` никогда не снимается.
7. **Дедлок**.

Дополнительно: `stream_loop` получает команду и сразу же уходит в следующую итерацию цикла,
не давая `adc_command_task` выполниться. Нужен явный yield после dispatch команды.

### Фикс

```rust
// network.rs — stream_loop

// После dispatch команды — явный yield
let had_cmd = try_recv_command(...).await;
if had_cmd {
    Timer::after(Duration::from_millis(0)).await;  // yield без задержки
}

// В reconfig spin-loop — обязательный yield
if RECONFIG_IN_PROGRESS.load(Ordering::Acquire) {
    Timer::after(Duration::from_millis(1)).await;  // ← без этого дедлок
    continue;
}
```

`Timer::after(0)` и `Timer::after(1ms)` — оба дают embassy шанс переключить задачу.

---

## Причина #3 (вторичная): `continue` обходил `give_back` при skip

Старый `adc_command_task` при "уже такой rate/PGA" делал ранний `continue`:

```rust
// БЫЛО
if current == v {
    let (spi, cs, drdy) = adc.into_parts();
    crate::isr_capture::give_back(spi, cs, drdy, tx);
    continue;
    // ← RECONFIG_IN_PROGRESS остаётся true навсегда!
}
```

Флаг `RECONFIG_IN_PROGRESS` ставился перед `take_peripherals`, но сбрасывался
**после** match-блока. При early `continue` сброс пропускался → флаг навсегда true.

### Фикс

Убрать early `continue`, унифицировать путь: `give_back` + сброс флага — всегда в конце:

```rust
// stream.rs — adc_command_task
match cmd {
    Command::SetPga(v) => {
        if current == v {
            // просто skip, не continue
        } else {
            adc.set_pga(v);
            CURRENT_PGA.store(v, Ordering::Relaxed);
        }
    }
    // ...
}
// ЕДИНСТВЕННЫЙ give_back и сброс флага — всегда выполняется:
let (spi, cs, drdy) = adc.into_parts();
crate::isr_capture::give_back(spi, cs, drdy, tx);
RECONFIG_IN_PROGRESS.store(false, Ordering::Release);
```

---

## Итого: три независимых бага, наслоившихся друг на друга

| # | Файл | Проблема | Фикс |
|---|------|----------|------|
| 1 | `isr_capture.rs` | `unlisten()` вне cs → гонка с GPIO_LOCK → MIE застрял в 0 | `unlisten()` внутрь `critical_section::with` |
| 2 | `network.rs` / `stream.rs` | `RECONFIG_IN_PROGRESS` spin-loop без yield → дедлок embassy | `Timer::after(1ms).await` перед `continue` |
| 3 | `stream.rs` | Early `continue` в `adc_command_task` обходил сброс флага | Убрать early continue, единый exit path |

Для воспроизведения зависания достаточно любого одного из трёх. Устранение всех трёх
сделало смену PGA/rate надёжной.

---

## Диагностические принты, помогли найти баги

```rust
// isr_capture.rs
esp_println::println!("take_peripherals: before cs");  // чекпоинт до cs
esp_println::println!("take_peripherals: done");       // чекпоинт после

// stream.rs
esp_println::println!("adc_command_task: got cmd {:?}, taking peripherals...", cmd);
esp_println::println!("adc_command_task: peripherals taken");

// ads1256.rs — в wait_drdy_low(), set_pga(), set_data_rate()
// печать состояния DRDY и elapsed time на каждом этапе
```

Когда логи показали `"before cs"` без `"done"` → сужение до `cs` или `unlisten()`.
Когда `"peripherals taken"` никогда не появлялось → сужение до `take_peripherals()`.

---

## Связанные коммиты

- `4638524` — fix: yield in stream_loop during reconfig to unblock adc_command_task
- `b27e5a0` — fw: flush stale samples during reconfig, unify give_back exit path
- `9f850b6` — diag: log adc_command_task entry + explicit yield after cmd dispatch
- `263f31a` — upd (isr_capture.rs переписан на `Mutex<CriticalSectionRawMutex, RefCell<Option<IsrState>>>`,
  `unlisten()` перенесён внутрь `critical_section::with`)
