#!/usr/bin/env python3
"""
Анализ фазы и амплитуды 1x из записей виброметра.

Алгоритм:
1. Определяем обороты по фронтам keyphasor (rising edge: flags 0→2)
2. Ищем непрерывный участок с постоянной скоростью (CV RPM < 1%)
3. IQ-демодуляция 1x по каждому обороту → усредняем по участку
"""
import sys
import json
import numpy as np
import pandas as pd
from pathlib import Path

SYSTIMER_HZ = 16_000_000.0
RECORDINGS_DIR = Path(__file__).parent / "recordings"

# --------------------------------------------------------------------------- #

def load_recording(num: int) -> pd.DataFrame:
    path = RECORDINGS_DIR / f"{num:03d}.parquet"
    return pd.read_parquet(path)


def detect_rising_edges(df: pd.DataFrame) -> np.ndarray:
    """Индексы сэмплов перед которыми keyphasor переходит 0→2 (rising)."""
    level = (df["flags"].values & 2).astype(np.int8)
    diff = np.diff(level)
    return np.where(diff > 0)[0] + 1   # индекс первого сэмпла с level=HIGH


def find_steady_segment(
    edge_ticks: np.ndarray,
    cv_threshold: float = 0.01,
    min_revs: int = 20,
) -> tuple[int, int] | None:
    """
    Ищет наибольший непрерывный участок с CV периода < cv_threshold.
    Возвращает (edge_start_idx, edge_end_idx) — индексы в edge_ticks.
    """
    periods = np.diff(edge_ticks).astype(np.float64)
    n = len(periods)
    best = None
    best_len = 0

    i = 0
    while i < n:
        j = i
        # Расширяем участок пока CV < threshold
        while j < n:
            seg = periods[i:j + 1]
            if seg.std() / seg.mean() < cv_threshold:
                j += 1
            else:
                break
        seg_len = j - i
        if seg_len >= min_revs and seg_len > best_len:
            best_len = seg_len
            best = (i, j)   # обороты i..j-1, края: edge_ticks[i]..edge_ticks[j]
        i = max(i + 1, j)

    return best


def iq_demodulate_1x(
    ticks: np.ndarray,
    ch0: np.ndarray,
    edge_tick_from: int,
    edge_tick_to: int,
) -> tuple[float, float]:
    """
    IQ-демодуляция 1x для одного оборота [edge_tick_from, edge_tick_to).
    Возвращает (amplitude, phase_deg).

    phase=0 → пик вибрации совпадает с keyphasor mark.
    """
    mask = (ticks >= edge_tick_from) & (ticks < edge_tick_to)
    t = ticks[mask]
    x = ch0[mask]
    n = len(t)
    if n == 0:
        return 0.0, 0.0

    period = edge_tick_to - edge_tick_from
    theta = 2.0 * np.pi * (t - edge_tick_from) / period
    I = np.sum(x * np.cos(theta)) / n
    Q = np.sum(x * np.sin(theta)) / n
    amp = 2.0 * np.hypot(I, Q)
    phase_deg = np.degrees(np.arctan2(Q, I)) % 360.0
    return amp, phase_deg


def analyze_recording(num: int, verbose: bool = False) -> dict:
    df = load_recording(num)
    ticks = df["tick"].values.astype(np.int64)
    ch0 = df["ch0"].values.astype(np.float64)

    # Обнаружение оборотов
    edge_idxs = detect_rising_edges(df)
    edge_ticks = ticks[edge_idxs]

    if len(edge_ticks) < 3:
        return {"num": num, "error": "слишком мало оборотов"}

    # Поиск стационарного участка
    seg = find_steady_segment(edge_ticks, cv_threshold=0.01, min_revs=10)
    if seg is None:
        # Смягчаем: CV 2%, min 5 оборотов
        seg = find_steady_segment(edge_ticks, cv_threshold=0.02, min_revs=5)
    if seg is None:
        return {"num": num, "error": "не найден стационарный участок"}

    ei_start, ei_end = seg   # edge indices
    # ei_end = последний edge конца участка

    # RPM по участку
    periods = np.diff(edge_ticks[ei_start:ei_end + 1]).astype(np.float64)
    mean_period_s = periods.mean() / SYSTIMER_HZ
    rpm = 60.0 / mean_period_s
    revs = len(periods)

    # IQ по каждому обороту
    amps, phases = [], []
    for k in range(ei_start, ei_end):
        t_from = edge_ticks[k]
        t_to = edge_ticks[k + 1]
        amp, phase = iq_demodulate_1x(ticks, ch0, t_from, t_to)
        amps.append(amp)
        phases.append(phase)

    amps = np.array(amps)
    phases = np.array(phases)

    # Усреднение по кругу (circular mean для фазы)
    mean_amp = amps.mean()
    sin_mean = np.sin(np.radians(phases)).mean()
    cos_mean = np.cos(np.radians(phases)).mean()
    mean_phase = np.degrees(np.arctan2(sin_mean, cos_mean)) % 360.0
    phase_std = np.degrees(np.sqrt(np.mean(
        np.angle(np.exp(1j * np.radians(phases - mean_phase))) ** 2
    )))

    # Время участка
    t_start_s = (edge_ticks[ei_start] - ticks[0]) / SYSTIMER_HZ
    t_end_s   = (edge_ticks[ei_end]   - ticks[0]) / SYSTIMER_HZ

    return {
        "num":       num,
        "rpm":       rpm,
        "revs":      revs,
        "amp_mean":  mean_amp,
        "phase_deg": mean_phase,
        "phase_std": phase_std,
        "t_start_s": t_start_s,
        "t_end_s":   t_end_s,
    }


def main():
    with open(RECORDINGS_DIR / "comments.json") as f:
        comments = json.load(f)

    nums = [91, 92, 93, 94]

    print(f"{'#':>3}  {'Комментарий':<15}  {'RPM':>6}  {'Обор':>5}  "
          f"{'Амп (LSB)':>10}  {'Фаза°':>7}  {'±σ':>5}  "
          f"{'Участок, с':>15}")
    print("-" * 85)

    for n in nums:
        key = f"{n:03d}.parquet"
        comment = comments.get(key, "—")
        r = analyze_recording(n)

        if "error" in r:
            print(f"{n:>3}  {comment:<15}  {r['error']}")
            continue

        t_range = f"{r['t_start_s']:.1f} – {r['t_end_s']:.1f}"
        print(
            f"{n:>3}  {comment:<15}  {r['rpm']:>6.1f}  {r['revs']:>5}  "
            f"{r['amp_mean']:>10.1f}  {r['phase_deg']:>7.1f}  "
            f"{r['phase_std']:>5.1f}  {t_range:>15}"
        )


if __name__ == "__main__":
    main()
