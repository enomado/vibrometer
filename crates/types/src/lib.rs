#![no_std]

use core::fmt;

use serde::{
    Deserialize,
    Serialize,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub struct AdcCount(pub i32);

impl AdcCount {
    pub const ZERO: Self = Self(0);

    pub fn as_i32(self) -> i32 {
        self.0
    }

    pub fn as_f64(self) -> f64 {
        self.0 as f64
    }
}

impl From<i32> for AdcCount {
    fn from(value: i32) -> Self {
        Self(value)
    }
}

impl fmt::Display for AdcCount {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub struct PacketSeq(pub u32);

impl PacketSeq {
    pub const ZERO: Self = Self(0);

    pub fn as_u32(self) -> u32 {
        self.0
    }
}

impl From<u32> for PacketSeq {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

impl fmt::Display for PacketSeq {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub struct SampleRateHz(pub u16);

impl SampleRateHz {
    pub const ZERO: Self = Self(0);

    pub fn as_u16(self) -> u16 {
        self.0
    }

    pub fn as_f64(self) -> f64 {
        self.0 as f64
    }

    pub fn as_hz(self) -> Hertz {
        Hertz(self.as_f64())
    }
}

impl From<u16> for SampleRateHz {
    fn from(value: u16) -> Self {
        Self(value)
    }
}

impl PartialEq<u16> for SampleRateHz {
    fn eq(&self, other: &u16) -> bool {
        self.0 == *other
    }
}

impl PartialEq<f64> for SampleRateHz {
    fn eq(&self, other: &f64) -> bool {
        self.as_f64() == *other
    }
}

impl fmt::Display for SampleRateHz {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct Hertz(pub f64);

impl Hertz {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f64) -> Self {
        Self(value)
    }

    pub fn as_f64(self) -> f64 {
        self.0
    }

    pub fn as_rpm(self) -> Rpm {
        Rpm(self.0 * 60.0)
    }
}

impl From<f64> for Hertz {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl core::ops::Sub<f64> for Hertz {
    type Output = f64;

    fn sub(self, rhs: f64) -> Self::Output {
        self.0 - rhs
    }
}

impl core::ops::Mul<f64> for Hertz {
    type Output = f64;

    fn mul(self, rhs: f64) -> Self::Output {
        self.0 * rhs
    }
}

impl fmt::Display for Hertz {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.3}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct Rpm(pub f64);

impl Rpm {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f64) -> Self {
        Self(value)
    }

    pub fn as_f64(self) -> f64 {
        self.0
    }

    pub fn as_hz(self) -> Hertz {
        Hertz(self.0 / 60.0)
    }
}

impl From<f64> for Rpm {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl core::ops::Sub<f64> for Rpm {
    type Output = f64;

    fn sub(self, rhs: f64) -> Self::Output {
        self.0 - rhs
    }
}

impl core::ops::Div<f64> for Rpm {
    type Output = f64;

    fn div(self, rhs: f64) -> Self::Output {
        self.0 / rhs
    }
}

impl PartialEq<f64> for Rpm {
    fn eq(&self, other: &f64) -> bool {
        self.0 == *other
    }
}

impl PartialOrd<f64> for Rpm {
    fn partial_cmp(&self, other: &f64) -> Option<core::cmp::Ordering> {
        self.0.partial_cmp(other)
    }
}

impl fmt::Display for Rpm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct AngleRad(pub f64);

impl AngleRad {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f64) -> Self {
        Self(value)
    }

    pub fn as_f64(self) -> f64 {
        self.0
    }

    pub fn normalized(self) -> Self {
        let two_pi = 2.0 * core::f64::consts::PI;
        let mut r = self.0 % two_pi;
        if r > core::f64::consts::PI {
            r -= two_pi;
        } else if r <= -core::f64::consts::PI {
            r += two_pi;
        }
        Self(r)
    }

    pub fn to_degrees(self) -> AngleDeg {
        AngleDeg(self.0.to_degrees())
    }
}

impl From<f64> for AngleRad {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl core::ops::Sub<f64> for AngleRad {
    type Output = f64;

    fn sub(self, rhs: f64) -> Self::Output {
        self.0 - rhs
    }
}

impl core::ops::Sub for AngleRad {
    type Output = f64;

    fn sub(self, rhs: Self) -> Self::Output {
        self.0 - rhs.0
    }
}

impl PartialEq<f64> for AngleRad {
    fn eq(&self, other: &f64) -> bool {
        self.0 == *other
    }
}

impl PartialOrd<f64> for AngleRad {
    fn partial_cmp(&self, other: &f64) -> Option<core::cmp::Ordering> {
        self.0.partial_cmp(other)
    }
}

impl fmt::Display for AngleRad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.3}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct AngleDeg(pub f64);

impl AngleDeg {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f64) -> Self {
        Self(value)
    }

    pub fn as_f64(self) -> f64 {
        self.0
    }

    pub fn to_radians(self) -> AngleRad {
        AngleRad(self.0.to_radians()).normalized()
    }
}

impl From<f64> for AngleDeg {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl fmt::Display for AngleDeg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.1}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct MassGrams(pub f64);

impl MassGrams {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f64) -> Self {
        Self(value)
    }

    pub fn as_f64(self) -> f64 {
        self.0
    }
}

impl From<f64> for MassGrams {
    fn from(value: f64) -> Self {
        Self(value)
    }
}

impl core::ops::Sub<f64> for MassGrams {
    type Output = f64;

    fn sub(self, rhs: f64) -> Self::Output {
        self.0 - rhs
    }
}

impl fmt::Display for MassGrams {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.2}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub struct SampleIndex(pub usize);

impl SampleIndex {
    pub const ZERO: Self = Self(0);

    pub fn as_usize(self) -> usize {
        self.0
    }

    pub fn as_f64(self) -> f64 {
        self.0 as f64
    }
}

impl From<usize> for SampleIndex {
    fn from(value: usize) -> Self {
        Self(value)
    }
}

impl PartialEq<usize> for SampleIndex {
    fn eq(&self, other: &usize) -> bool {
        self.0 == *other
    }
}

impl fmt::Display for SampleIndex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize)]
pub struct SampleCount(pub usize);

impl SampleCount {
    pub const ZERO: Self = Self(0);

    pub fn as_usize(self) -> usize {
        self.0
    }

    pub fn as_f64(self) -> f64 {
        self.0 as f64
    }
}

impl From<usize> for SampleCount {
    fn from(value: usize) -> Self {
        Self(value)
    }
}

impl fmt::Display for SampleCount {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}
