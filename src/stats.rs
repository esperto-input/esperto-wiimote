use crate::points::Vec3;
use std::ops::{AddAssign, Mul, MulAssign};
use std::time::{Duration, Instant};

// Time-Weighted Exponential Moving Average
pub struct TWEMA {
   pub averages: Vec3,
   pub variances: Vec3,
   pub last: Instant,
}

impl TWEMA {
   pub const TAU: f32 = Duration::from_secs(10).as_millis_f32();

   pub fn new(now: Instant, initial: Vec3) -> Self {
      TWEMA {
         averages: initial,
         variances: Default::default(),
         last: now,
      }
   }

   pub fn add_value(&mut self, value: Vec3, now: Instant) {
      let old_average = self.averages;

      let w = (-now.duration_since(self.last).as_millis_f32() / TWEMA::TAU)
         .exp()
         .clamp(0.0, 1.0);
      self.last = now;

      self.averages = self.averages * w + value * (1.0 - w);
      self.variances = self.variances * w + (value - old_average).component_mul(&(value - self.averages)) * (1.0 - w);
   }

   pub fn sd(&self) -> Vec3 {
      self.variances.map(f32::sqrt)
   }
}

// Time-Weighted Bounded Exponential Moving Average
#[derive(Debug, Clone, Copy)]
pub struct TWBEMA<T: Copy + Default + AddAssign<T> + MulAssign<f32> + Mul<f32, Output = T>> {
   pub average: T,
   last: T,
   timestamp: Instant,
}

impl<T: Copy + Default + AddAssign<T> + MulAssign<f32> + Mul<f32, Output = T>> Default for TWBEMA<T> {
   fn default() -> Self {
      Self {
         average: T::default(),
         last: T::default(),
         timestamp: Instant::now(),
      }
   }
}

impl<T: Copy + Default + AddAssign<T> + MulAssign<f32> + Mul<f32, Output = T>> TWBEMA<T> {
   pub fn add_value(&mut self, value: T, timestamp: Instant) {
      let elapsed = (timestamp.duration_since(self.timestamp).as_secs_f32() / TWEMA_MAX_ELAPSED_TIME).clamp(0.0, 1.0);
      let weight = TWEMA_WEIGHT_LOWER_BOUND + (elapsed * (TWEMA_WEIGHT_UPPER_BOUND - TWEMA_WEIGHT_LOWER_BOUND));
      self.average *= 1.0 - weight;
      self.average += value * weight;
      self.last = value;
      self.timestamp = timestamp;
   }
}

// Welford/Exponential Moving Average & Variance
#[derive(Debug, Clone, Copy, Default)]
pub struct WEMAV {
   pub average: f32,
   count: f32,
   pub variance: f32,
}

impl WEMAV {
   pub fn add_value(&mut self, value: f32) {
      let old_avg = self.average;
      self.count += (self.count < WELFORD_MAX_COUNT) as u8 as f32;

      self.average += (value - self.average) / (self.count);
      self.variance =
         self.variance * (self.count - 1.0) / self.count + (value - old_avg) * (value - self.average) / self.count;
   }

   pub fn sd(&self) -> f32 {
      self.variance.sqrt()
   }
}

// maximum alpha for twema
const TWEMA_WEIGHT_UPPER_BOUND: f32 = 0.85;
// minimum alpha for twema
const TWEMA_WEIGHT_LOWER_BOUND: f32 = 0.2;
// time mapped to maximum alpha
const TWEMA_MAX_ELAPSED_TIME: f32 = 0.25;
// count for switching to exponential averaging
const WELFORD_MAX_COUNT: f32 = 30_000.0;
