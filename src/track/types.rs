use crate::{dprintln, irprintln};
use crate::points::Dot;
use crate::track::{TWEMA_MAX_ELAPSED_TIME, TWEMA_WEIGHT_LOWER_BOUND, TWEMA_WEIGHT_UPPER_BOUND, WELFORD_MAX_COUNT};
use ordered_float::OrderedFloat;
use std::cmp::{max_by_key, min, min_by_key};
use std::ops::{AddAssign, Mul, MulAssign};
use std::time::Instant;
use crate::track::print_utils::sensorbar_pane;

// Time-Weighted Exponential Moving Average
#[derive(Debug, Clone, Copy)]
pub struct TWEMA<T: Copy + Default + AddAssign<T> + MulAssign<f32> + Mul<f32, Output = T>> {
   pub average: T,
   last: T,
   timestamp: Instant,
}

impl<T: Copy + Default + AddAssign<T> + MulAssign<f32> + Mul<f32, Output = T>> Default for TWEMA<T> {
   fn default() -> Self {
      Self {
         average: T::default(),
         last: T::default(),
         timestamp: Instant::now(),
      }
   }
}

impl<T: Copy + Default + AddAssign<T> + MulAssign<f32> + Mul<f32, Output = T>> TWEMA<T> {
   pub fn add_value(&mut self, value: T, timestamp: Instant) {
      let elapsed = (timestamp.duration_since(self.timestamp).as_secs_f32() / TWEMA_MAX_ELAPSED_TIME).clamp(0.0, 1.0);
      let weight = TWEMA_WEIGHT_LOWER_BOUND + (elapsed * (TWEMA_WEIGHT_UPPER_BOUND - TWEMA_WEIGHT_LOWER_BOUND));
      self.average *= 1.0 - weight;
      self.average += value * weight;
      self.last = value;
      self.timestamp = timestamp;
      // dprintln!("Elapsed: {}; Weight: {}", elapsed, weight);
      // dprintln!("Gate value: {}", self.gate);
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

   pub fn standard_deviation(&self) -> f32 {
      self.variance.sqrt()
   }
}

// information on one IR dot from the Wiimote
#[derive(Clone, Copy, Debug)]
pub struct RawDot {
   pub x: i32, // X coordinate (0-1023)
   pub y: i32, // Y coordinate (0-768)
}

impl RawDot {
   pub fn is_valid(&self) -> bool {
      self.x != 1023 || self.y != 1023
   }
}

impl Into<Dot> for RawDot {
   fn into(self) -> Dot {
      Dot {
         x: 512.0 - self.x as f32,
         y: 384.0 - self.y as f32,
      } / 512.0
   }
}

impl Default for RawDot {
   fn default() -> Self {
      Self { x: 1023, y: 1023 }
   }
}

#[derive(Clone, Copy, Eq, PartialEq, Default)]
pub enum IRState {
   #[default]
   DEAD,
   GOOD,
   SINGLE,
   LOST,
}

pub struct BarDotGuess<'a> {
   pub dot: &'a Dot,
   #[cfg(feature = "tuning")]
   pub i: usize,
   pub closest: usize,
   pub distance: f32,
}

// holds state information on the sensor bar
#[derive(Clone, Copy, Default)]
pub struct SensorBar {
   dots: [Dot; 2],
   flat_dots: [Dot; 2],
}

impl SensorBar {
   pub fn set_order_dots(&mut self, dot1: &Dot, dot2: &Dot) -> f32 {
      if dot1.x <= dot2.x {
         self.dots = [*dot1, *dot2];
      } else {
         self.dots = [*dot2, *dot1];
      }
      self.recompute_flat_dots()
   }

   pub fn set_by_key_dots(&mut self, key: usize, main: &Dot, other: &Dot) -> f32 {
      self.dots[key] = *main;
      self.dots[key ^ 1] = *other;
      self.recompute_flat_dots()
   }

   pub fn find_closest<'a>(&self, dots: &'a [Dot]) -> BarDotGuess<'a> {
      dots
         .iter()
         .enumerate()
         .map(|(i, dot)| {
            let (distance, closest) = min(
               (OrderedFloat(dot.distance(self.left())), 0usize),
               (OrderedFloat(dot.distance(self.right())), 1usize),
            );
            BarDotGuess {
               dot,
               #[cfg(feature = "tuning")]
               i,
               closest,
               distance: distance.into(),
            }
         })
         .min_by_key(|guess| OrderedFloat(guess.distance))
         .unwrap()
   }

   pub fn align_to(&mut self, i: usize, dot: &Dot) {
      self.set_by_key_dots(i, dot, &(self.other(i) + self.dot(i).offset(dot)));
   }

   pub fn align_furthest(&mut self, guess: &Dot, roll: f32) -> usize{
      // try dot as both places in the sensor bar
      // and pick the one that places the other dot furthest off-screen
      let (other, i) = max_by_key(
         (guess + self.offset(), 0usize),
         (guess - self.offset(), 1usize),
         |(bardot, _)| OrderedFloat(bardot.rotate(-roll).norm()),
      );
      self.set_by_key_dots(i, guess, &other);
      irprintln!("IR: dot is {}", if i == 0 { "LEFT" } else { "RIGHT" });
      i
   }

   pub fn left(&self) -> &Dot {
      &self.dots[0]
   }

   pub fn right(&self) -> &Dot {
      &self.dots[1]
   }

   pub fn other(&self, i: usize) -> &Dot {
      &self.dots[i ^ 1]
   }

   pub fn flat_left(&self) -> &Dot {
      &self.flat_dots[0]
   }

   pub fn flat_right(&self) -> &Dot {
      &self.flat_dots[1]
   }

   pub fn slope(&self) -> f32 {
      self.left().slope(self.right())
   }

   pub fn offset(&self) -> Dot {
      self.left().offset(self.right())
   }

   pub fn flat_offset(&self) -> Dot {
      self.flat_left().offset(self.flat_right())
   }

   #[allow(unused)]
   pub fn distance(&self) -> f32 {
      self.left().distance(self.right())
   }

   #[allow(unused)]
   pub fn flat_distance(&self) -> f32 {
      self.flat_left().distance(self.flat_right())
   }

   pub fn dot(&self, i: usize) -> &Dot {
      &self.dots[i]
   }

   pub fn position(&self) -> Dot {
      (self.flat_left() + self.flat_right() + 2.0) / 4.0
   }

   pub fn iter(&self) -> impl Iterator<Item = &Dot> {
      self.dots.iter()
   }

   #[allow(unused)]
   pub fn iter_flat(&self) -> impl Iterator<Item = &Dot> {
      self.flat_dots.iter()
   }

   fn recompute_flat_dots(&mut self) -> f32 {
      let off_angle = self.offset().y.atan2(self.offset().x);
      self.flat_dots = [self.left().rotate(-off_angle), self.right().rotate(-off_angle)];
      off_angle
   }
}
