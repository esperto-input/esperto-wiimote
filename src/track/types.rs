use crate::irprintln;
use crate::points::{Dot, DotLike};
use crate::track::{
   MAX_SB_SLOPE, MIN_SB_WIDTH
   ,
};
use nalgebra::vector;
use ordered_float::OrderedFloat;
use std::cmp::{max_by_key, min};
#[derive(Clone, Copy, Eq, PartialEq, Default)]
pub enum IRState {
   #[default]
   DEAD,
   GOOD,
   SINGLE,
   LOST,
}

#[derive(Clone, Copy, Debug)]
pub struct BarDotGuess<'a> {
   pub dot: &'a Dot,
   #[cfg(feature = "tuning")]
   pub i: usize,
   pub closest: usize,
   pub dist2: f32,
}

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
      vector![self.x as f32 - 512.0, self.y as f32 - 384.0,] / 512.0
   }
}

impl Default for RawDot {
   fn default() -> Self {
      Self { x: 1023, y: 1023 }
   }
}

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

   pub fn align_guess(&mut self, reference: &SensorBar, guess: &BarDotGuess) {
      self.set_by_key_dots(
         guess.closest,
         guess.dot,
         &(reference.other(guess.closest) + reference.dot(guess.closest).offset(guess.dot)),
      );
   }

   pub fn align_furthest(&mut self, guess: &Dot, roll: f32) -> usize {
      // try dot as both places in the sensor bar
      // and pick the one that places the other dot furthest off-screen
      let (other, i) = max_by_key(
         (guess + self.offset(), 0usize),
         (guess - self.offset(), 1usize),
         |(bardot, _)| OrderedFloat(bardot.rotate(-roll).norm2()),
      );
      self.set_by_key_dots(i, guess, &other);
      irprintln!("IR: dot is {}", if i == 0 { "LEFT" } else { "RIGHT" });
      i
   }

   pub fn find_closest<'a>(&self, dots: &'a [Dot]) -> BarDotGuess<'a> {
      #[allow(unused_variables)]
      dots
         .iter()
         .enumerate()
         .map(|( i, dot)| {
            let (dist2, closest) = min(
               (OrderedFloat(dot.distance2(self.left())), 0usize),
               (OrderedFloat(dot.distance2(self.right())), 1usize),
            );
            BarDotGuess {
               dot,
               #[cfg(feature = "tuning")]
               i,
               closest,
               dist2: dist2.into(),
            }
         })
         .min_by_key(|guess| OrderedFloat(guess.dist2))
         .unwrap()
   }

   pub fn angle_check(&self) -> bool {
      self.angle().abs() <= MAX_SB_SLOPE
   }

   pub fn distance_check(&self) -> bool {
      self.flat_distance() >= MIN_SB_WIDTH
   }

   pub fn bounds_check(&self, off_angle: f32, dot: &Dot, sensorbar_size: &SensorBarSize) -> bool {
      let margin = vector![sensorbar_size.cluster_width, sensorbar_size.cluster_height,] / 2.0 / sensorbar_size.width * self.offset().x;
      let flat_dot = dot.rotate(off_angle);
      let nw = self.flat_left() + margin;
      let se = self.flat_right() - margin;
      flat_dot.x <= nw.x || flat_dot.y >= nw.y || flat_dot.x >= se.x || flat_dot.y <= se.y
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

   pub fn angle(&self) -> f32 {
      self.left().off_angle(self.right())
   }

   pub fn offset(&self) -> Dot {
      self.left().offset(self.right())
   }

   pub fn flat_offset(&self) -> Dot {
      self.flat_left().offset(self.flat_right())
   }

   pub fn flat_distance(&self) -> f32 {
      self.flat_offset().x
   }

   pub fn dot(&self, i: usize) -> &Dot {
      &self.dots[i]
   }

   pub fn flat_avg(&self) -> Dot {
      self.flat_left().avg(self.flat_right())
   }

   fn recompute_flat_dots(&mut self) -> f32 {
      let off_angle = self.offset().y.atan2(self.offset().x);
      self.flat_dots = [self.left().rotate(-off_angle), self.right().rotate(-off_angle)];
      off_angle
   }
}

macro_rules! square {
   ($arg:expr) => {
      (($arg) * ($arg))
   };
}

pub(crate) use square;
use crate::config::SensorBarSize;
