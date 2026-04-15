use super::{accprintln, dprintln, irprint, irprintln};
use crate::points::Dot;
use crate::points::Vec3;
use crate::track::print_utils::{acc_pane, sensorbar_pane};
use ordered_float::OrderedFloat;
use std::time::Instant;
pub use types::RawDot;
use types::{BarDotGuess, IRState, SensorBar, TWEMA, WEMAV};

mod print_utils;
mod types;

#[derive(Clone, Copy, Debug, Default)]
struct ACC {
   gravity: WEMAV,
   smoothed: TWEMA<Vec3>, // you cannot average an angle, but you can average coordinates
   roll: f32,             // roll from accelerometer (rotation) in radians
}

impl ACC {
   fn process(&mut self, data: Vec3) {
      let dist = (self.smoothed.average - data).norm();

      if dist < GATE_THRESHOLD {
         // the wiimote is almost not moving, we are measuring gravity
         // we don't want to compose multiple averages
         self.gravity.add_value(data.norm());
      }
      accprintln!("Acc: Gate value: {:?}", dist);
      accprintln!("Acc: Gravity value: {:?}", self.gravity.average);
      accprintln!(
         "Acc: Gravity standard-deviation: {:?}",
         self.gravity.standard_deviation()
      );

      // percentage of gravity, plus standard deviation
      let acc_threshold = self.gravity.average * ACCELERATION_THRESHOLD + self.gravity.standard_deviation();
      let dist_threshold = self.gravity.average * ROTATION_THRESHOLD + self.gravity.standard_deviation();
      accprintln!(
         "Acc: rot threshold: {}, acc_threshold: {}",
         dist_threshold,
         acc_threshold
      );

      // smooth coordinates
      self.smoothed.add_value(data, Instant::now());
      let acc = self.smoothed.average.norm();
      accprintln!("Acc: Smoothed acceleration value: {:?}", acc);

      if (self.gravity.average - acc).abs() <= acc_threshold && dist <= dist_threshold {
         // the wiimote is not accelerating excessively (acceleration similar to gravity)
         // the wiimote is also not quickly rotating
         self.roll = (-self.smoothed.average.x).atan2(self.smoothed.average.z);
         accprintln!("Acc: Roll value: {}", self.roll);
      }
   }
}

#[derive(Clone, Copy, Default)]
struct IR {
   state: IRState,
   position: Dot, // raw XY coordinate (-512..512, 0 is center)
   distance: f32, // pixel width of the sensor bar
   z: f32,        // wiimote to sensor bar distance in meters
   sensorbar: SensorBar,
}

impl IR {
   fn find_edge_dot<'a>(&self, raw_dots: &'a [Dot]) -> (usize, &'a Dot) {
      // find the dot closest to the sensor edge
      raw_dots
         .iter()
         .enumerate()
         .max_by_key(|(_, dot)| OrderedFloat(dot.norm()))
         .unwrap()
   }

   fn track_single_adjust(&mut self, roll: f32, sb: &mut SensorBar, guess: &BarDotGuess) -> bool {
      sb.align_to(guess.closest, guess.dot);
      // compute the raw frame with the inverse rotation
      let raw_dot = sb.other(guess.closest).rotate(-roll);
      if (raw_dot.x.abs() < SB_OFF_SCREEN_X) && (raw_dot.y.abs() < SB_OFF_SCREEN_Y) {
         // this dot should be visible but isn't, since the
         // candidate section failed. fall through and try to
         // pick out the sensor bar without previous information
         irprintln!("IR: dot falls on screen, abort");
         return false;
      }
      true
   }

   fn track_sensorbar(&self, roll: f32, dots: &[Dot], sb: &mut SensorBar) -> bool {
      let mut cand: SensorBar = *sb;
      let mut min_distance = f32::INFINITY;
      let mut found = false;
      let mut ind: (usize, usize) = (0, 0);

      // iterate through all dot pairs
      for first in 0..dots.len() - 1 {
         for second in first + 1..dots.len() {
            irprintln!("IR: trying dots {} and {}", first, second);
            // order the dots leftmost first into cand
            // storing both the raw dots and the accel-rotated dots
            let off_angle = cand.set_order_dots(&dots[first], &dots[second]);
            // check angle
            if cand.slope().abs() > MAX_SB_SLOPE.to_radians() {
               irprintln!("\tfailed angle check");
               continue;
            }
            irprintln!("\tpassed angle check");
            // check distance
            if cand.offset().x < MIN_SB_WIDTH / 1023.0 {
               irprintln!("\tfailed distance check");
               continue;
            }
            irprintln!("\tpassed distance check");
            // middle dot check. If there's another source somewhere in the
            // middle of this candidate, then this can't be a sensor bar
            let margin = Dot {
               x: SB_DOT_CLUSTER_WIDTH,
               y: SB_DOT_CLUSTER_HEIGHT,
            } / 2.0
               / SB_WIDTH
               * cand.offset().x;
            if dots.iter().enumerate().any(|(i, dot)| {
               i != first && i != second && {
                  let upper = cand.flat_left() + margin;
                  let lower = cand.flat_right() - margin;
                  let flat_dot = dot.rotate(off_angle);
                  flat_dot.x > upper.x && flat_dot.y < upper.y && flat_dot.x < lower.x && flat_dot.y > lower.y
               }
            }) {
               irprintln!("\tfailed middle dot check");
               continue;
            }
            irprintln!("\tpassed middle dot check");
            // pick the candidate with the smallest distance
            if cand.offset().x < min_distance {
               irprintln!("\tnew candidate");
               ind = (first, second);
               min_distance = cand.offset().x;
               *sb = cand;
               found = true;
            }
         }
      }
      sensorbar_pane::double(found, ind.0, ind.1, cand.slope(), min_distance);
      found
   }

   fn track(&mut self, raw_dots: &[RawDot; 4], roll: f32) -> bool {
      // count visible dots and populate dots structure
      // dots[] is in -1..1 units for width
      sensorbar_pane::raw_raw_dots(raw_dots);
      let mut raw_dots = raw_dots.clone();
      let num_dots = raw_dots.iter_mut().partition_in_place(|dots| dots.is_valid());
      sensorbar_pane::raw_dots(&raw_dots);

      if num_dots == 0 {
         if self.state != IRState::DEAD {
            self.state = IRState::LOST;
         }
         self.position = Dot::default();
         self.distance = 0.0;
         self.z = 0.0;
         sensorbar_pane::lost();
         return false;
      }

      // first rotate according to accelerometer orientation
      let raw_dots: [Dot; 4] = raw_dots.map(|raw_dot| raw_dot.into());
      let dots = &raw_dots.map(|dot| dot.rotate(roll))[..num_dots];
      let raw_dots = &raw_dots[..num_dots];
      sensorbar_pane::dots(raw_dots, dots);

      let mut new_sb = SensorBar::default();

      if self.track_sensorbar(roll, dots, &mut new_sb) {
         self.state = IRState::GOOD;
         self.sensorbar = new_sb;
      } else {
         // no sensor bar candidates, try to work with a lone dot
         irprintln!("IR: no candidates");
         if self.state == IRState::DEAD {
            sensorbar_pane::dead();
            irprintln!("IR: no sensor bar reference");
            // we've never seen a sensor bar before, so we're screwed
            return false;
         }
         irprintln!("IR: track single dot");
         // try to find the dot closest to the previous sensor bar position
         let guess = self.sensorbar.find_closest(dots);
         if (self.state != IRState::LOST || guess.distance < (SB_SINGLE_ADJUST_DISTANCE / 1023.0).powi(2))
            && self.track_single_adjust(roll, &mut new_sb, &guess)
         {
            irprintln!(
               "IR: kept track of single {} dot",
               if guess.closest == 0 { "LEFT" } else { "RIGHT" }
            );
            #[cfg(feature = "tuning")]
            sensorbar_pane::single_adjust(guess.i, guess.closest);
            self.sensorbar = new_sb;
         } else {
            irprintln!("IR: adjust skipped");
            let (i, dot) = self.find_edge_dot(raw_dots);
            let bardot = self.sensorbar.align_furthest(dot, roll);
            sensorbar_pane::single_lost(i, bardot);
         }
         self.state = IRState::SINGLE;
         sensorbar_pane::single();
      }
      self.position = self.sensorbar.position();
      self.distance = self.sensorbar.flat_offset().x * 512.0;
      self.z = SB_Z_COEFFICIENT / self.distance;
      true
   }
}

#[derive(Clone, Copy, Default)]
pub struct Tracker {
   ir: IR,
   acc: ACC,
   smooth_valid: bool, // is the smoothed position valid?
   smoothed: Dot,      // smoothed XY coordinate
   error_cnt: i32,     // error count from smoothing algorithm
   glitch_cnt: i32,    // glitch count from smoothing algorithm
}

impl Tracker {
   pub fn new() -> Tracker {
      Tracker {
         ir: IR::default(),
         acc: ACC::default(),
         smooth_valid: false,
         smoothed: Dot::default(),
         error_cnt: 0,
         glitch_cnt: ERROR_MAX_COUNT,
      }
   }

   fn apply_smoothing(&mut self) {
      irprint!(
         "SMT: ({:.2},{:.2})~({:.2},{:.2}) ",
         self.ir.position.x,
         self.ir.position.y,
         self.smoothed.x,
         self.smoothed.y
      );
      let diff = self.smoothed.offset(&self.ir.position);
      let dist = diff.norm();
      if dist > SMOOTH_IR_DEADZONE.powi(2) {
         if dist < SMOOTH_IR_RADIUS.powi(2) {
            irprintln!("INSIDE");
            self.smoothed += diff * SMOOTH_IR_SPEED;
         } else {
            irprintln!("OUTSIDE");
            let theta = diff.y.atan2(diff.x);
            self.smoothed = self.ir.position
               - Dot {
                  x: theta.cos(),
                  y: theta.sin(),
               } * SMOOTH_IR_RADIUS;
         }
      } else {
         irprintln!("DEADZONE");
      }
   }

   pub fn process_accelerometer_data(&mut self, data: Vec3) {
      acc_pane::begin();
      self.acc.process(
         data
            - Vec3 {
               x: -27.57731246399373,
               y: -32.8657151306604,
               z: -28.154628797327067,
            },
      );
      acc_pane::end();
   }

   pub fn process_ir_data(&mut self, raw_dots: &[RawDot; 4]) {
      sensorbar_pane::begin();
      let raw_valid = self.ir.track(raw_dots, self.acc.roll);
      sensorbar_pane::end();

      if raw_valid {
         if self.error_cnt >= ERROR_MAX_COUNT {
            self.smoothed = self.ir.position;
            self.glitch_cnt = 0;
         } else {
            let dist = self.ir.position.distance(&self.smoothed);
            if dist > GLITCH_DIST.powi(2) {
               if self.glitch_cnt > GLITCH_MAX_COUNT {
                  self.apply_smoothing();
                  self.glitch_cnt = 0;
               } else {
                  self.glitch_cnt += 1;
               }
            } else {
               self.glitch_cnt = 0;
               self.apply_smoothing();
            }
         }
         self.smooth_valid = true;
         self.error_cnt = 0;
      } else {
         if self.error_cnt >= ERROR_MAX_COUNT {
            self.smooth_valid = false;
         } else {
            self.smooth_valid = true;
            self.error_cnt += 1;
         }
      }
   }

   pub fn get_position(&self) -> Option<Dot> {
      if self.smooth_valid {
         let smoothed = Dot {
            x: self.smoothed.x.clamp(0.0, 1.0),
            y: (1.0 - self.smoothed.y).clamp(0.0, 1.0),
         };
         Some(smoothed)
      } else {
         None
      }
   }
}

// half-height of the IR sensor if half-width is 1
// const HEIGHT: f32 = 384.0 / 512.0;

// maximum sensor bar slope in degrees
const MAX_SB_SLOPE: f32 = 35.0;
// minimum sensor bar width in units, relative to half of the IR sensor area
const MIN_SB_WIDTH: f32 = 100.0;

// physical dimensions
// cm center to center of emitters
const SB_WIDTH: f32 = 19.5;
// width in cm of emitters
const SB_DOT_CLUSTER_WIDTH: f32 = 4.5;
// height in cm of emitters (with some tolerance)
const SB_DOT_CLUSTER_HEIGHT: f32 = 1.0;

// dots further out than these coords are allowed to not be picked up
// otherwise assume something's wrong
// disabled, may be doing more harm than good due to sensor pickup glitches
const SB_OFF_SCREEN_X: f32 = 0.0;
//#define SB_OFF_SCREEN_X 0.8f
const SB_OFF_SCREEN_Y: f32 = 0.0;
//#define SB_OFF_SCREEN_Y (0.8f * HEIGHT)

// if a point is closer than this to one of the previous SB points
// when it reappears, consider it the same instead of trying to guess
// which one of the two it is
const SB_SINGLE_ADJUST_DISTANCE: f32 = 100.0;

// width of the sensor bar in pixels at one meter from the Wiimote
const SB_Z_COEFFICIENT: f32 = 256.0;

// distance in meters from the center of the FOV to the left or right edge,
// when the wiimote is at one meter
// const WIIMOTE_FOV_COEFFICIENT: f32 = 0.39;

const SMOOTH_IR_RADIUS: f32 = 8.0 / 1023.0;
const SMOOTH_IR_SPEED: f32 = 0.17;
const SMOOTH_IR_DEADZONE: f32 = 1.7 / 1023.0;

// max number of errors before cooked data drops out
const ERROR_MAX_COUNT: i32 = 8;
// max number of glitches before cooked data updates
const GLITCH_MAX_COUNT: i32 = 5;
// squared delta over which we consider something a glitch
const GLITCH_DIST: f32 = 150.0 / 1023.0;

const TWEMA_WEIGHT_UPPER_BOUND: f32 = 0.8;

const TWEMA_WEIGHT_LOWER_BOUND: f32 = 0.1;

const TWEMA_MAX_ELAPSED_TIME: f32 = 0.25;

const GATE_THRESHOLD: f32 = 3.0;

const ROTATION_THRESHOLD: f32 = 0.1;

const ACCELERATION_THRESHOLD: f32 = 0.1;

const WELFORD_MAX_COUNT: f32 = 30_000.0;
