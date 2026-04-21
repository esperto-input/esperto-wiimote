use super::{accprintln, dprintln, irprint, irprintln};
use crate::config::Config;
use crate::points::Dot;
use crate::points::Vec3;
use crate::track::print_utils::{acc_pane, sensorbar_pane, smooth_pane};
use nalgebra::{OMatrix, SMatrix, matrix, vector};
use ordered_float::OrderedFloat;
use proc_macros::process;
use std::time::Instant;
pub use types::RawDot;
use types::{BarDotGuess, IRState, SensorBar, TWEMA, WEMAV, square};

pub mod print_utils;
mod types;

#[derive(Clone, Copy, Debug, Default)]
struct ACC {
   gravity: WEMAV,
   smoothed: TWEMA<Vec3>, // you cannot average an angle, but you can average coordinates
   roll: f32,             // roll from accelerometer (rotation) in radians
   corrected: Dot,
   calibration: SMatrix<f32, 3, 4>,
}

impl ACC {
   pub fn new(config: &Config) -> Self {
      ACC {
         calibration: SMatrix::<f32, 3, 4>::from_column_slice(&config.accelerometer_calibration),
         ..ACC::default()
      }
   }

   fn process(&mut self, data: Vec3) {
      let data = vector![data.x, data.y, data.z, 1.0];
      let data = self.calibration * data;
      let data = Vec3::new(data.x, data.y, data.z);
      let old = self.smoothed.average;

      // smooth coordinates
      self.smoothed.add_value(data, Instant::now());
      let acc = self.smoothed.average.norm();

      let dist = (self.smoothed.average - old).norm();

      let gate_test: bool = dist < GATE_THRESHOLD;
      if gate_test {
         // the wiimote is almost not moving, we are measuring gravity
         // we don't want to compose multiple averages
         self.gravity.add_value(acc);
      }
      accprintln!("ACC: gate value: {:.3}", dist);
      accprintln!("ACC: gravity value: {:.3}", self.gravity.average);
      accprintln!("ACC: gravity standard-deviation: {:.3}", self.gravity.sd());
      accprintln!("ACC: acceleration difference {:.3}", (self.gravity.average - acc).abs());

      // percentage of gravity, plus standard deviation
      let acc_threshold = (self.gravity.average + self.gravity.sd()) * ACCELERATION_THRESHOLD;
      let dist_threshold = (self.gravity.average + self.gravity.sd()) * DIST_THRESHOLD;
      accprintln!(
         "ACC: rot threshold: {:.3}, acc_threshold: {:.3}",
         dist_threshold,
         acc_threshold
      );
      accprintln!("ACC: smoothed acceleration value: {:.3}", acc);

      let acc_tan = ((self.gravity.average - acc).abs() / acc_threshold).tanh();
      let dist_tan = (dist / dist_threshold).tanh();
      let alpha = f32::max(acc_tan, dist_tan);

      accprintln!("ACC: acc_tan: {:.3}", acc_tan);
      accprintln!("ACC: dist_tan: {:.3}", dist_tan);
      accprintln!("ACC: alpha: {:.3}", alpha);

      self.corrected = (self.corrected * alpha)
         + (Dot {
            x: self.smoothed.average.z,
            y: self.smoothed.average.x,
         } * (1.0 - alpha));
      self.roll = -self.corrected.atan2();
      accprintln!("ACC: roll: {:.3}", self.roll.to_degrees());

      #[cfg(feature = "tuning")]
      let acc_test: bool = (self.gravity.average - acc).abs() <= acc_threshold;
      #[cfg(feature = "tuning")]
      let dist_test: bool = dist <= dist_threshold;
      acc_pane::line1!(self.gravity.sd(), self.gravity.average, data, gate_test);
      acc_pane::line2!(acc_threshold, acc, self.smoothed.average, acc_test);
      acc_pane::line3!(dist_threshold, dist, acc_tan, dist_tan, alpha, dist_test);
      acc_pane::status!(self.roll, acc_test && dist_test);
   }
}

#[derive(Clone, Copy, Default)]
struct IR {
   state: IRState,
   raw_position: Dot, // raw XY coordinate (0..1, 0.5 is center)
   // distance: f32,     // pixel width of the sensor bar
   z: f32, // wiimote to sensor bar distance in meters
   sensorbar: SensorBar,
   position: Option<Dot>, // smoothed XY coordinate
   errors_left: u32,      // error count from smoothing algorithm
   glitch_cnt: u32,       // glitch count from smoothing algorithm
}

impl IR {
   fn new() -> Self {
      IR::default()
   }

   fn find_edge_dot<'a>(&self, raw_dots: &'a [Dot]) -> (usize, &'a Dot) {
      // find the dot closest to the sensor edge
      raw_dots
         .iter()
         .enumerate()
         .max_by_key(|(_, dot)| OrderedFloat(dot.norm2()))
         .unwrap()
   }

   fn track_single_adjust(&mut self, roll: f32, sb: &mut SensorBar, guess: &BarDotGuess) -> bool {
      sb.align_guess(&self.sensorbar, guess);
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

   fn track_sensorbar(&self, dots: &[Dot], sb: &mut SensorBar) -> bool {
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
            if !cand.angle_check() {
               irprintln!("\tfailed angle check");
               continue;
            }
            irprintln!("\tpassed angle check");
            if !cand.distance_check() {
               irprintln!("\tfailed distance check");
               continue;
            }
            irprintln!("\tpassed distance check");

            // middle dot check. If there's another source somewhere in the
            // middle of this candidate, then this can't be a sensor bar
            if dots
               .iter()
               .enumerate()
               .any(|(i, dot)| i != first && i != second && !cand.bounds_check(off_angle, dot))
            {
               irprintln!("\tfailed middle dot check");
               continue;
            }
            irprintln!("\tpassed middle dot check");
            // pick the candidate with the smallest distance
            if cand.offset().x < min_distance {
               irprintln!("\tnew best");
               ind = (first, second);
               min_distance = cand.offset().x;
               *sb = cand;
               found = true;
            }
         }
      }
      sensorbar_pane::double!(found, ind.0, ind.1, sb.angle(), min_distance);
      found
   }

   fn track(&mut self, roll: f32, raw_dots: &mut [RawDot; 4]) -> bool {
      // count visible dots and populate dots structure
      // dots[] is in -1..1 units for width
      let num_dots = raw_dots.iter_mut().partition_in_place(|dots| dots.is_valid());
      sensorbar_pane::raw_dots!(&raw_dots);

      if num_dots == 0 {
         if self.state != IRState::DEAD {
            self.state = IRState::LOST;
         }
         self.raw_position = Dot::default();
         self.z = 0.0;
         sensorbar_pane::lost!();
         return false;
      }

      // first rotate according to accelerometer orientation
      let raw_dots: [Dot; 4] = raw_dots.map(|raw_dot| raw_dot.into());
      let dots = &raw_dots.map(|dot| dot.rotate(roll))[..num_dots];
      let raw_dots = &raw_dots[..num_dots];
      sensorbar_pane::dots!(raw_dots, dots);

      let mut new_sb = SensorBar::default();

      if self.track_sensorbar(dots, &mut new_sb) {
         irprintln!(
            "IR: sb d:{:.3} a:{:.3}°",
            new_sb.flat_distance(),
            new_sb.angle().to_degrees()
         );
         self.state = IRState::GOOD;
         self.sensorbar = new_sb;
      } else {
         // no sensor bar candidates, try to work with a lone dot
         irprintln!("IR: no candidates");
         if self.state == IRState::DEAD {
            sensorbar_pane::dead!();
            irprintln!("IR: no sensor bar reference");
            // we've never seen a sensor bar before, so we're screwed
            return false;
         }
         irprintln!("IR: track single dot");
         // try to find the dot closest to the previous sensor bar position
         let guess = self.sensorbar.find_closest(dots);
         if (self.state != IRState::LOST || guess.dist2 < SB_SINGLE_ADJUST_DISTANCE)
            && self.track_single_adjust(roll, &mut new_sb, &guess)
         {
            irprintln!(
               "IR: kept track of single {} dot",
               if guess.closest == 0 { "LEFT" } else { "RIGHT" }
            );
            sensorbar_pane::single_adjust!(guess.i, guess.closest);
            self.sensorbar = new_sb;
         } else {
            irprintln!("IR: adjust skipped");
            let (i, dot) = self.find_edge_dot(raw_dots);
            let bardot = self.sensorbar.align_furthest(dot, roll);
            sensorbar_pane::single_lost!(i, bardot);
         }
         self.state = IRState::SINGLE;
         sensorbar_pane::single!();
      }
      self.raw_position = self.sensorbar.flat_avg();
      self.z = SB_Z_COEFFICIENT / (self.sensorbar.flat_offset().x * 512.0);
      true
   }

   fn smooth(&mut self, position: &Dot, dist2: f32) {
      irprint!(
         "SMT: {}~:{:?} ",
         self.raw_position,
         self.position.map_or_else(|| "None".to_string(), |p| p.to_string())
      );
      let diff = position.offset(&self.raw_position);
      if dist2 > SMOOTH_IR_DEADZONE {
         if dist2 < SMOOTH_IR_RADIUS.powi(2) {
            self.position = Some(position + diff * SMOOTH_IR_SPEED);
            irprintln!("inside");
            smooth_pane::inside!(dist2);
         } else {
            let theta = diff.atan2();
            self.position = Some(
               self.raw_position
                  - Dot {
                     x: theta.cos(),
                     y: theta.sin(),
                  } * SMOOTH_IR_RADIUS,
            );
            irprintln!("outside");
            smooth_pane::outside!(dist2);
         }
         return;
      }
      irprintln!("deadzone");
      smooth_pane::deadzone!(dist2);
   }

   fn process(&mut self, roll: f32, raw_dots: &mut [RawDot; 4]) {
      let raw_valid = self.track(roll, raw_dots);

      if raw_valid {
         if self.errors_left > 0 {
            let position = &self.position.unwrap();
            let dist2 = self.raw_position.distance2(position);
            if dist2 <= GLITCH_DIST || self.glitch_cnt > GLITCH_MAX_COUNT {
               self.glitch_cnt = 0;
               self.smooth(position, dist2);
            } else {
               self.glitch_cnt += 1;
            }
         } else {
            self.position = Some(self.raw_position);
            self.glitch_cnt = 0;
         }
         self.errors_left = ERROR_MAX_COUNT;
      } else {
         if self.errors_left > 0 {
            self.errors_left -= 1;
         } else {
            self.position = None;
         }
      }
      smooth_pane::counters!(self.errors_left, self.glitch_cnt);
   }
}

#[derive(Clone, Copy, Default)]
pub struct Tracker {
   ir: IR,
   acc: ACC,
}

impl Tracker {
   pub fn new(config: &Config) -> Tracker {
      Tracker {
         ir: IR::new(),
         acc: ACC::new(config),
      }
   }

   pub fn process_ir_data(&mut self, mut raw_dots: [RawDot; 4]) {
      sensorbar_pane::begin!();
      smooth_pane::begin!();
      self.ir.process(self.acc.roll, &mut raw_dots);
      sensorbar_pane::end!();
      smooth_pane::end!();
   }

   pub fn process_accelerometer_data(&mut self, data: Vec3) {
      acc_pane::begin!();
      // SVD puro
      // let correction = matrix![
      //          1.0022228,  -0.0146525735,   -0.008091033 ,     28.390265 ;
      //   -0.013285754   ,   1.0142968   ,-0.009260553    ,  33.626842 ;
      // -0.00021422282   ,0.0007917918   ,   0.9951243    ,   28.16185;
      //        ];

      // soluzione ibrida
      //     let correction = matrix![
      //      1.0022228 , -0.0146525735,   -0.008091033,      29.093016 ;
      //   -0.013285754 ,     1.0142968,   -0.009260553,      33.692974 ;
      // -0.00021422282 ,  0.0007917918,      0.9951243,      29.718075 ;
      //     ];
      // seconda calibrazione
      /*let correction = matrix![
         0.010226312,  -0.00026041837, -0.000097612996,      0.29604778;
      -0.00017977913,     0.010307922,  -0.00015668989,      0.34658885;
      0.000029927236,  0.000102972146,     0.010124174,      0.29518628;
        ];

      accprintln!("{}", correction);
      // let data = data-Vec3 { x: -29.732002, y: -33.8565, z: -29.842999 };
      let data = vector![data.x, data.y, data.z, 1.0];
      let corrected = correction * data;
      let data = Vec3::new(corrected.x, corrected.y, corrected.z);*/

      self.acc.process(
         data, // Vec3 { x: -27.57731246399373, y: -32.8657151306604, z: -28.154628797327067 }
              /*- Vec3 {
                 x: -29.7,
                 y: -34.0,
                 z: -30.0,
              }*/
              // - Vec3 { x: -29.049667, y: -33.7915, z: -28.279167 },
              // - Vec3 { x: -29.732002, y: -33.8565, z: -29.842999 },
              // - Vec3 { x: -29.731003, y: -33.856403, z: -29.84071 },
              // - Vec3 { x: -29.049667, y: -33.7915, z: -28.279165 },
              // - Vec3 { x: -29.220253, y: -33.80775, z: -28.670122 },
              // - Vec3 { x: -0.70275116, y: -0.06613159, z: -1.5562248 },
      );
      acc_pane::end!();
   }

   pub fn get_position(&self) -> Option<Dot> {
      self.ir.position.as_ref().map(Dot::position)
   }
}

// cm center to center of emitters
const SB_WIDTH: f32 = 19.5;

// width in cm of emitters
const SB_DOT_CLUSTER_WIDTH: f32 = 4.5;

// height in cm of emitters (with some tolerance)
const SB_DOT_CLUSTER_HEIGHT: f32 = 1.0;

// maximum sensor bar slope in degrees
#[process(MAX_SB_SLOPE.to_radians())]
const MAX_SB_SLOPE: f32 = 35.0;

// minimum sensor bar width in units, relative to half of the IR sensor area
#[process(MIN_SB_WIDTH / 1023.0)]
const MIN_SB_WIDTH: f32 = 100.0;

// dots further out than these coords are allowed to not be picked up
// otherwise assume something's wrong
// disabled, may be doing more harm than good due to sensor pickup glitches
#[process(SB_OFF_SCREEN_X / 1023.0)]
const SB_OFF_SCREEN_X: f32 = 0.0;
#[process(SB_OFF_SCREEN_Y / 1023.0)]
const SB_OFF_SCREEN_Y: f32 = 0.0;

// if a point is closer than this to one of the previous SB points
// when it reappears, consider it the same instead of trying to guess
// which one of the two it is
#[process(square!(SB_SINGLE_ADJUST_DISTANCE / 1023.0))]
const SB_SINGLE_ADJUST_DISTANCE: f32 = 100.0;

// width of the sensor bar in pixels at one meter from the Wiimote
const SB_Z_COEFFICIENT: f32 = 256.0;

// distance in meters from the center of the FOV to the left or right edge,
// when the wiimote is at one meter
// const WIIMOTE_FOV_COEFFICIENT: f32 = 0.39;

#[process(SMOOTH_IR_RADIUS / 1023.0)]
const SMOOTH_IR_RADIUS: f32 = 30.0;
const SMOOTH_IR_SPEED: f32 = 0.15;
#[process(square!(SMOOTH_IR_DEADZONE / 1023.0))]
const SMOOTH_IR_DEADZONE: f32 = 8.0;

// max number of errors before cooked data drops out
const ERROR_MAX_COUNT: u32 = 1;

// max number of glitches before cooked data updates
const GLITCH_MAX_COUNT: u32 = 5;

// squared delta over which we consider something a glitch
#[process(square!(GLITCH_DIST / 1023.0))]
const GLITCH_DIST: f32 = 150.0;

// maximum alpha for twema
const TWEMA_WEIGHT_UPPER_BOUND: f32 = 0.85;

// minimum alpha for twema
const TWEMA_WEIGHT_LOWER_BOUND: f32 = 0.2;

// time mapped to maximum alpha
const TWEMA_MAX_ELAPSED_TIME: f32 = 0.25;

// threshold for accepting gravity readings
const GATE_THRESHOLD: f32 = 1.0;

// distance threshold at which rotation readings are almost capped
// in units of gravity
const DIST_THRESHOLD: f32 = 0.15;

// acceleration threshold at which rotation readings are almost capped
// in units of gravity
const ACCELERATION_THRESHOLD: f32 = 0.13;

// count for switching to exponential averaging
const WELFORD_MAX_COUNT: f32 = 30_000.0;
