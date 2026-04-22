use crate::points::Vec3;
use crate::print_utils;
use crate::print_utils::calibration_pane;
use crate::routing::{make_abs_stream, Raw, SyncAccelEvents};
use crossterm::event;
use evdevil::Evdev;
use futures::StreamExt;
use nalgebra::{matrix, vector, Matrix3x4, Matrix4x3, SMatrix};
use std::time::{Duration, Instant};

struct TWEMA {
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

#[repr(usize)]
#[derive(Copy, Clone, Debug)]
pub enum Position {
   PosX = 0,
   NegX = 1,
   PosY = 2,
   NegY = 3,
   PosZ = 4,
   NegZ = 5,
}

struct Calibrate {
   start_time: Instant,
   averages: TWEMA,
}

impl Calibrate {
   const THRESHOLD: f32 = 1.4;

   pub fn sample(&mut self, new: Vec3) -> Option<Vec3> {
      calibration_pane::begin();

      self.averages.add_value(new, Instant::now());
      let sds = self.averages.sd();
      let sd = sds.max();

      let elapsed = self.averages.last.duration_since(self.start_time).as_millis_f32();
      if elapsed <= TWEMA::TAU {
         calibration_pane::warming_up(10 - (elapsed * 10.0 / TWEMA::TAU) as i32);
         calibration_pane::end();
         return None;
      }

      let progress = 20 - ((sd - Self::THRESHOLD).sqrt().clamp(0.0, 3.0) * (20.0 / 3.0)) as usize;
      calibration_pane::progress(progress);
      calibration_pane::sds(sds);
      calibration_pane::avgs(self.averages.averages);
      calibration_pane::end();

      if sd < Self::THRESHOLD {
         return Some(self.averages.averages);
      }
      None
   }
}

pub async fn process(accel_device: Evdev, position: Position) -> Result<(Evdev, Vec3), Box<dyn std::error::Error>> {
   calibration_pane::splash(position);
   while !event::read()?.is_key_press() {}

   let mut reader = accel_device.into_reader()?;
   'a: {
      let mut raw_stream = make_abs_stream(reader.async_events()?, Raw::AccelSyn).boxed();
      let sync = SyncAccelEvents::new();
      let stream = sync.to_stream(&mut raw_stream);
      tokio::pin!(stream);

      let mut cal;

      if let Some(event) = stream.next().await {
         let now = Instant::now();
         cal = Calibrate {
            start_time: now,
            averages: TWEMA::new(now, event),
         };
         loop {
            if let Some(event) = stream.next().await {
               if let Some(value) = cal.sample(event) {
                  break 'a Ok(value);
               }
            } else {
               break;
            }
         }
      }
      Err(std::io::Error::new(std::io::ErrorKind::BrokenPipe, "End of event stream").into())
   }
   .map(|value| (reader.into_evdev(), value))
}

pub async fn calibrate(mut accel_device: Evdev) -> Result<Matrix3x4<f32>, Box<dyn std::error::Error>> {
   let mut samples = [Vec3::default(); 6];

   print_utils::clear();

   (accel_device, samples[Position::PosZ as usize]) = process(accel_device, Position::PosZ).await?;
   (accel_device, samples[Position::NegZ as usize]) = process(accel_device, Position::NegZ).await?;
   (accel_device, samples[Position::PosX as usize]) = process(accel_device, Position::PosX).await?;
   (accel_device, samples[Position::NegX as usize]) = process(accel_device, Position::NegX).await?;
   (accel_device, samples[Position::PosY as usize]) = process(accel_device, Position::PosY).await?;
   (accel_device, samples[Position::NegY as usize]) = process(accel_device, Position::NegY).await?;
   drop(accel_device);

   calibration_pane::optimizing();
   let affine_matrix = optimize(
      samples[Position::PosX as usize],
      samples[Position::NegX as usize],
      samples[Position::PosY as usize],
      samples[Position::NegY as usize],
      samples[Position::PosZ as usize],
      samples[Position::NegZ as usize],
   );
   calibration_pane::done();

   Ok(affine_matrix)
}

fn optimize(pos_x: Vec3, neg_x: Vec3, pos_y: Vec3, neg_y: Vec3, pos_z: Vec3, neg_z: Vec3) -> Matrix3x4<f32> {
   let sample_matrix: SMatrix<f32, 18, 12> = matrix![
      pos_x.x, pos_x.y, pos_x.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     pos_x.x, pos_x.y, pos_x.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     pos_x.x, pos_x.y, pos_x.z, 1.0;
      neg_x.x, neg_x.y, neg_x.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     neg_x.x, neg_x.y, neg_x.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     neg_x.x, neg_x.y, neg_x.z, 1.0;
      pos_y.x, pos_y.y, pos_y.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     pos_y.x, pos_y.y, pos_y.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     pos_y.x, pos_y.y, pos_y.z, 1.0;
      neg_y.x, neg_y.y, neg_y.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     neg_y.x, neg_y.y, neg_y.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     neg_y.x, neg_y.y, neg_y.z, 1.0;
      pos_z.x, pos_z.y, pos_z.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     pos_z.x, pos_z.y, pos_z.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     pos_z.x, pos_z.y, pos_z.z, 1.0;
      neg_z.x, neg_z.y, neg_z.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     neg_z.x, neg_z.y, neg_z.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     neg_z.x, neg_z.y, neg_z.z, 1.0;
   ];

   let target_vector: SMatrix<f32, 18, 1> = vector![
      98.0, 0.0, 0.0, -98.0, 0.0, 0.0, 0.0, 98.0, 0.0, 0.0, -98.0, 0.0, 0.0, 0.0, 98.0, 0.0, 0.0, -98.0,
   ];

   let weight = 500.0;
   let weights = SMatrix::<f32, 18, 18>::from_diagonal(
      &vector![
         1.0 * weight,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0 * weight,
      ]
      .map(f32::sqrt),
   );

   let svd = (weights * sample_matrix).svd(true, true);
   let p = svd.solve(&(weights * target_vector), 1e-10).expect("SVD solve failed");
   let affine_matrix = Matrix4x3::from_column_slice(p.as_slice()).transpose();
   affine_matrix
}
