use crate::points::Vec3;
use crate::print_utils;
use crate::print_utils::calibration_pane;
use crate::routing::{Raw, SyncAccelEvents, make_abs_stream};
use crate::stats::TWEMA;
use crossterm::event;
use evdevil::Evdev;
use futures::StreamExt;
use nalgebra::{Matrix3x4, Matrix4x3, SMatrix, matrix, vector};
use std::future;
use std::time::{Duration, Instant};

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
   sd_threshold: f32,
}

impl Calibrate {
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

      let progress = 20 - ((sd - self.sd_threshold).sqrt().clamp(0.0, 3.0) * (20.0 / 3.0)) as usize;
      calibration_pane::progress(progress);
      calibration_pane::sds(sds);
      calibration_pane::avgs(self.averages.averages);
      calibration_pane::end();

      if sd < self.sd_threshold {
         return Some(self.averages.averages);
      }
      None
   }
}

pub async fn process(
   accel_device: Evdev,
   position: Position,
   sysname: &str,
   sd_threshold: f32,
) -> Result<(Evdev, Vec3), Box<dyn std::error::Error>> {
   calibration_pane::splash(position);
   pause().await?;

   let mut reader = accel_device.into_reader()?;
   'a: {
      let mut raw_stream = make_abs_stream(reader.async_events()?, Raw::AccelSyn, "Accel", sysname).boxed();
      let sync = SyncAccelEvents::new();
      let stream = sync.to_stream(&mut raw_stream);
      tokio::pin!(stream);

      let mut cal;

      if let Some(event) = stream.next().await {
         let now = Instant::now();
         cal = Calibrate {
            start_time: now,
            averages: TWEMA::new(now, event),
            sd_threshold,
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

pub async fn calibrate(
   mut accel_device: Evdev,
   sysname: &str,
   sd_threshold: f32,
   weight: f32,
) -> Result<Matrix3x4<f32>, Box<dyn std::error::Error>> {
   let mut samples = [Vec3::default(); 6];

   print_utils::clear();

   (accel_device, samples[Position::PosZ as usize]) =
      process(accel_device, Position::PosZ, sysname, sd_threshold).await?;
   (accel_device, samples[Position::NegZ as usize]) =
      process(accel_device, Position::NegZ, sysname, sd_threshold).await?;
   (accel_device, samples[Position::PosX as usize]) =
      process(accel_device, Position::PosX, sysname, sd_threshold).await?;
   (accel_device, samples[Position::NegX as usize]) =
      process(accel_device, Position::NegX, sysname, sd_threshold).await?;
   (accel_device, samples[Position::PosY as usize]) =
      process(accel_device, Position::PosY, sysname, sd_threshold).await?;
   (accel_device, samples[Position::NegY as usize]) =
      process(accel_device, Position::NegY, sysname, sd_threshold).await?;
   drop(accel_device);

   calibration_pane::optimizing();
   let affine_matrix = optimize(
      samples[Position::PosX as usize],
      samples[Position::NegX as usize],
      samples[Position::PosY as usize],
      samples[Position::NegY as usize],
      samples[Position::PosZ as usize],
      samples[Position::NegZ as usize],
      weight,
   );
   calibration_pane::done();

   Ok(affine_matrix)
}

fn optimize(
   pos_x: Vec3,
   neg_x: Vec3,
   pos_y: Vec3,
   neg_y: Vec3,
   pos_z: Vec3,
   neg_z: Vec3,
   weight: f32,
) -> Matrix3x4<f32> {
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

   // let weight = 500.0;
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

async fn pause() -> Result<(), Box<dyn std::error::Error>> {
   // consume already present events
   while event::poll(Duration::default())? {
      event::read()?;
   }
   // wait for keypress
   let mut reader = event::EventStream::new();
   let was = crossterm::terminal::is_raw_mode_enabled()?;
   crossterm::terminal::enable_raw_mode()?;
   while let Some(event) = reader.next().await
      && !event?.is_key_press()
   {}
   if !was {
      crossterm::terminal::disable_raw_mode()?;
   }
   // consume stray events
   while event::poll(Duration::default())? {
      event::read()?;
   }
   Ok(())
}
