#![feature(iter_partition_in_place)]

mod calibration;
pub mod config;
mod points;
mod routing;
mod track;

#[macro_export]
macro_rules! dprintln {
    ($($arg:tt)*) => {
       #[cfg(all(debug_assertions,not(feature = "tuning")))]
       ::std::println!($($arg)*)
    };
}

#[macro_export]
macro_rules! regprintln {
    ($($arg:tt)*) => {
       #[cfg(not(feature = "tuning"))]
       ::std::println!($($arg)*)
    };
}

#[macro_export]
macro_rules! errprintln {
    ($($arg:tt)*) => {
       #[cfg(not(feature = "tuning"))]
       ::std::eprintln!($($arg)*)
    };
}

// use std::time::Instant;
use crate::config::Config;
use crate::points::Vec3;
use crate::routing::device_handler;
use clap::Parser;
use evdevil::Evdev;
use futures::stream::StreamExt;
use routing::DeviceMatcher;
use std::fs::File;
use std::path::PathBuf;
use std::rc::Rc;
use nalgebra::Storage;
use tokio;
use tokio_udev::{AsyncMonitorSocket, MonitorBuilder};
use track::print_utils;
use udev::{Device, Enumerator};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
   /// Sets a custom config file
   #[arg(short, long, value_name = "FILE",value_hint = clap::ValueHint::FilePath)]
   config: Option<PathBuf>,

   /// Start calibration wizard
   #[arg(short = 'C', long)]
   calibration: bool,
}

#[tokio::main(flavor = "local")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
   // clear screen for tuning
   print_utils::clear!();

   let calibration = calibration::optimize(
      Vec3::new( 67.944, -32.606, -26.832),
      Vec3::new(-127.707, -36.005, -26.209),
      Vec3::new(-27.998, 62.078, -30.509),
      Vec3::new(-32.917,-131.999, -28.507),
      Vec3::new(-28.999, -32.979, 68.561),
      Vec3::new(-30.950, -36.000,-128.799),
   );
   regprintln!(
      "{:?}",
      calibration
      /*calibration::optimize(
         Vec3::new(68.062, -32.436, -26.039),
         Vec3::new(-127.526, -35.000, -26.121),
         Vec3::new(-27.174, 62.780, -28.992),
         Vec3::new(-30.000, -130.493, -28.837),
         Vec3::new(-28.000, -32.893, 68.559),
         Vec3::new(-29.660, -34.707, -128.245),
      )*/
   );

   let args = Args::parse();
   let mut config: Config = if let Some(config) = args.config {
      serde_yaml::from_reader(File::open(config)?)?
   } else {
      serde_yaml::from_str(include_str!("default.yaml"))?
   };

   config.accelerometer_calibration = *calibration.as_slice().as_array().unwrap();

   config.validate()?;
   serde_yaml::to_writer(File::create("default.yaml")?, &config)?;
   let config = Rc::new(config);
   // return Ok(());

   regprintln!("Monitoring for input devices...");
   let mut connected_devices = udev::Enumerator::new()?;
   connected_devices.match_subsystem("input")?;
   let builder = MonitorBuilder::new()?.match_subsystem("input")?;
   let monitor = builder.listen()?;
   let hotplug_devices = AsyncMonitorSocket::new(monitor)?;
   let mut enumerator = device_enumerator(&mut connected_devices, hotplug_devices)?;
   tokio::pin!(enumerator);

   let mut matcher = DeviceMatcher::new();

   while let Some(device) = enumerator.next().await {
      if let Some((name, key_device, ir_device, accel_device)) = matcher.new_device(device) {
         tokio::task::spawn_local(run(config.clone(), name, (key_device, ir_device, accel_device)));
      }
   }

   regprintln!("Stop waiting for devices, exiting...");
   Ok(())
}

fn device_enumerator(
   connected: &mut Enumerator,
   hotplug: AsyncMonitorSocket,
) -> Result<impl StreamExt<Item = Device>, Box<std::io::Error>> {
   Ok(futures::stream::unfold(
      (connected.scan_devices()?, hotplug),
      |(mut connected, mut hotplug)| async {
         if let Some(dev) = connected.next() {
            Some((dev, (connected, hotplug)))
         } else if let Some(Ok(event)) = hotplug.next().await
            && event.event_type() == tokio_udev::EventType::Add
         {
            Some((event.device(), (connected, hotplug)))
         } else {
            None
         }
      },
   ))
}

async fn run(config: Rc<Config>, sysname: String, devices: (Evdev, Evdev, Evdev)) {
   if let Err(e) = device_handler(&config, &sysname, devices).await {
      errprintln!("Device {sysname}, {e}");
   }
}
