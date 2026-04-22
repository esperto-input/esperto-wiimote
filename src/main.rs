#![feature(iter_partition_in_place)]
#![feature(duration_millis_float)]
extern crate core;

mod calibration;
pub mod config;
mod points;
mod print_utils;
mod routing;
mod track;

use std::cell::RefCell;
// use std::time::Instant;
use crate::config::Config;
use crate::routing::device_handler;
use clap::Parser;
use evdevil::Evdev;
use futures::stream::StreamExt;
use routing::DeviceMatcher;
use std::fs::File;
use std::path::PathBuf;
use std::process::Command;
use std::rc::Rc;
use tokio;
use tokio_udev::{AsyncMonitorSocket, MonitorBuilder};
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
   let args = Args::parse();
   let config: Config = if let Some(config) = &args.config {
      serde_yaml::from_reader(File::open(config)?)?
   } else {
      serde_yaml::from_str(include_str!("default.yaml"))?
   };

   config.validate()?;
   serde_yaml::to_writer(File::create("default.yaml")?, &config)?;
   let mut config = Rc::new(config);

   println!("Monitoring for input devices...");
   let mut connected_devices = udev::Enumerator::new()?;
   connected_devices.match_subsystem("input")?;
   let builder = MonitorBuilder::new()?.match_subsystem("input")?;
   let monitor = builder.listen()?;
   let hotplug_devices = AsyncMonitorSocket::new(monitor)?;
   let enumerator = device_enumerator(&mut connected_devices, hotplug_devices)?;
   tokio::pin!(enumerator);

   let mut matcher = DeviceMatcher::new();
   let counter = RefCell::new(0);

   while let Some(device) = enumerator.next().await {
      if let Some((name, key_device, ir_device, accel_device)) = matcher.new_device(device) {
         if args.calibration {
            let affine_matrix = calibration::calibrate(accel_device).await?;
            let accelerometer_calibration: [f32; 12] = *affine_matrix.as_slice().as_array().unwrap();
            if let Some(file) = args.config {
               let config: &mut Config = Rc::make_mut(&mut config);
               config.accelerometer_calibration = accelerometer_calibration;
               serde_yaml::to_writer(File::create(file.clone())?, config)?;
               println!("\n\nCalibration written to file: {:?}", file);
            } else {
               println!("\n\naccelerometer_calibration: {:?}", accelerometer_calibration);
            }
            return Ok(());
         }

         // clear screen for tuning
         #[cfg(feature = "tuning")]
         print_utils::clear();
         tokio::task::spawn_local(run(
            counter.clone(),
            config.clone(),
            name,
            (key_device, ir_device, accel_device),
         ));
      }
   }

   println!("Stop waiting for devices, exiting...");
   Ok(())
}

fn device_enumerator(
   connected: &mut Enumerator,
   hotplug: AsyncMonitorSocket,
) -> Result<impl StreamExt<Item = Device>, Box<std::io::Error>> {
   Ok(futures::stream::unfold(
      (connected.scan_devices()?, hotplug),
      |(mut connected, mut hotplug)| async {
         loop {
            if let Some(dev) = connected.next() {
               break Some((dev, (connected, hotplug)))
            } else if let Some(Ok(event)) = hotplug.next().await {
               if event.event_type() == tokio_udev::EventType::Add {
                  break Some((event.device(), (connected, hotplug)))
               } else {
                  continue;
               }
            } else {
               break None
            }
         }
      },
   ))
}

async fn run(counter: RefCell<u32>, config: Rc<Config>, sysname: String, devices: (Evdev, Evdev, Evdev)) {
   if *counter.borrow() == 0
      && let Some(ref command) = config.on_connect
   {
      match Command::new(command[0].to_owned()).args(&command[1..]).output() {
         Ok(output) => {
            println!("on_connect executed successfully: {}", output.status);
         }
         Err(output) => {
            println!("on_connect failed execution: {}", output);
         }
      }
   }
   counter.replace_with(|n| *n + 1);
   if let Err(e) = device_handler(&config, &sysname, devices).await {
      eprintln!("Device {sysname}, {e}");
   }
   counter.replace_with(|n| *n - 1);
   if *counter.borrow() == 0
      && let Some(ref command) = config.on_disconnect
   {
      match Command::new(command[0].to_owned()).args(&command[1..]).output() {
         Ok(output) => {
            println!("on_disconnect executed successfully: {}", output.status);
         }
         Err(output) => {
            println!("on_disconnect failed execution: {}", output);
         }
      }
   }
}
