#![feature(iter_partition_in_place)]
#![feature(duration_millis_float)]
extern crate core;

mod calibration;
mod config;
mod points;
mod print_utils;
mod routing;
mod stats;
mod track;

// use std::time::Instant;
use crate::config::Config;
use crate::routing::device_handler;
use clap::{Parser, Subcommand};
use evdevil::Evdev;
use futures::stream::StreamExt;
use routing::DeviceMatcher;
use std::cell::RefCell;
use std::fs::File;
use std::io::stdout;
use std::path::PathBuf;
use std::process::Command;
use std::rc::Rc;
use tokio;
use tokio::signal;
use tokio::signal::unix::SignalKind;
use tokio_udev::{AsyncMonitorSocket, MonitorBuilder};
use udev::{Device, Enumerator};

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
   /// Sets a custom config file
   #[arg(short, long, value_name = "FILE",value_hint = clap::ValueHint::FilePath)]
   config: Option<PathBuf>,

   #[clap(subcommand)]
   subcommand: Option<Subcommands>,
}

#[derive(Subcommand)]
enum Subcommands {
   /// Accelerometer calibration wizard
   ///
   /// If a config file was specified, calibration data will be written to it
   Calibration {
      /// Maximum acceptable standard deviations for calibration readings
      #[arg(short, long, value_name = "THRSH", default_value = "1.4")]
      standard_deviation_threshold: f32,
      /// Higher weights are best for biased sensor, lower weights for misaligned sensors
      #[arg(short, long, value_name = "WEIGH", default_value = "500.0")]
      weight: f32,
   },
}

#[tokio::main(flavor = "local")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
   // get configuration
   let args = Args::parse();
   let config: Config = if let Some(config) = &args.config {
      serde_yaml::from_reader(File::open(config)?)?
   } else {
      serde_yaml::from_str(include_str!("default.yaml"))?
   };
   config.validate()?;
   let config = Rc::new(config);
   let counter = Rc::new(RefCell::new(0));
   let calibration_mode = matches!(args.subcommand, Some(Subcommands::Calibration { .. }));

   let handle = tokio::task::spawn_local(main_loop(args, config.clone(), counter.clone()));

   if ! calibration_mode {
      let ctrlc = signal::ctrl_c();
      let mut sigterm = signal::unix::signal(SignalKind::terminate())?;
      let sigterm = sigterm.recv();
      match tokio::select! {
      err = ctrlc => err,
      _ = sigterm => Ok(())

   } {
         Ok(_) => {
            if *counter.borrow() > 0 {
               on_disconnect(&config);
            }
         }
         Err(err) => eprintln!("Error while listening to signals {:?}", err),
      }
   } else {
      handle.await??;
   }

   // we should never get here, unless something happened to udev
   println!("Exiting due to termination signal...");
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
               break Some((dev, (connected, hotplug)));
            } else if let Some(Ok(event)) = hotplug.next().await {
               if event.event_type() == tokio_udev::EventType::Add {
                  break Some((event.device(), (connected, hotplug)));
               } else {
                  continue;
               }
            } else {
               break None;
            }
         }
      },
   ))
}

async fn main_loop(
   args: Args,
   mut config: Rc<Config>,
   counter: Rc<RefCell<u32>>,
) -> Result<(), Box<dyn std::error::Error>> {
   // enumerate devices
   println!("Monitoring for input devices...");
   let mut connected_devices = Enumerator::new()?;
   connected_devices.match_subsystem("input")?;
   let builder = MonitorBuilder::new()?.match_subsystem("input")?;
   let monitor = builder.listen()?;
   let hotplug_devices = AsyncMonitorSocket::new(monitor)?;
   let enumerator = device_enumerator(&mut connected_devices, hotplug_devices)?;
   tokio::pin!(enumerator);

   let mut matcher = DeviceMatcher::new();

   while let Some(device) = enumerator.next().await {
      if let Some((name, key_device, ir_device, accel_device)) = matcher.new_device(device) {
         if let Some(Subcommands::Calibration {
            standard_deviation_threshold,
            weight,
         }) = args.subcommand
         {
            let affine_matrix = calibration::calibrate(accel_device, &name, standard_deviation_threshold, weight).await?;
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

   // we should never get here, unless something happened to udev
   println!("Exiting...");
   Ok(())
}

async fn run(counter: Rc<RefCell<u32>>, config: Rc<Config>, sysname: String, devices: (Evdev, Evdev, Evdev)) {
   println!("Device {sysname}: Fully connected");
   if *counter.borrow() == 0 {
      on_connect(&config);
   }
   counter.replace_with(|n| *n + 1);

   if let Err(e) = device_handler(&config, &sysname, devices).await {
      eprintln!("Device {sysname}: {e}");
   }

   counter.replace_with(|n| *n - 1);
   if *counter.borrow() == 0 {
      on_disconnect(&config);
   }
   eprintln!("Device {sysname}: Disconnected");
}

fn on_connect(config: &Config) {
   if let Some(ref command) = config.on_connect {
      match Command::new(command[0].to_owned())
         .args(&command[1..])
         .stdout(stdout())
         .output()
      {
         Ok(output) => {
            println!("on_connect executed successfully: {}", output.status);
         }
         Err(output) => {
            println!("on_connect failed execution: {}", output);
         }
      }
   }
}

fn on_disconnect(config: &Config) {
   if let Some(ref command) = config.on_disconnect {
      match Command::new(command[0].to_owned())
         .args(&command[1..])
         .stdout(stdout())
         .output()
      {
         Ok(output) => {
            if output.stdout.len() > 0 {
               println!("{}", String::from_utf8(output.stdout).unwrap_or("".to_string()));
            }
            println!("on_disconnect executed successfully: {}", output.status);
         }
         Err(output) => {
            println!("on_disconnect failed execution: {}", output);
         }
      }
   }
}
