#![feature(iter_partition_in_place)]

pub mod config;
mod events;
mod points;
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

use crate::events::{Raw, SyncTracker, WiimoteEvent};
use clap::Parser;
use esperto::config::{Action, Combo, ModifierDecl};
use evdevil::bits::BitSet;
use evdevil::event::{Abs, EventType, InputEvent, Key, Rel};
use evdevil::uinput::{AbsSetup, UinputDevice};
use evdevil::{AbsInfo, Evdev, InputProp};
use futures::stream::StreamExt;
use std::collections::HashMap;
use std::fs::File;
use std::io::ErrorKind;
use std::path::{Path, PathBuf};
use esperto::types::Kind;
// use std::time::Instant;
use crate::config::{Config, OutputCodes, OutputEvent, Slot};
use tokio;
use tokio_udev::{AsyncMonitorSocket, MonitorBuilder};
use track::print_utils;

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
   /// Sets a custom config file
   #[arg(short, long, value_name = "FILE",value_hint = clap::ValueHint::FilePath)]
   config: Option<PathBuf>,
}

async fn handler(key_device: Evdev, ir_device: Evdev, accel_device: Evdev) -> Result<(), std::io::Error> {
   // key_device.grab()?;
   // ir_device.grab()?;
   // accel_device.grab()?;

   key_device.set_key_mask(&BitSet::from_iter([
      Key::BTN_SOUTH,
      Key::BTN_EAST,
      Key::BTN_DPAD_UP,
      Key::BTN_DPAD_DOWN,
      Key::BTN_DPAD_LEFT,
      Key::BTN_DPAD_RIGHT,
      Key::BTN_START,
      Key::BTN_SELECT,
      Key::BTN_MODE,
      Key::BTN_1,
      Key::BTN_2,
   ]))?;
   key_device.set_event_mask(&BitSet::from_iter([EventType::KEY]))?;
   ir_device.set_abs_mask(&BitSet::from_iter([
      Abs::HAT0X,
      Abs::HAT0Y,
      Abs::HAT1X,
      Abs::HAT1Y,
      Abs::HAT2X,
      Abs::HAT2Y,
      Abs::HAT3X,
      Abs::HAT3Y,
   ]))?;
   ir_device.set_event_mask(&BitSet::from_iter([EventType::ABS, EventType::SYN]))?;
   accel_device.set_abs_mask(&BitSet::from_iter([Abs::X, Abs::Y, Abs::Z]))?;
   accel_device.set_event_mask(&BitSet::from_iter([EventType::ABS, EventType::SYN]))?;

   // check if the device has an ENTER key
   let mut key_stream = key_device.into_reader()?;
   let mut ir_stream = ir_device.into_reader()?;
   let mut accel_stream = accel_device.into_reader()?;
   let key_stream = key_stream.async_events()?;
   let mut ir_stream = ir_stream.async_events()?;
   let accel_stream = accel_stream.async_events()?;

   // workaround for `evdevil` bug
   // consume and throw away wrong resync events
   for _ in 0..8 {
      let _ = ir_stream.next_event().await;
   }

   let stream = events::combine_streams(key_stream, ir_stream, accel_stream);
   tokio::pin!(stream);

   let dev = UinputDevice::builder()?
      .with_keys([
         Key::BTN_LEFT,
         Key::BTN_RIGHT,
         Key::BTN_MIDDLE,
         Key::KEY_SPACE,
         Key::KEY_ESC,
         Key::KEY_UP,
         Key::KEY_DOWN,
         Key::KEY_LEFT,
         Key::KEY_RIGHT,
         Key::KEY_SCROLLUP,
         Key::KEY_SCROLLDOWN,
         Key::KEY_SCROLLLOCK,
         Key::KEY_PAGEUP,
         Key::KEY_PAGEDOWN,
         Key::BTN_MIDDLE,
         Key::BTN_WHEEL,
      ])?
      .with_rel_axes([Rel::WHEEL, Rel::HWHEEL])?
      .with_abs_axes([
         AbsSetup::new(Abs::X, AbsInfo::new(0, 4095)),
         AbsSetup::new(Abs::Y, AbsInfo::new(0, 4095)),
      ])?
      // .with_key_repeat()?
      .with_props([InputProp::POINTER])?
      .build("Wiimote Mouse")?;

   let mut tracker = SyncTracker::new();

   while let Some(event) = stream.next().await {
      match event {
         Raw::Key(Key::BTN_SOUTH, kind) => {
            dev.write(&[InputEvent::new(
               EventType::KEY,
               Key::KEY_PAGEUP.raw(),
               if kind == Kind::Down { 1 } else { 0 },
            )])?;
         }
         Raw::Key(Key::BTN_SELECT, kind) => {
            dev.write(&[InputEvent::new(
               EventType::KEY,
               Key::KEY_PAGEDOWN.raw(),
               if kind == Kind::Down { 1 } else { 0 },
            )])?;
         }
         Raw::Key(Key::BTN_START, kind) => {
            dev.write(&[InputEvent::new(
               EventType::KEY,
               Key::BTN_MIDDLE.raw(),
               if kind == Kind::Down { 1 } else { 0 },
            )])?;
         }
         event @ (Raw::Abs(_, _) | Raw::AccelSyn | Raw::IRSyn) => {
            const MIN_RELIABLE: f32 = 0.23;
            const MAX_RELIABLE: f32 = 0.77;
            const ASPECT_RATIO_Y: f32 = 21.0 / 9.0;
            const ASPECT_RATIO_X: f32 = 9.0 / 9.0;

            // let now = Instant::now();
            let dot = tracker.sync_event(event);
            // println!("MAIN: tracking time: {:?}μs", now.elapsed().as_nanos() as f32 / 1000.0);

            if let Some(dot) = dot {
               dev.write(&[
                  InputEvent::new(
                     EventType::ABS,
                     Abs::X.raw(),
                     (((dot.x - MIN_RELIABLE) / (MAX_RELIABLE - MIN_RELIABLE) * ASPECT_RATIO_X) * 4095.0)
                        .clamp(0.0, 4095.0) as i32,
                  ),
                  InputEvent::new(
                     EventType::ABS,
                     Abs::Y.raw(),
                     (((dot.y - MIN_RELIABLE) / (MAX_RELIABLE - MIN_RELIABLE) * ASPECT_RATIO_Y) * 4095.0)
                        .clamp(0.0, 4095.0) as i32,
                  ),
               ])?;
            }
         }
         _ => {}
      }
   }
   Err(std::io::Error::new(ErrorKind::BrokenPipe, "End of event stream"))
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
   // clear screen for tuning
   print_utils::clear!();

   let args = Args::parse();
   let mut config: Config = if let Some(config) = args.config {
      serde_yaml::from_reader(File::open(config)?)?
   } else {
      serde_yaml::from_str(include_str!("default.yaml"))?
   };

   config.validate()?;
   serde_yaml::to_writer(File::create("default.yaml")?, &config)?;
   // return Ok(());

   // Set up the udev monitor for the "input" subsystem
   let builder = MonitorBuilder::new()?.match_subsystem("input")?;

   let monitor = builder.listen()?;
   let mut stream = AsyncMonitorSocket::new(monitor)?;

   // println!("Monitoring for input devices...");

   let mut enumerator = udev::Enumerator::new()?;
   enumerator.match_subsystem("input")?;

   let mut composite = Composite::new();

   // println!("Scanning existing devices...");
   for device in enumerator.scan_devices()? {
      if let (Some(dev_node), Some(Some(sysname))) = (
         device.devnode(),
         device
            .parent()
            .map(|d| d.parent().map(|d| d.sysname().to_string_lossy().to_string())),
      ) {
         dprintln!("sysname: {:?}", sysname);
         // Now use your existing evdev logic to open it
         composite.update(dev_node, sysname).await;
      }
   }

   // println!("Waiting for device hotplug...");
   while let Some(Ok(event)) = stream.next().await {
      // We only care about new devices being "added"
      if event.event_type() == tokio_udev::EventType::Add {
         let device = event.device();
         if let (Some(dev_node), Some(Some(sysname))) = (
            device.devnode(),
            device
               .parent()
               .map(|d| d.parent().map(|d| d.sysname().to_string_lossy().to_string())),
         ) {
            dprintln!("sysname: {:?}", sysname);
            // println!("node: {dev_node:?}");
            // Now use your existing evdev logic to open it
            composite.update(dev_node, sysname).await;
         }
      }
   }
   regprintln!("Stop waiting for devices, exiting...");
   Ok(())
}

struct Composite {
   matches: HashMap<String, (Option<Evdev>, Option<Evdev>, Option<Evdev>)>,
   // ir_device: Option<Evdev>,
   // key_device: Option<Evdev>,
   // accel_device: Option<Evdev>,
}

impl Composite {
   fn new() -> Composite {
      Composite {
         matches: HashMap::new(),
         // key_device: None,
         // ir_device: None,
         // accel_device: None,
      }
   }

   fn add_key(&mut self, dev: Evdev, sysname: String) {
      self.matches.entry(sysname).or_insert((None, None, None)).0 = Some(dev);
   }

   fn add_ir(&mut self, dev: Evdev, sysname: String) {
      self.matches.entry(sysname).or_insert((None, None, None)).1 = Some(dev);
   }

   fn add_accel(&mut self, dev: Evdev, sysname: String) {
      self.matches.entry(sysname).or_insert((None, None, None)).2 = Some(dev);
   }

   async fn update(&mut self, new_node: &Path, sysname: String) {
      let devices = Evdev::open(new_node)
         .and_then(|dev| dev.name().map(|name| (dev, name)))
         .map(|(dev, name)| {
            // let d = Device::from_syspath(new_node).map(|d| d.devpath().to_owned());
            // println!("dev: {:?}, name: {:?}", d, name);
            match name.as_str() {
               "Nintendo Wii Remote" => {
                  self.add_key(dev, sysname.clone());
                  // self.key_device = Some(dev);
               }
               "Nintendo Wii Remote IR" => {
                  self.add_ir(dev, sysname.clone());
                  // self.ir_device = Some(dev);
               }
               "Nintendo Wii Remote Accelerometer" => {
                  self.add_accel(dev, sysname.clone());
                  // self.accel_device = Some(dev);
               }
               _ => {}
            }
         })
         .map_or_else(
            |err| {
               dprintln!(
                  "Info: Failed to open device: {}, with error: {:?}",
                  new_node.to_string_lossy(),
                  err
               );
               None
            },
            |_| self.matches.get_mut(&sysname),
         );
      if let Some((key_device, ir_device, accel_device)) = devices {
         async fn run(sysname: String, key_device: Evdev, ir_device: Evdev, accel_device: Evdev) {
            if let Err(e) = handler(key_device, ir_device, accel_device).await {
               errprintln!("Device {sysname}, {e}");
            }
         }
         if key_device.is_some() && ir_device.is_some() && accel_device.is_some() {
            tokio::spawn(run(sysname.clone(),
               key_device.take().unwrap(),
               ir_device.take().unwrap(),
               accel_device.take().unwrap(),
            ));
            self.matches.remove(&sysname);
         }
      }
   }
}
