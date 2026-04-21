use crate::config::{Config, EspertoInput, EspertoOutput, OutputCodes, WiimoteComboHandler, WiimoteEvent};
use crate::dprintln;
use crate::points::{Dot, Vec3};
use crate::track::RawDot;
use crate::track::Tracker;
use esperto::combo::ComboHandler;
use esperto::types::Kind;
use evdevil::bits::BitSet;
use evdevil::event::{Abs, EventKind, EventType, InputEvent, Key, KeyState};
use evdevil::reader::AsyncEvents;
use evdevil::{Evdev, EventReader};
use futures::{StreamExt, stream};
use std::collections::HashMap;
use std::collections::VecDeque;
use std::io::ErrorKind;
use std::path::Path;
use udev::Device;

struct SyncIrEvents {
   dots: [RawDot; 4],
   // synced_dots: [Option<RawDot>; 4],
}

impl SyncIrEvents {
   fn new() -> SyncIrEvents {
      SyncIrEvents {
         dots: [RawDot::default(); 4],
      }
   }

   fn sync_event(&mut self, event: Raw) -> bool {
      match event {
         Raw::IRSyn => true,
         Raw::Abs(abs, value) => {
            match abs {
               Abs::HAT0X => {
                  self.dots[0].x = value;
               }
               Abs::HAT0Y => {
                  self.dots[0].y = value;
               }
               Abs::HAT1X => {
                  self.dots[1].x = value;
               }
               Abs::HAT1Y => {
                  self.dots[1].y = value;
               }
               Abs::HAT2X => {
                  self.dots[2].x = value;
               }
               Abs::HAT2Y => {
                  self.dots[2].y = value;
               }
               Abs::HAT3X => {
                  self.dots[3].x = value;
               }
               Abs::HAT3Y => {
                  self.dots[3].y = value;
               }
               _ => {}
            }
            false
         }
         _ => false,
      }
   }
}

struct SyncAccelEvents {
   x: i32,
   y: i32,
   z: i32,
}

impl SyncAccelEvents {
   fn new() -> SyncAccelEvents {
      SyncAccelEvents { x: 0, y: 0, z: 0 }
   }

   fn sync_event(&mut self, event: Raw) -> Option<Vec3> {
      match event {
         Raw::AccelSyn => Some(Vec3 {
            x: self.x as f32,
            y: self.y as f32,
            z: self.z as f32,
         }),
         Raw::Abs(abs, value) => {
            match abs {
               Abs::X => {
                  self.x = value;
               }
               Abs::Y => {
                  self.y = value;
               }
               Abs::Z => {
                  self.z = value;
               }
               _ => {}
            }
            None
         }
         _ => None,
      }
   }
}

pub struct SyncTracker {
   sync_ir: SyncIrEvents,
   sync_acc: SyncAccelEvents,
   tracker: Tracker,
}

impl SyncTracker {
   pub fn new(config: &Config) -> SyncTracker {
      SyncTracker {
         sync_ir: SyncIrEvents::new(),
         sync_acc: SyncAccelEvents::new(),
         tracker: Tracker::new(config),
      }
   }

   pub fn sync_event(&mut self, event: Raw) -> Option<Dot> {
      if event.is_accel() {
         if let Some(evt) = self.sync_acc.sync_event(event) {
            self.tracker.process_accelerometer_data(evt);
         }
      } else {
         if self.sync_ir.sync_event(event) {
            self.tracker.process_ir_data(self.sync_ir.dots);
            return self.tracker.get_position();
         }
      }
      None
   }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Raw {
   Key(Key, Kind),
   Abs(Abs, i32),
   AccelSyn,
   IRSyn,
}

impl Raw {
   pub fn is_accel(&self) -> bool {
      matches!(
         self,
         Raw::AccelSyn | Raw::Abs(Abs::X, _) | Raw::Abs(Abs::Y, _) | Raw::Abs(Abs::Z, _)
      )
   }
}

pub fn combine_streams(
   (key_stream, ir_stream, accel_stream): (AsyncEvents, AsyncEvents, AsyncEvents),
) -> impl StreamExt<Item = Raw> {
   let key_stream = stream::unfold(key_stream, |mut stream| async {
      loop {
         break match stream.next_event().await {
            Ok(event) => Some((
               match event.kind() {
                  EventKind::Key(event) => Raw::Key(
                     event.key(),
                     if event.state() == KeyState::PRESSED {
                        Kind::Down
                     } else {
                        Kind::Up
                     },
                  ),
                  _ => continue,
               },
               stream,
            )),
            Err(e) => {
               eprintln!("Keys stream error: {:?}", e);
               None
            }
         };
      }
   });
   let accel_stream = stream::unfold(accel_stream, |mut stream| async {
      loop {
         break match stream.next_event().await {
            Ok(event) => Some((
               match event.kind() {
                  EventKind::Syn(_) => Raw::AccelSyn,
                  EventKind::Abs(event) => Raw::Abs(event.abs(), event.value()),
                  _ => continue,
               },
               stream,
            )),
            Err(e) => {
               eprintln!("Accelerometer stream error: {:?}", e);
               None
            }
         };
      }
   });
   let ir_stream = stream::unfold(ir_stream, |mut stream| async {
      loop {
         break match stream.next_event().await {
            Ok(event) => Some((
               match event.kind() {
                  EventKind::Syn(_) => Raw::IRSyn,
                  EventKind::Abs(event) => Raw::Abs(event.abs(), event.value()),
                  _ => continue,
               },
               stream,
            )),
            Err(e) => {
               eprintln!("IR stream error: {:?}", e);
               None
            }
         };
      }
   });
   stream::select(accel_stream, stream::select(ir_stream, key_stream))
}

pub struct DeviceMatcher {
   matches: HashMap<String, (Option<Evdev>, Option<Evdev>, Option<Evdev>)>,
   // ir_device: Option<Evdev>,
   // key_device: Option<Evdev>,
   // accel_device: Option<Evdev>,
}

impl DeviceMatcher {
   pub fn new() -> DeviceMatcher {
      DeviceMatcher {
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

   pub fn new_device(&mut self, device: Device) -> Option<(String, Evdev, Evdev, Evdev)> {
      if let (Some(dev_node), Some(Some(sysname))) = (
         device.devnode(),
         device
            .parent()
            .map(|d| d.parent().map(|d| d.sysname().to_string_lossy().to_string())),
      ) {
         dprintln!("sysname: {:?}", sysname);
         // Now use your existing evdev logic to open it
         return self.update(dev_node, sysname);
      }
      None
   }

   pub fn update(&mut self, new_node: &Path, sysname: String) -> Option<(String, Evdev, Evdev, Evdev)> {
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
         if key_device.is_some() && ir_device.is_some() && accel_device.is_some() {
            let ret = (
               sysname.clone(),
               key_device.take().unwrap(),
               ir_device.take().unwrap(),
               accel_device.take().unwrap(),
            );
            self.matches.remove(&sysname);
            return Some(ret);
         }
      }
      None
   }
}

async fn devices_into_readers(
   (key_device, ir_device, accel_device): (Evdev, Evdev, Evdev),
   grab: bool,
) -> Result<(EventReader, EventReader, EventReader), std::io::Error> {
   if grab {
      key_device.grab()?;
      ir_device.grab()?;
      accel_device.grab()?;
   }
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
   let key_stream = key_device.into_reader()?;
   let ir_stream = ir_device.into_reader()?;
   let accel_stream = accel_device.into_reader()?;

   Ok((key_stream, ir_stream, accel_stream))
}

pub async fn device_handler(
   config: &Config,
   sysname: &str,
   devices: (Evdev, Evdev, Evdev),
) -> Result<(), std::io::Error> {
   let mut readers = devices_into_readers(devices, config.grab).await?;
   let key_stream = readers.0.async_events()?;
   let mut ir_stream = readers.1.async_events()?;
   let accel_stream = readers.2.async_events()?;

   // workaround for `evdevil` bug
   // consume and throw away wrong resync events
   for _ in 0..8 {
      let _ = ir_stream.next_event().await;
   }

   let stream = combine_streams((key_stream, ir_stream, accel_stream));
   tokio::pin!(stream);

   let devs = config.build_devices()?;

   /*let dev = UinputDevice::builder()?
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
   .build("Wiimote Mouse")?;*/

   let mut handler = WiimoteComboHandler::new(&config.esperto);

   let mut tracker = SyncTracker::new(config);

   while let Some(event) = stream.next().await {
      match event {
         Raw::Key(key, kind) => {
            handler.handle(EspertoInput {
               keycode: key.into(),
               kind,
               value: 0,
            });
         }
         event @ (Raw::Abs(_, _) | Raw::AccelSyn | Raw::IRSyn) => {
            /*const MIN_RELIABLE: f32 = 0.23;
            const MAX_RELIABLE: f32 = 0.77;
            const ASPECT_RATIO_Y: f32 = 21.0 / 9.0;
            const ASPECT_RATIO_X: f32 = 9.0 / 9.0;*/

            // let now = Instant::now();
            let dot = tracker.sync_event(event);
            // println!("MAIN: tracking time: {:?}μs", now.elapsed().as_nanos() as f32 / 1000.0);

            if let Some(dot) = dot {
               handler.handle(EspertoInput {
                  keycode: WiimoteEvent::IRAbsX,
                  kind: Kind::AxisUpdate,
                  value: config.screen_limits.map_x(dot.x),
               });
               handler.handle(EspertoInput {
                  keycode: WiimoteEvent::IRAbsY,
                  kind: Kind::AxisUpdate,
                  value: config.screen_limits.map_y(dot.y),
               });
               // dprintln!("Got position from tracker")
            }
         }
      }
      while let Some(EspertoOutput { keycode, kind, value }) = handler.events().pop_front() {
         match keycode.code {
            OutputCodes::Axis(abs) => {
               devs[keycode.slot.to_index()]
                  .as_ref()
                  .unwrap()
                  .write(&[InputEvent::new(EventType::ABS, abs.raw(), value)])?;
            }
            OutputCodes::Key(key) => {
               devs[keycode.slot.to_index()]
                  .as_ref()
                  .unwrap()
                  .write(&[InputEvent::new(
                     EventType::KEY,
                     key.raw(),
                     if kind == Kind::Down { 1 } else { 0 },
                  )])?;
            }
            OutputCodes::CustomAxis(abs) => {
               devs[keycode.slot.to_index()]
                  .as_ref()
                  .unwrap()
                  .write(&[InputEvent::new(EventType::ABS, abs, value)])?;
            }
            OutputCodes::CustomKey(key) => {
               devs[keycode.slot.to_index()]
                  .as_ref()
                  .unwrap()
                  .write(&[InputEvent::new(
                     EventType::KEY,
                     key,
                     if kind == Kind::Down { 1 } else { 0 },
                  )])?;
            }
         }
      }
   }
   Err(std::io::Error::new(ErrorKind::BrokenPipe, "End of event stream"))
}
