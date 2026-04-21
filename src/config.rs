use crate::regprintln;
use esperto::combo::ComboHandlerSimple;
use esperto::types::Event;
use esperto::types::Scalar;
use evdevil::AbsInfo;
use evdevil::event::{Abs, Key};
use evdevil::uinput::{AbsSetup, UinputDevice};
use serde::{Deserialize, Serialize};
use std::collections::{HashSet, VecDeque};
use std::fmt::{Display, Formatter};
use std::ops::Index;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, Scalar, Ord, PartialOrd)]
pub enum WiimoteEvent {
   A,
   B,
   Up,
   Down,
   Left,
   Right,
   Plus,
   Minus,
   Home,
   Btn1,
   Btn2,
   IRAbsX,
   IRAbsY,
}

impl WiimoteEvent {
   pub fn is_axis(&self) -> bool {
      self == &WiimoteEvent::IRAbsX || self == &WiimoteEvent::IRAbsY
   }

   pub fn is_key(&self) -> bool {
      !self.is_axis()
   }
}

impl From<Key> for WiimoteEvent {
   fn from(key: Key) -> Self {
      match key {
         Key::BTN_SOUTH => WiimoteEvent::A,
         Key::BTN_EAST => WiimoteEvent::B,
         Key::BTN_DPAD_UP => WiimoteEvent::Up,
         Key::BTN_DPAD_DOWN => WiimoteEvent::Down,
         Key::BTN_DPAD_LEFT => WiimoteEvent::Left,
         Key::BTN_DPAD_RIGHT => WiimoteEvent::Right,
         Key::BTN_START => WiimoteEvent::Plus,
         Key::BTN_SELECT => WiimoteEvent::Minus,
         Key::BTN_MODE => WiimoteEvent::Home,
         Key::BTN_1 => WiimoteEvent::Btn1,
         Key::BTN_2 => WiimoteEvent::Btn2,
         _ => {
            panic!("Unexpected key {:?} received from wiimote", key);
         }
      }
   }
}

#[derive(Copy, Clone, Debug, Hash, PartialOrd, Ord, Eq, PartialEq, Serialize, Deserialize)]
pub enum OutputCodes {
   Axis(Abs),
   Key(Key),
   CustomAxis(u16),
   CustomKey(u16),
}

impl OutputCodes {
   pub fn is_key(&self) -> bool {
      matches!(self, Self::Key(_) | Self::CustomKey(_))
   }

   pub fn is_axis(&self) -> bool {
      !self.is_key()
   }

   pub fn get_raw(self) -> u16 {
      match self {
         OutputCodes::Axis(raw) => raw.raw(),
         OutputCodes::Key(raw) => raw.raw(),
         OutputCodes::CustomAxis(raw) | OutputCodes::CustomKey(raw) => raw,
      }
   }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SlotInfo {
   pub name: String,
   #[serde(default = "Default::default")]
   pub repeating: bool,
}

pub fn default_slot_info<const N: usize>() -> SlotInfo {
   SlotInfo {
      name: format!("Esperto Wiimote {}", N),
      repeating: 1 == N,
   }
}

impl Default for SlotInfo {
   fn default() -> Self {
      default_slot_info::<0>()
   }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Slots {
   #[serde(default = "default_slot_info::<1>")]
   pub slot1: SlotInfo,
   #[serde(default = "default_slot_info::<2>")]
   pub slot2: SlotInfo,
   #[serde(default = "default_slot_info::<3>")]
   pub slot3: SlotInfo,
   #[serde(default = "default_slot_info::<4>")]
   pub slot4: SlotInfo,
}

impl Default for Slots {
   fn default() -> Self {
      Slots {
         slot1: default_slot_info::<1>(),
         slot2: default_slot_info::<2>(),
         slot3: default_slot_info::<3>(),
         slot4: default_slot_info::<4>(),
      }
   }
}

impl Index<usize> for Slots {
   type Output = SlotInfo;
   fn index(&self, index: usize) -> &Self::Output {
      match index {
         0 => &self.slot1,
         1 => &self.slot2,
         2 => &self.slot3,
         3 => &self.slot4,
         _ => panic!("index out of bounds"),
      }
   }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScreenLimits {
   north: i32,
   south: i32,
   west: i32,
   east: i32,
}

impl Default for ScreenLimits {
   fn default() -> Self {
      ScreenLimits {
         north: 0,
         south: 4095,
         west: 0,
         east: 4095,
      }
   }
}

impl ScreenLimits {
   pub fn map_x(&self, x: f32) -> i32 {
      ((x * (self.east - self.west) as f32 + self.west as f32) as i32).clamp(0, 4095)
   }

   pub fn map_y(&self, y: f32) -> i32 {
      ((y * (self.south - self.north) as f32 + self.north as f32) as i32).clamp(0, 4095)
   }
}

#[repr(u8)]
#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Scalar, Serialize, Deserialize)]
pub enum Slot {
   #[default]
   Slot1,
   Slot2,
   Slot3,
   Slot4,
}

impl Slot {
   pub fn to_index(&self) -> usize {
      match self {
         Slot::Slot1 => 0,
         Slot::Slot2 => 1,
         Slot::Slot3 => 2,
         Slot::Slot4 => 3,
      }
   }
}

#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct OutputEvent {
   pub slot: Slot,
   pub code: OutputCodes,
}

impl Display for OutputEvent {
   fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
      write!(f, "{:?}@{:?}", self.code, self.slot)
   }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorBarSize {
   width: f32,
   cluster_width: f32,
   cluster_height: f32,
   pixel_size: f32,
}

impl Default for SensorBarSize {
   fn default() -> Self {
      SensorBarSize {
         width: 19.5,
         cluster_width: 4.5,
         cluster_height: 1.0,
         pixel_size: 256.0,
      }
   }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Smoothing {
   radius: f32,
   speed: f32,
   deadzone: f32,
}

impl Default for Smoothing {
   fn default() -> Self {
      Smoothing {
         radius: 30.0,
         speed: 0.15,
         deadzone: 8.0,
      }
   }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Parking {
   x: i32,
   y: i32,
}

impl Default for Parking {
   fn default() -> Self {
      Parking { x: 4095, y: 4095 }
   }
}

pub type EspertoConfig = esperto::config::Config<WiimoteEvent, OutputEvent>;
pub type EspertoInput = Event<WiimoteEvent, i32>;
pub type EspertoOutput = Event<OutputEvent, i32>;
pub type EventQueue = VecDeque<Event<OutputEvent, i32>>;
pub type WiimoteComboHandler = ComboHandlerSimple<WiimoteEvent, OutputEvent, i32, EventQueue>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Config {
   #[serde(default = "Default::default")]
   pub grab: bool,
   #[serde(default = "Default::default")]
   pub slots: Slots,
   #[serde(default = "Default::default")]
   pub accelerometer_calibration: [f32; 12],
   #[serde(default = "Default::default")]
   pub screen_limits: ScreenLimits,
   #[serde(default = "Default::default")]
   pub sensor_bar: SensorBarSize,
   #[serde(default = "Default::default")]
   pub smoothing: Smoothing,
   #[serde(default = "Default::default")]
   pub parking: Parking,
   pub esperto: EspertoConfig,
}

impl Config {
   pub fn build_devices(&self) -> Result<[Option<UinputDevice>; 4], std::io::Error> {
      let indices = [0, 1, 2, 3];
      let mut key_capabilities = indices.map(|_| HashSet::new());
      let mut abs_capabilities = indices.map(|_| HashSet::new());
      self.esperto.iter_actions().for_each(|(_, _, output)| {
         if let Some(output) = output {
            if output.code.is_key() {
               key_capabilities[output.slot.to_index()].insert(output.code.get_raw());
            } else {
               abs_capabilities[output.slot.to_index()].insert(output.code.get_raw());
            }
         }
      });

      let mut devs: [_; 4] = indices.map(|_| None);

      for i in 0..4 {
         if key_capabilities[i].is_empty() && abs_capabilities[i].is_empty() {
            devs[i] = None;
         } else {
            let mut builder = UinputDevice::builder()?
               .with_keys(key_capabilities[i].iter().map(|raw| Key::from_raw(*raw)))?
               .with_abs_axes(
                  abs_capabilities[i]
                     .iter()
                     .map(|raw| AbsSetup::new(Abs::from_raw(*raw), AbsInfo::new(0, 4095))),
               )?;
            if self.slots[i].repeating {
               builder = builder.with_key_repeat()?;
            }
            devs[i] = Some(builder.build(&self.slots[i].name)?);
         }
      }

      Ok(devs)
   }

   pub fn validate(&self) -> Result<(), std::io::Error> {
      match self.esperto.validate() {
         Ok(warnings) => {
            for warn in warnings.iter() {
               regprintln!("Warning: {}", warn);
            }
            Ok(())
         }
         Err(error) => Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            format!("Config validation failed: {}", error),
         )),
      }
      .and_then(|_| {
         self.esperto.iter_actions().fold(Ok(()), |res, (input, _, output)| {
            res.and_then(|_| {
               if let Some(output) = output {
                  if input.is_axis() && output.code.is_key() {
                     return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        format!("Config validation failed: cannot map absolut axis {input:?}, to key event {output}",),
                     ));
                  }
                  if input.is_key() && output.code.is_axis() {
                     return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        format!("Config validation failed: cannot map key {input:?}, to absolute axis {output}",),
                     ));
                  }
               }
               Ok(())
            })
         })
      })
   }
}
