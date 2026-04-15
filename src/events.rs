use crate::points::{Dot, Vec3};
use crate::track::Tracker;
use crate::track::RawDot;
use esperto::types::Kind;
use evdevil::event::{Abs, EventKind, Key, KeyState};
use evdevil::reader::AsyncEvents;
use frozen_collections::Scalar;
use futures::{StreamExt, stream};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, Scalar, Ord, PartialOrd)]
pub enum WiimoteEvents {
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

/*fn from_key_event(event: InputEvent) -> Option<Event<WiimoteEvents, f32>> {
   match event.kind() {
      EventKind::Key(event) => Some(Event {
         keycode: match event.key() {
            Key::BTN_SOUTH => WiimoteEvents::A,
            Key::BTN_EAST => WiimoteEvents::B,
            Key::BTN_DPAD_UP => WiimoteEvents::Up,
            Key::BTN_DPAD_DOWN => WiimoteEvents::Down,
            Key::BTN_DPAD_LEFT => WiimoteEvents::Left,
            Key::BTN_DPAD_RIGHT => WiimoteEvents::Right,
            Key::BTN_START => WiimoteEvents::Plus,
            Key::BTN_SELECT => WiimoteEvents::Minus,
            Key::BTN_MODE => WiimoteEvents::Home,
            Key::BTN_1 => WiimoteEvents::Btn1,
            Key::BTN_2 => WiimoteEvents::Btn2,
            _ => {
               return None;
            }
         },
         kind: if event.state() == KeyState::PRESSED {
            Kind::Down
         } else {
            Kind::Up
         },
         value: 0.0,
      }),
      _ => None,
   }
}*/

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
   pub fn new() -> SyncTracker {
      SyncTracker {
         sync_ir: SyncIrEvents::new(),
         sync_acc: SyncAccelEvents::new(),
         tracker: Tracker::new(),
      }
   }

   pub fn sync_event(&mut self, event: Raw) -> Option<Dot> {
      if event.is_accel() {
         if let Some(evt) = self.sync_acc.sync_event(event) {
            self.tracker.process_accelerometer_data(evt);
         }
      } else {
         if self.sync_ir.sync_event(event) {
            self.tracker.process_ir_data(&self.sync_ir.dots);
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
   key_stream: AsyncEvents,
   ir_stream: AsyncEvents,
   accel_stream: AsyncEvents,
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
