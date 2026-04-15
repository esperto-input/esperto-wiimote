use evdevil::event::{Abs, Key};
use serde::{Deserialize, Serialize};
use crate::events::WiimoteEvents;

#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub enum Codes {
   Axis(Abs),
   Key(Key),
   Custom(u16)
}

fn default_mouse() -> Vec<Codes> {
   vec![Codes::Axis(Abs::X), Codes::Axis(Abs::Y), Codes::Key(Key::BTN_LEFT), Codes::Key(Key::BTN_RIGHT)]
}

#[derive(Clone, Debug, Deserialize, Serialize, Default)]
pub struct Capabilities{
   #[serde(default = "Vec::default")]
   keyboard: Vec<Codes>,
   #[serde(default = "Vec::default")]
   mouse: Vec<Codes>,
   #[serde(default = "Vec::default")]
   gamepad: Vec<Codes>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Config {
   #[serde(default = "Capabilities::default")]
   capabilities: Capabilities,
   esperto: esperto::config::Config<WiimoteEvents, Codes>
}


