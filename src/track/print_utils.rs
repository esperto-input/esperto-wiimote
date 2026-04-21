#![allow(unused_must_use)]

#[cfg(feature = "tuning")]
use crate::points::Dot;
#[cfg(feature = "tuning")]
use crate::points::Vec3;
#[cfg(feature = "tuning")]
use crate::track::types::RawDot;
#[cfg(feature = "tuning")]
use crate::track::{ERROR_MAX_COUNT, GLITCH_MAX_COUNT};
#[cfg(feature = "tuning")]
use const_format::{concatcp, str_repeat};
#[cfg(feature = "tuning")]
use crossterm::{
   Command,
   cursor::{MoveDown, MoveTo, MoveToColumn},
   queue,
   style::Print,
   terminal::{Clear, ClearType},
};
use proc_macros::phantom_pub;
#[cfg(feature = "tuning")]
use std::io::Write;
#[cfg(feature = "tuning")]
use std::io::{Stdout, stdout};

const SMOOTHING_PANEL_X: u16 = 60;
const SMOOTHING_PANEL_Y: u16 = 60;

#[macro_export]
macro_rules! irprintln {
    ($($arg:tt)*) => {
       #[cfg(feature = "ir-debug")]
       ::std::println!($($arg)*);
    };
}
#[macro_export]
macro_rules! irprint {
    ($($arg:tt)*) => {
       #[cfg(feature = "ir-debug")]
       ::std::print!($($arg)*);
    };
}

#[macro_export]
macro_rules! accprintln {
    ($($arg:tt)*) => {
       #[cfg(feature = "acc-debug")]
       ::std::println!($($arg)*);
    };
}

#[macro_export]
macro_rules! accprint {
    ($($arg:tt)*) => {
       #[cfg(feature = "acc-debug")]
       ::std::print!($($arg)*);
    };
}

#[cfg(feature = "tuning")]
struct NewLine {
   x: u16,
}

#[cfg(feature = "tuning")]
impl Command for NewLine {
   fn write_ansi(&self, f: &mut impl std::fmt::Write) -> std::fmt::Result {
      MoveDown(1).write_ansi(f)?;
      MoveToColumn(self.x).write_ansi(f)?;
      Ok(())
   }
}

#[allow(non_snake_case)]
#[cfg(feature = "tuning")]
fn Newline(x: u16) -> NewLine {
   NewLine { x }
}

#[phantom_pub("tuning", "print_utils")]
pub fn clear() {
   let mut so: Stdout = stdout();
   queue!(so, Clear(ClearType::All));
}

macro_rules! gen_begin {
   ($title:literal, $module:literal) => {
      #[phantom_pub("tuning", $module)]
      pub fn begin() {
         queue!(
            stdout(),
            MoveTo(X, Y),
            Print("┌"),
            Print(format!("{:─^WIDTH$}", $title)),
            Print("┐")
         );
      }
   };
}

macro_rules! gen_end {
   ($module:literal) => {
      #[phantom_pub("tuning", $module)]
      pub fn end() {
         let mut so: Stdout = stdout();
         queue!(
            so,
            MoveTo(X, Y + HEIGHT),
            Print(concatcp!("└", str_repeat!("─", WIDTH), "┘"))
         );
         so.flush().unwrap();
      }
   };
}

#[cfg(feature = "tuning")]
macro_rules! print {
      () => {
         queue!(
            stdout(),
            Print(concatcp!("│", str_repeat!(" ", WIDTH), "│"))
         );
      };
      ($($arg:tt)*) => {
         queue!(
            stdout(),
            Print("│"),
            Print(format!("{:WIDTH$}", format!($($arg)*))),
            Print("│"),
         );
      };
}

#[cfg(feature = "tuning")]
macro_rules! println {
      ()=>{
         print!();
         queue!(stdout(), Newline(X));};
      ($($arg:tt)*) => {
         print!($($arg)*);
         queue!(stdout(), Newline(X));
      };
}

pub mod sensorbar_pane {
   use super::*;

   #[cfg(feature = "tuning")]
   const X: u16 = 0;
   #[cfg(feature = "tuning")]
   const Y: u16 = 0;
   #[cfg(feature = "tuning")]
   const WIDTH: usize = 60;
   #[cfg(feature = "tuning")]
   const HEIGHT: u16 = 7;

   gen_begin!("sensorbar", "sensorbar_pane");
   gen_end!("sensorbar_pane");

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn raw_dots(raw_dots: &[RawDot; 4]) {
      let mut so = stdout();
      queue!(so, MoveTo(X, Y + 1), Print("│"));
      for dot in raw_dots {
         queue!(so, Print(format!("({:6},{:6})", dot.x, dot.y)));
      }
      queue!(so, Print("│"));
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn dots(raw_dots: &[Dot], dots: &[Dot]) {
      queue!(stdout(), MoveTo(X, Y + 2));
      println!(
         "{}",
         raw_dots
            .iter()
            .map(|dot| format!("({:+6.3},{:+6.3})", dot.x, dot.y))
            .collect::<String>()
      );
      print!(
         "{}",
         dots
            .iter()
            .map(|dot| format!("({:+6.3},{:+6.3})", dot.x, dot.y))
            .collect::<String>()
      );
   }

   #[cfg(feature = "tuning")]
   fn aligned(mode: &str, dot: usize, bardot: usize) {
      queue!(stdout(), MoveTo(X, Y + 4));
      let mut s: String = (0..4)
         .map(|i| {
            if dot == i {
               if bardot == 0 {
                  "       Ⓛ       "
               } else {
                  "       Ⓡ       "
               }
            } else {
               "               "
            }
         })
         .collect();
      s.replace_range(0..mode.len(), mode);
      println!("{}", s);
      print!(
         "{:^WIDTH$}",
         format!("aligned {}", if bardot == 0 { "LEFT" } else { "RIGHT" })
      );
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn single_adjust(dot: usize, bardot: usize) {
      aligned("ADJUST", dot, bardot);
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn single_lost(dot: usize, bardot: usize) {
      aligned("LOST", dot, bardot);
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn single() {
      queue!(stdout(), MoveTo(X, Y + 6));
      print!("{:^WIDTH$}", "SINGLE");
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn double(found: bool, first: usize, second: usize, angle: f32, dist: f32) {
      if !found {
         return;
      }
      queue!(stdout(), MoveTo(X, Y + 4));
      let mut s1: String = (0..4)
         .map(|i| {
            if i == first {
               "       Ⓛ       "
            } else if i == second {
               "       Ⓡ       "
            } else {
               "               "
            }
         })
         .collect();
      s1.replace_range(0..2, "OK");
      println!("{}", s1);
      s1 = format!("{:>WIDTH$}", format!("dist: {:+.2}", dist));
      let s2 = format!("angle: {:+.2}°", angle.to_degrees());
      s1.replace_range(0..s2.len(), &s2);
      println!("{}", s1);
      print!("{:^WIDTH$}", "OK");
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn dead() {
      queue!(stdout(), MoveTo(X, Y + 2));
      println!();
      println!();
      println!();
      println!();
      print!("{:^WIDTH$}", "DEAD");
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn lost() {
      queue!(stdout(), MoveTo(X, Y + 2));
      println!();
      println!();
      println!();
      println!();
      print!("{:^WIDTH$}", "LOST");
   }
}

pub mod acc_pane {
   use super::*;

   #[cfg(feature = "tuning")]
   const X: u16 = 0;
   #[cfg(feature = "tuning")]
   const Y: u16 = 8;
   #[cfg(feature = "tuning")]
   const WIDTH: usize = 50;
   #[cfg(feature = "tuning")]
   const HEIGHT: u16 = 5;

   gen_begin!("accelerometer", "acc_pane");
   gen_end!("acc_pane");

   #[cfg(feature = "tuning")]
   fn f(a: f32, b: f32, c: Vec3, is_vec3: bool, ok: bool) -> String {
      let mut s = format!("{:<WIDTH$.3}", a);
      let b = format!("{:7.3} {}", b, if ok { " " } else { "●" });
      let c = format!(
         "{}{:+8.3},{:+8.3},{:+8.3}{}",
         if is_vec3 { "(" } else { "" },
         c.x,
         c.y,
         c.z,
         if is_vec3 { ")" } else { "" }
      );
      s.replace_range(s.len() - c.len()..s.len(), &c);
      s.replace_range(9..9 + b.chars().count(), &b);
      s
   }

   #[phantom_pub("tuning", "acc_pane")]
   pub fn line1(sd: f32, gravity: f32, data: Vec3, ok: bool) {
      queue!(stdout(), MoveTo(X, Y + 1));
      print!("{}", f(sd, gravity, data, true, ok));
   }

   #[phantom_pub("tuning", "acc_pane")]
   pub fn line2(acc_treshold: f32, acc: f32, smooth: Vec3, ok: bool) {
      queue!(stdout(), MoveTo(X, Y + 2));
      print!("{}", f(acc_treshold, acc, smooth, true, ok));
   }

   #[phantom_pub("tuning", "acc_pane")]
   pub fn line3(dist_treshold: f32, dist: f32, acc_tan: f32, dist_tan: f32, alpha: f32, ok: bool) {
      queue!(stdout(), MoveTo(X, Y + 3));
      print!(
         "{}",
         f(dist_treshold, dist, Vec3::new(acc_tan, dist_tan, alpha), false, ok)
      );
   }

   #[phantom_pub("tuning", "acc_pane")]
   pub fn status(roll: f32, status: bool) {
      queue!(stdout(), MoveTo(X, Y + 4));
      print!(
         "{:^WIDTH$}",
         format!(
            "{:>6} {:+8.3}°",
            if status { "OK" } else { "REJECT" },
            roll.to_degrees()
         )
      );
   }
}

pub mod smooth_pane {
   use super::*;

   #[cfg(feature = "tuning")]
   const X: u16 = 52;
   #[cfg(feature = "tuning")]
   const Y: u16 = 8;
   #[cfg(feature = "tuning")]
   const WIDTH: usize = 14;
   #[cfg(feature = "tuning")]
   const HEIGHT: u16 = 4;

   gen_begin!("smoothing", "smooth_pane");
   gen_end!("smooth_pane");

   #[phantom_pub("tuning", "smooth_pane")]
   fn smoothing(dist2: f32, mode: &str) {
      queue!(stdout(), MoveTo(X, Y + 3));
      print!("{:^WIDTH$}", format!("{:4.0} {:8}", dist2.sqrt() * 1023.0, mode));
   }

   #[phantom_pub("tuning", "smooth_pane")]
   pub fn deadzone(dist2: f32) {
      smoothing!(dist2, "deadzone");
   }

   #[phantom_pub("tuning", "smooth_pane")]
   pub fn inside(dist2: f32) {
      smoothing!(dist2, "inside");
   }

   #[phantom_pub("tuning", "smooth_pane")]
   pub fn outside(dist2: f32) {
      smoothing!(dist2, "outside");
   }

   #[phantom_pub("tuning", "smooth_pane")]
   pub fn counters(errors_left: u32, glitch_cnt: u32) {
      queue!(stdout(), MoveTo(X, Y + 1));
      println!(
         "{:^WIDTH$}",
         format!("errors: {}/{ERROR_MAX_COUNT}", ERROR_MAX_COUNT - errors_left)
      );
      print!("{:^WIDTH$}", format!("glitch: {}/{GLITCH_MAX_COUNT}", glitch_cnt));
   }
}
