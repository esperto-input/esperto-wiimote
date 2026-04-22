#![allow(unused_must_use)]
#![allow(unused)]

use crate::points::Dot;
use crate::points::Vec3;
use crate::track::RawDot;
use crate::track::{ERROR_MAX_COUNT, GLITCH_MAX_COUNT};
use const_format::{concatcp, str_repeat};
use crossterm::{
   Command,
   cursor::{MoveDown, MoveTo, MoveToColumn},
   queue,
   style::Print,
   terminal::{Clear, ClearType},
};
use proc_macros::phantom_pub;
use std::io::Write;
use std::io::{Stdout, stdout};

#[macro_export]
macro_rules! dprintln {
    ($($arg:tt)*) => {
       #[cfg(all(debug_assertions,not(feature = "tuning")))]
       ::std::println!($($arg)*)
    };
}

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

pub fn clear() {
   let mut so: Stdout = stdout();
   queue!(so, Clear(ClearType::All));
}

struct NewLine {
   x: u16,
}

impl Command for NewLine {
   fn write_ansi(&self, f: &mut impl std::fmt::Write) -> std::fmt::Result {
      MoveDown(1).write_ansi(f)?;
      MoveToColumn(self.x).write_ansi(f)?;
      Ok(())
   }
}

#[allow(non_snake_case)]
fn Newline(x: u16) -> NewLine {
   NewLine { x }
}

macro_rules! gen_begin {
   ($(#[$meta:meta])*, $title:literal) => {
      $(#[$meta])*
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
   ($(#[$meta:meta])*) => {
      $(#[$meta])*
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
            Print(format!("{:^WIDTH$}", format!($($arg)*))),
            Print("│"),
         );
      };
}

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

   const X: u16 = 0;
   const Y: u16 = 0;
   const WIDTH: usize = 60;
   const HEIGHT: u16 = 7;

   gen_begin!(#[phantom_pub("tuning", "sensorbar_pane")], "sensorbar");
   gen_end!(#[phantom_pub("tuning", "sensorbar_pane")]);

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
         "{:<WIDTH$}",
         raw_dots
            .iter()
            .map(|dot| format!("({:+6.3},{:+6.3})", dot.x, dot.y))
            .collect::<String>()
      );
      print!(
         "{:<WIDTH$}",
         dots
            .iter()
            .map(|dot| format!("({:+6.3},{:+6.3})", dot.x, dot.y))
            .collect::<String>()
      );
   }

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
      print!("aligned {}", if bardot == 0 { "LEFT" } else { "RIGHT" });
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
      print!("SINGLE");
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
      print!("OK");
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn dead() {
      queue!(stdout(), MoveTo(X, Y + 2));
      println!();
      println!();
      println!();
      println!();
      print!("DEAD");
   }

   #[phantom_pub("tuning", "sensorbar_pane")]
   pub fn lost() {
      queue!(stdout(), MoveTo(X, Y + 2));
      println!();
      println!();
      println!();
      println!();
      print!("LOST");
   }
}

pub mod acc_pane {
   use super::*;

   const X: u16 = 0;
   const Y: u16 = 8;
   const WIDTH: usize = 50;
   const HEIGHT: u16 = 5;

   gen_begin!(#[phantom_pub("tuning", "acc_pane")], "accelerometer");
   gen_end!(#[phantom_pub("tuning", "acc_pane")]);

   fn f(a: f32, b: f32, c: Vec3, is_vec3: bool, ok: bool) -> String {
      let mut s = format!("{:<WIDTH$.3}", a);
      let b = format!("{:7.3} {}", b, if ok { " " } else { "●" });
      let c = format!(
         "{}{:+8.3},{:+8.3},{:+8.3}{}",
         if is_vec3 { "(" } else { " " },
         c.x,
         c.y,
         c.z,
         if is_vec3 { ")" } else { " " }
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
         "{:>6} {:+8.3}°",
         if status { "OK" } else { "REJECT" },
         roll.to_degrees()
      );
   }
}

pub mod smooth_pane {
   use super::*;

   const X: u16 = 52;
   const Y: u16 = 8;
   const WIDTH: usize = 14;
   const HEIGHT: u16 = 4;

   gen_begin!(#[phantom_pub("tuning", "smooth_pane")], "smoothing");
   gen_end!(#[phantom_pub("tuning", "smooth_pane")]);

   #[phantom_pub("tuning", "smooth_pane")]
   fn smoothing(dist2: f32, mode: &str) {
      queue!(stdout(), MoveTo(X, Y + 3));
      print!("{:4.0} {:8}", dist2.sqrt() * 1023.0, mode);
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
      println!("errors: {}/{ERROR_MAX_COUNT}", ERROR_MAX_COUNT - errors_left);
      print!("glitch: {}/{GLITCH_MAX_COUNT}", glitch_cnt);
   }
}

pub mod calibration_pane {
   use super::*;
   use crate::calibration::Position;

   const X: u16 = 0;
   const Y: u16 = 0;
   const WIDTH: usize = 30;
   const HEIGHT: u16 = 4;

   gen_begin!(,"calibration");
   gen_end!();

   pub fn warming_up(countdown: i32) {
      queue!(stdout(), MoveTo(X, Y + 1));
      println!();
      println!("Warming up!  {:2}", countdown);
      println!();
      print!();
   }

   pub fn progress(progress: usize) {
      queue!(stdout(), MoveTo(X, Y + 3));
      print!("[{:▯<20}]", "▮".repeat(progress));
   }

   pub fn sds(sds: Vec3) {
      queue!(stdout(), MoveTo(X, Y + 1));
      print!("({:+8.3},{:+8.3},{:+8.3})", sds.x, sds.y, sds.z);
   }

   pub fn avgs(avgs: Vec3) {
      queue!(stdout(), MoveTo(X, Y + 2));
      print!("({:+8.3},{:+8.3},{:+8.3})", avgs.x, avgs.y, avgs.z);
   }

   pub fn splash(position: Position) {
      begin();
      queue!(stdout(), MoveTo(X, Y + 1));
      println!("Place the wiimote");
      match position {
         Position::PosX => {println!("right side down");}
         Position::NegX => {println!("right side up");}
         Position::PosY => {println!("sensor side down");}
         Position::NegY => {println!("sensor side up");}
         Position::PosZ => {println!("face side up");}
         Position::NegZ => {println!("face side down");}
      }
      print!("press any key to continue");
      end();
   }

   pub fn optimizing() {
      begin();
      queue!(stdout(), MoveTo(X, Y + 1));
      println!();
      println!("Optimizing...");
      print!();
      end();
   }

   pub fn done() {
      begin();
      queue!(stdout(), MoveTo(X, Y + 1));
      println!();
      println!("DONE !");
      print!();
      end();
   }
}