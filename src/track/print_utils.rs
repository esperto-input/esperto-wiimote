#![allow(unused_must_use)]

use crate::points::Dot;
use crate::track::types::RawDot;
use const_format::{concatcp, str_repeat};
use crossterm::cursor::MoveTo;
use crossterm::cursor::{MoveDown, MoveToColumn};
use crossterm::style::Print;
use crossterm::terminal::{Clear, ClearType};
use crossterm::{Command, queue};
use crossterm::{cursor, execute};
use std::io::Write;
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

pub fn clear() {
   let mut so: Stdout = stdout();
   queue!(so, Clear(ClearType::All));
}

macro_rules! gen_end {
   () => {
      pub fn end() {
         #[cfg(feature = "tuning")]
         {
            let mut so: Stdout = stdout();
            queue!(
               so,
               MoveTo(X, Y + HEIGHT),
               Print(concatcp!("└", str_repeat!("─", WIDTH), "┘"))
            );
            so.flush().unwrap();
         }
      }
   };
}

macro_rules! gen_begin {
   ($title:literal) => {
      pub fn begin() {
         #[cfg(feature = "tuning")]
         {
            queue!(
               stdout(),
               MoveTo(X, Y),
               Print("┌"),
               Print(format!("{:─^WIDTH$}", $title)),
               Print("┐")
            );
         }
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
         // #[cfg(feature = "tuning")]
         queue!(
            stdout(),
            Print("│"),
            Print(format!("{:WIDTH$}", format!($($arg)*))),
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
   const HEIGHT: u16 = 8;

   gen_begin!("sensorbar");
   gen_end!();

   pub fn raw_raw_dots(raw_dots: &[RawDot; 4]) {
      #[cfg(feature = "tuning")]
      {
         let mut so = stdout();
         queue!(so, MoveTo(X, Y + 1), Print("│"));
         for dot in raw_dots {
            queue!(so, Print(format!("({:6},{:6})", dot.x, dot.y)));
         }
         queue!(so, Print("│"));
      }
   }

   pub fn raw_dots(raw_dots: &[RawDot; 4]) {
      #[cfg(feature = "tuning")]
      {
         let mut so = stdout();
         queue!(so, MoveTo(X, Y + 2), Print("│"));
         for dot in raw_dots {
            queue!(so, Print(format!("({:6},{:6})", dot.x, dot.y)));
         }
         queue!(so, Print("│"));
      }
   }

   pub fn dots(raw_dots: &[Dot], dots: &[Dot]) {
      #[cfg(feature = "tuning")]
      {
         queue!(stdout(), MoveTo(X, Y + 3));
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
   }

   fn aligned(mode: &'static str, dot: usize, bardot: usize) {
      #[cfg(feature = "tuning")]
      {
         queue!(stdout(), MoveTo(X, Y + 5));
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
   }

   pub fn single_adjust(dot: usize, bardot: usize) {
      #[cfg(feature = "tuning")]
      {
         aligned("ADJUST", dot, bardot);
      }
   }

   pub fn single_lost(dot: usize, bardot: usize) {
      #[cfg(feature = "tuning")]
      {
         aligned("LOST", dot, bardot);
      }
   }

   pub fn single() {
      #[cfg(feature = "tuning")]
      {
         queue!(stdout(), MoveTo(X, Y + 7));
         print!("{:^WIDTH$}", "SINGLE");
      }
   }

   pub fn double(found: bool, first: usize, second: usize, angle: f32, dist: f32) {
      #[cfg(feature = "tuning")]
      if found {
         queue!(stdout(), MoveTo(X, Y + 5));
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
         s1.replace_range((0..2), "OK");
         println!("{}", s1);
         s1 = format!("{:>WIDTH$}", format!("dist: {:+.2}", dist));
         let s2 = format!("angle: {:+.2}°", angle.to_degrees());
         s1.replace_range(0..s2.len(), &s2);
         println!("{}", s1);
         print!("{:^WIDTH$}", "OK");
      }
   }

   pub fn dead() {
      #[cfg(feature = "tuning")]
      {
         queue!(stdout(), MoveTo(X, Y + 5));
         println!();
         println!();
         print!("{:^WIDTH$}", "DEAD");
      }
   }

   pub fn lost() {
      #[cfg(feature = "tuning")]
      {
         queue!(stdout(), MoveTo(X, Y + 5));
         println!();
         println!();
         print!("{:^WIDTH$}", "LOST");
      }
   }
}

pub mod acc_pane {
   use super::*;

   const X: u16 = 0;
   const Y: u16 = 8;
   const WIDTH: usize = 20;
   const HEIGHT: u16 = 1;

   gen_begin!("accelerometer");
   gen_end!();
}
