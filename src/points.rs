use std::fmt::Display;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};
#[derive(Clone, Copy, Default, Debug)]
pub struct Dot {
   pub x: f32,
   pub y: f32,
}

impl Display for Dot {
   fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
      write!(f, "({:.3}, {:.3})", self.x, self.y)
   }
}

impl Dot {
   const MAX_WIDTH: f32 = 512.0 / 512.0;
   const MAX_HEIGHT: f32 = 384.0 / 512.0;

   pub fn rotate(&self, theta: f32) -> Dot {
      let sin = theta.sin();
      let cos = theta.cos();
      Dot {
         x: cos * self.x + -sin * self.y,
         y: sin * self.x + cos * self.y,
      }
   }

   pub fn slope(&self, dot: &Dot) -> f32 {
      let offset = self.offset(dot);
      offset.y.atan2(offset.x)
   }

   pub fn offset(&self, dot: &Dot) -> Dot {
      dot - self
   }

   pub fn distance(&self, dot: &Dot) -> f32 {
      self.offset(dot).norm()
   }

   pub fn norm(&self) -> f32 {
      self.x.powi(2) + self.y.powi(2)
   }

   // pub fn edge_distance(&self) -> f32 {
   //    f32::min(Dot::MAX_WIDTH - self.x.abs(), Dot::MAX_HEIGHT - self.y.abs())
   // }
}

impl AddAssign for Dot {
   fn add_assign(&mut self, other: Dot) {
      self.x += other.x;
      self.y += other.y;
   }
}

impl SubAssign for Dot {
   fn sub_assign(&mut self, other: Dot) {
      self.x -= other.x;
      self.y -= other.y;
   }
}

impl AddAssign<&Dot> for Dot {
   fn add_assign(&mut self, other: &Dot) {
      self.x += other.x;
      self.y += other.y;
   }
}

impl SubAssign<&Dot> for Dot {
   fn sub_assign(&mut self, other: &Dot) {
      self.x -= other.x;
      self.y -= other.y;
   }
}

impl AddAssign<f32> for Dot {
   fn add_assign(&mut self, other: f32) {
      self.x += other;
      self.y += other;
   }
}

impl SubAssign<f32> for Dot {
   fn sub_assign(&mut self, other: f32) {
      self.x -= other;
      self.y -= other;
   }
}

impl AddAssign<&f32> for Dot {
   fn add_assign(&mut self, other: &f32) {
      self.x += other;
      self.y += other;
   }
}

impl SubAssign<&f32> for Dot {
   fn sub_assign(&mut self, other: &f32) {
      self.x -= other;
      self.y -= other;
   }
}

impl DivAssign<f32> for Dot {
   fn div_assign(&mut self, other: f32) {
      self.x /= other;
      self.y /= other;
   }
}

impl MulAssign<f32> for Dot {
   fn mul_assign(&mut self, other: f32) {
      self.x *= other;
      self.y *= other;
   }
}

impl DivAssign<&f32> for Dot {
   fn div_assign(&mut self, other: &f32) {
      self.x /= other;
      self.y /= other;
   }
}

impl MulAssign<&f32> for Dot {
   fn mul_assign(&mut self, other: &f32) {
      self.x *= other;
      self.y *= other;
   }
}

impl Add for Dot {
   type Output = Self;

   fn add(self, rhs: Self) -> Self::Output {
      Self {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
      }
   }
}

impl Sub for Dot {
   type Output = Self;

   fn sub(self, rhs: Self) -> Self::Output {
      Self {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
      }
   }
}

impl Add<Dot> for &Dot {
   type Output = Dot;

   fn add(self, rhs: Dot) -> Self::Output {
      Dot {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
      }
   }
}

impl Sub<Dot> for &Dot {
   type Output = Dot;

   fn sub(self, rhs: Dot) -> Self::Output {
      Dot {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
      }
   }
}

impl Add for &Dot {
   type Output = Dot;

   fn add(self, rhs: Self) -> Self::Output {
      Dot {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
      }
   }
}

impl Sub for &Dot {
   type Output = Dot;

   fn sub(self, rhs: Self) -> Self::Output {
      Dot {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
      }
   }
}

impl Add<&Dot> for Dot {
   type Output = Self;

   fn add(self, rhs: &Dot) -> Self::Output {
      Self {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
      }
   }
}

impl Sub<&Dot> for Dot {
   type Output = Self;

   fn sub(self, rhs: &Dot) -> Self::Output {
      Self {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
      }
   }
}

impl Add<f32> for Dot {
   type Output = Self;

   fn add(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x + rhs,
         y: self.y + rhs,
      }
   }
}

impl Sub<f32> for Dot {
   type Output = Self;

   fn sub(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x - rhs,
         y: self.y - rhs,
      }
   }
}

impl Add<&f32> for Dot {
   type Output = Self;

   fn add(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x + rhs,
         y: self.y + rhs,
      }
   }
}

impl Sub<&f32> for Dot {
   type Output = Self;

   fn sub(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x - rhs,
         y: self.y - rhs,
      }
   }
}

impl Add<f32> for &Dot {
   type Output = Dot;

   fn add(self, rhs: f32) -> Self::Output {
      Dot {
         x: self.x + rhs,
         y: self.y + rhs,
      }
   }
}

impl Sub<f32> for &Dot {
   type Output = Dot;

   fn sub(self, rhs: f32) -> Self::Output {
      Dot {
         x: self.x - rhs,
         y: self.y - rhs,
      }
   }
}

impl Add<&f32> for &Dot {
   type Output = Dot;

   fn add(self, rhs: &f32) -> Self::Output {
      Dot {
         x: self.x + rhs,
         y: self.y + rhs,
      }
   }
}

impl Sub<&f32> for &Dot {
   type Output = Dot;

   fn sub(self, rhs: &f32) -> Self::Output {
      Dot {
         x: self.x - rhs,
         y: self.y - rhs,
      }
   }
}

impl Mul<f32> for Dot {
   type Output = Self;

   fn mul(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x * rhs,
         y: self.y * rhs,
      }
   }
}

impl Div<f32> for Dot {
   type Output = Self;

   fn div(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x / rhs,
         y: self.y / rhs,
      }
   }
}

impl Mul<&f32> for Dot {
   type Output = Self;

   fn mul(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x * rhs,
         y: self.y * rhs,
      }
   }
}

impl Div<&f32> for Dot {
   type Output = Self;

   fn div(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x / rhs,
         y: self.y / rhs,
      }
   }
}

impl Mul<f32> for &Dot {
   type Output = Dot;

   fn mul(self, rhs: f32) -> Self::Output {
      Dot {
         x: self.x * rhs,
         y: self.y * rhs,
      }
   }
}

impl Div<f32> for &Dot {
   type Output = Dot;

   fn div(self, rhs: f32) -> Self::Output {
      Dot {
         x: self.x / rhs,
         y: self.y / rhs,
      }
   }
}

impl Mul<&f32> for &Dot {
   type Output = Dot;

   fn mul(self, rhs: &f32) -> Self::Output {
      Dot {
         x: self.x * rhs,
         y: self.y * rhs,
      }
   }
}

impl Div<&f32> for &Dot {
   type Output = Dot;

   fn div(self, rhs: &f32) -> Self::Output {
      Dot {
         x: self.x / rhs,
         y: self.y / rhs,
      }
   }
}

#[derive(Debug, Copy, Clone, Default)]
pub struct Vec3 {
   pub x: f32,
   pub y: f32,
   pub z: f32,
}

impl Vec3 {
   pub fn norm(&self) -> f32 {
      (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
   }
}

impl AddAssign for Vec3 {
   fn add_assign(&mut self, other: Vec3) {
      self.x += other.x;
      self.y += other.y;
      self.z += other.z;
   }
}

impl SubAssign for Vec3 {
   fn sub_assign(&mut self, other: Vec3) {
      self.x -= other.x;
      self.y -= other.y;
      self.z -= other.z;
   }
}

impl AddAssign<&Vec3> for Vec3 {
   fn add_assign(&mut self, other: &Vec3) {
      self.x += other.x;
      self.y += other.y;
      self.z += other.z;
   }
}

impl SubAssign<&Vec3> for Vec3 {
   fn sub_assign(&mut self, other: &Vec3) {
      self.x -= other.x;
      self.y -= other.y;
      self.z -= other.z;
   }
}

impl AddAssign<f32> for Vec3 {
   fn add_assign(&mut self, other: f32) {
      self.x += other;
      self.y += other;
      self.z += other;
   }
}

impl SubAssign<f32> for Vec3 {
   fn sub_assign(&mut self, other: f32) {
      self.x -= other;
      self.y -= other;
      self.z -= other;
   }
}

impl AddAssign<&f32> for Vec3 {
   fn add_assign(&mut self, other: &f32) {
      self.x += other;
      self.y += other;
      self.z += other;
   }
}

impl SubAssign<&f32> for Vec3 {
   fn sub_assign(&mut self, other: &f32) {
      self.x -= other;
      self.y -= other;
      self.z -= other;
   }
}

impl DivAssign<f32> for Vec3 {
   fn div_assign(&mut self, other: f32) {
      self.x /= other;
      self.y /= other;
      self.z /= other;
   }
}

impl MulAssign<f32> for Vec3 {
   fn mul_assign(&mut self, other: f32) {
      self.x *= other;
      self.y *= other;
      self.z *= other;
   }
}

impl DivAssign<&f32> for Vec3 {
   fn div_assign(&mut self, other: &f32) {
      self.x /= other;
      self.y /= other;
      self.z /= other;
   }
}

impl MulAssign<&f32> for Vec3 {
   fn mul_assign(&mut self, other: &f32) {
      self.x *= other;
      self.y *= other;
      self.z *= other;
   }
}

impl Add for Vec3 {
   type Output = Self;

   fn add(self, rhs: Self) -> Self::Output {
      Self {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
         z: self.z + rhs.z,
      }
   }
}

impl Sub for Vec3 {
   type Output = Self;

   fn sub(self, rhs: Self) -> Self::Output {
      Self {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
         z: self.z - rhs.z,
      }
   }
}

impl Add<Vec3> for &Vec3 {
   type Output = Vec3;

   fn add(self, rhs: Vec3) -> Self::Output {
      Vec3 {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
         z: self.z + rhs.z,
      }
   }
}

impl Sub<Vec3> for &Vec3 {
   type Output = Vec3;

   fn sub(self, rhs: Vec3) -> Self::Output {
      Vec3 {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
         z: self.z - rhs.z,
      }
   }
}

impl Add for &Vec3 {
   type Output = Vec3;

   fn add(self, rhs: Self) -> Self::Output {
      Vec3 {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
         z: self.z + rhs.z,
      }
   }
}

impl Sub for &Vec3 {
   type Output = Vec3;

   fn sub(self, rhs: Self) -> Self::Output {
      Vec3 {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
         z: self.z - rhs.z,
      }
   }
}

impl Add<&Vec3> for Vec3 {
   type Output = Self;

   fn add(self, rhs: &Vec3) -> Self::Output {
      Self {
         x: self.x + rhs.x,
         y: self.y + rhs.y,
         z: self.z + rhs.z,
      }
   }
}

impl Sub<&Vec3> for Vec3 {
   type Output = Self;

   fn sub(self, rhs: &Vec3) -> Self::Output {
      Self {
         x: self.x - rhs.x,
         y: self.y - rhs.y,
         z: self.z - rhs.z,
      }
   }
}

impl Add<f32> for Vec3 {
   type Output = Self;

   fn add(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x + rhs,
         y: self.y + rhs,
         z: self.z + rhs,
      }
   }
}

impl Sub<f32> for Vec3 {
   type Output = Self;

   fn sub(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x - rhs,
         y: self.y - rhs,
         z: self.z - rhs,
      }
   }
}

impl Add<&f32> for Vec3 {
   type Output = Self;

   fn add(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x + rhs,
         y: self.y + rhs,
         z: self.z + rhs,
      }
   }
}

impl Sub<&f32> for Vec3 {
   type Output = Self;

   fn sub(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x - rhs,
         y: self.y - rhs,
         z: self.z - rhs,
      }
   }
}

impl Add<f32> for &Vec3 {
   type Output = Vec3;

   fn add(self, rhs: f32) -> Self::Output {
      Vec3 {
         x: self.x + rhs,
         y: self.y + rhs,
         z: self.z + rhs,
      }
   }
}

impl Sub<f32> for &Vec3 {
   type Output = Vec3;

   fn sub(self, rhs: f32) -> Self::Output {
      Vec3 {
         x: self.x - rhs,
         y: self.y - rhs,
         z: self.z - rhs,
      }
   }
}

impl Add<&f32> for &Vec3 {
   type Output = Vec3;

   fn add(self, rhs: &f32) -> Self::Output {
      Vec3 {
         x: self.x + rhs,
         y: self.y + rhs,
         z: self.z + rhs,
      }
   }
}

impl Sub<&f32> for &Vec3 {
   type Output = Vec3;

   fn sub(self, rhs: &f32) -> Self::Output {
      Vec3 {
         x: self.x - rhs,
         y: self.y - rhs,
         z: self.z - rhs,
      }
   }
}

impl Mul<f32> for Vec3 {
   type Output = Self;

   fn mul(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x * rhs,
         y: self.y * rhs,
         z: self.z * rhs,
      }
   }
}

impl Div<f32> for Vec3 {
   type Output = Self;

   fn div(self, rhs: f32) -> Self::Output {
      Self {
         x: self.x / rhs,
         y: self.y / rhs,
         z: self.z / rhs,
      }
   }
}

impl Mul<&f32> for Vec3 {
   type Output = Self;

   fn mul(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x * rhs,
         y: self.y * rhs,
         z: self.z * rhs,
      }
   }
}

impl Div<&f32> for Vec3 {
   type Output = Self;

   fn div(self, rhs: &f32) -> Self::Output {
      Self {
         x: self.x / rhs,
         y: self.y / rhs,
         z: self.z / rhs,
      }
   }
}

impl Mul<f32> for &Vec3 {
   type Output = Vec3;

   fn mul(self, rhs: f32) -> Self::Output {
      Vec3 {
         x: self.x * rhs,
         y: self.y * rhs,
         z: self.z * rhs,
      }
   }
}

impl Div<f32> for &Vec3 {
   type Output = Vec3;

   fn div(self, rhs: f32) -> Self::Output {
      Vec3 {
         x: self.x / rhs,
         y: self.y / rhs,
         z: self.z / rhs,
      }
   }
}

impl Mul<&f32> for &Vec3 {
   type Output = Vec3;

   fn mul(self, rhs: &f32) -> Self::Output {
      Vec3 {
         x: self.x * rhs,
         y: self.y * rhs,
         z: self.z * rhs,
      }
   }
}

impl Div<&f32> for &Vec3 {
   type Output = Vec3;

   fn div(self, rhs: &f32) -> Self::Output {
      Vec3 {
         x: self.x / rhs,
         y: self.y / rhs,
         z: self.z / rhs,
      }
   }
}
