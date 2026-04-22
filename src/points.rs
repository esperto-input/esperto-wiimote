use nalgebra::{vector, Vector2, Vector3};

pub type Dot = Vector2<f32>;

pub trait DotLike {
   fn rotate(&self, theta: f32) -> Self;

   fn off_angle(&self, dot: &Self) -> f32;

   fn offset(&self, dot: &Self) -> Self;

   fn distance2(&self, dot: &Self) -> f32;

   fn norm2(&self) -> f32;

   fn avg(&self, dot: &Self) -> Self;

   fn atan2(&self) -> f32;

   fn position(&self) -> Self;
}

impl DotLike for Dot {
   fn rotate(&self, theta: f32) -> Dot {
      let sin = theta.sin();
      let cos = theta.cos();
      vector![cos * self.x + -sin * self.y, sin * self.x + cos * self.y,]
   }

   fn off_angle(&self, dot: &Dot) -> f32 {
      self.offset(dot).atan2()
   }

   fn offset(&self, dot: &Dot) -> Dot {
      dot - self
   }

   fn distance2(&self, dot: &Dot) -> f32 {
      self.offset(dot).norm2()
   }

   fn norm2(&self) -> f32 {
      self.norm_squared()
   }

   fn avg(&self, dot: &Dot) -> Dot {
      (self + dot) / 2.0
   }

   fn atan2(&self) -> f32 {
      self.y.atan2(self.x)
   }

   fn position(&self) -> Dot {
      let dot: Dot = (self + vector![1.0, 1.0]) / 2.0;
      vector![(1.0 - dot.x).clamp(0.0, 1.0), dot.y.clamp(0.0, 1.0),]
   }
}

pub type Vec3 = Vector3<f32>;
