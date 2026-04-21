use crate::points::Vec3;
use crate::regprintln;
use nalgebra::{Matrix, Matrix4x3, OMatrix, SMatrix, U3, U4, matrix, vector};
const EPSILON: f32 = f32::EPSILON;

#[derive(Default)]
pub struct Welford3 {
   pub average: Vec3,
   pub deviations: Vec3,
   count: f32,
}

impl Welford3 {
   pub fn add_value(&mut self, value: Vec3) {
      let old_average = self.average;
      self.count += 1.0;

      self.average += (value - self.average) / (self.count);
      self.deviations += (value - old_average).component_mul(&(value - self.average));
   }

   pub fn average(&self) -> Vec3 {
      self.average
   }

   pub fn variance(&self) -> Vec3 {
      self.deviations / self.count
   }
}

pub fn symmetrize(pos_x: Vec3, neg_x: Vec3, pos_y: Vec3, neg_y: Vec3, pos_z: Vec3, neg_z: Vec3) -> Vec3 {
   let mut diffs = [0.0; 6];
   let mut norm_diffs = [0.0; 6];
   let mut norms = [
      pos_z.norm(),
      neg_z.norm(),
      pos_x.norm(),
      neg_x.norm(),
      pos_y.norm(),
      neg_y.norm(),
   ];
   let zero_offset = Vec3 {
      x: (pos_z.x + neg_z.x + pos_y.x + neg_y.x) / 4.0,
      y: (pos_z.y + neg_z.y + pos_x.y + neg_x.y) / 4.0,
      z: (pos_x.z + neg_x.z + pos_y.z + neg_y.z) / 4.0,
   };
   let mut average: f32;
   let mut score = norms.iter().sum::<f32>();
   let mut old_score = score * 1.1;
   let mut offset = Vec3::default(); //zero_offset;
   regprintln!("score: {:?}, old_score: {:?}", score, old_score);

   while old_score - score > EPSILON {
      regprintln!("\nnew iteration");
      let mut norms = [
         (pos_z - offset).norm(),
         (neg_z - offset).norm(),
         (pos_x - offset).norm(),
         (neg_x - offset).norm(),
         (pos_y - offset).norm(),
         (neg_y - offset).norm(),
      ];
      regprintln!("norms: {:?}", norms);
      average = norms.iter().sum::<f32>() / 6.0;
      regprintln!("average: {:?}", average);
      // norm_diffs = norms.map(|n| n - average);
      norm_diffs[0] = norms[0] - norms[1];
      norm_diffs[1] = norms[1] - norms[0];
      norm_diffs[2] = norms[2] - norms[3];
      norm_diffs[3] = norms[3] - norms[2];
      norm_diffs[4] = norms[4] - norms[5];
      norm_diffs[5] = norms[5] - norms[4];
      regprintln!("norm_diffs: {:?}", norm_diffs);
      old_score = score;
      score = norm_diffs.map(f32::abs).iter().sum();
      regprintln!("score: {:?}, old_score: {:?}", score, old_score);

      diffs[0] = pos_z.z - norm_diffs[0];
      diffs[1] = neg_z.z - norm_diffs[1];
      diffs[2] = pos_x.x - norm_diffs[2];
      diffs[3] = neg_x.x - norm_diffs[3];
      diffs[4] = pos_y.y - norm_diffs[4];
      diffs[5] = neg_y.y - norm_diffs[5];

      regprintln!("diffs: {:?}", diffs);

      let opposing_offset = Vec3 {
         x: (diffs[2] + diffs[3]) / 2.0,
         y: (diffs[4] + diffs[5]) / 2.0,
         z: (diffs[0] + diffs[1]) / 2.0,
      };
      offset = opposing_offset / 1.0;

      regprintln!("offset: {:?}", offset);
   }

   offset
}

pub fn optimize(pos_x: Vec3, neg_x: Vec3, pos_y: Vec3, neg_y: Vec3, pos_z: Vec3, neg_z: Vec3) -> OMatrix<f32, U3, U4> {
   let sample_matrix: SMatrix<f32, 18, 12> = matrix![
      pos_x.x, pos_x.y, pos_x.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     pos_x.x, pos_x.y, pos_x.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     pos_x.x, pos_x.y, pos_x.z, 1.0;
      neg_x.x, neg_x.y, neg_x.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     neg_x.x, neg_x.y, neg_x.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     neg_x.x, neg_x.y, neg_x.z, 1.0;
      pos_y.x, pos_y.y, pos_y.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     pos_y.x, pos_y.y, pos_y.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     pos_y.x, pos_y.y, pos_y.z, 1.0;
      neg_y.x, neg_y.y, neg_y.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     neg_y.x, neg_y.y, neg_y.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     neg_y.x, neg_y.y, neg_y.z, 1.0;
      pos_z.x, pos_z.y, pos_z.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     pos_z.x, pos_z.y, pos_z.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     pos_z.x, pos_z.y, pos_z.z, 1.0;
      neg_z.x, neg_z.y, neg_z.z, 1.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     neg_z.x, neg_z.y, neg_z.z, 1.0,     0.0,     0.0,     0.0,     0.0;
      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     neg_z.x, neg_z.y, neg_z.z, 1.0;
   ];

   let target_vector: SMatrix<f32, 18, 1> = vector![
      98.0, 0.0, 0.0, -98.0, 0.0, 0.0, 0.0, 98.0, 0.0, 0.0, -98.0, 0.0, 0.0, 0.0, 98.0, 0.0, 0.0, -98.0,
   ];

   let weight = 500.0;
   let weights = SMatrix::<f32, 18, 18>::from_diagonal(
      &vector![
         1.0 * weight,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0,
         1.0 * weight,
         1.0,
         1.0,
         1.0 * weight,
      ]
      .map(f32::sqrt),
   );

   // --- SVD Solve ---
   // We compute the SVD of M. 'true, true' means compute U and V matrices.
   let svd = (weights * sample_matrix).svd(true, true);

   // solve() uses the Moore-Penrose pseudoinverse logic internally.
   // The epsilon (1e-7) handles rank-deficiency (colinearity).
   let p = svd.solve(&(weights * target_vector), 1e-10).expect("SVD solve failed");

   // Reshape the 12x1 vector into a 3x4 Affine Matrix
   let mut affine_matrix = Matrix4x3::from_column_slice(p.as_slice()).transpose();

   println!("Calibrated line vector:\n{:}", p);
   println!("Calibrated 4x3 Affine Matrix:\n{:}", affine_matrix);

   affine_matrix
}
