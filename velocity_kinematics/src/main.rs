use nalgebra::{Scalar, Vector3};

fn print_vector<T: Scalar>(m: &Vector3<T>) {
  println!("{:?}", m)
}

fn main() {
  let v1 = Vector3::new(1, 2, 3);

  print_vector(&v1);
}
