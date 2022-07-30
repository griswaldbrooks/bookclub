extern crate nalgebra as na;
// use na::{Scalar, Vector3};

// S = [-ω x q, ω]^T = [v, ω]^T
//   [ω] = skew symmetric matrix
//   R = I + [ω]sinθ + [ω]^2(1 - cosθ) // rodriguez
//   p = (Iθ + (1 − cos θ)[ω] + (θ − sin θ)[ω]^2)v
//   e^[S]θ = [R p]
//            [0 1]
//   Ad_e^[S]θ = [ R   0]
//               [[p]R R]

#[derive(Debug)]
struct Twist {
    angular: na::Vector3<f64>,
    linear: na::Vector3<f64>,
}

struct Screw {
    axis: Twist,
}

impl Screw {
    fn from(&self, axis: Twist) -> Screw {
        Screw { axis }
    }
}

fn main() {
    let v1 = Twist {
        angular: na::Vector3::<f64>::new(1., 2., 3.),
        linear: na::Vector3::<f64>::new(4., 5., 6.),
    };

    println!("{:?}", &v1);
}
