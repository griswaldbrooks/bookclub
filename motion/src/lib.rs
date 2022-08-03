use nalgebra as na;
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
pub struct Twist {
    pub angular: na::Vector3<f64>,
    pub linear: na::Vector3<f64>,
}

pub struct Screw {
    axis: Twist,
}

impl Screw {
    fn from(&self, axis: Twist) -> Screw {
        Screw { axis }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
