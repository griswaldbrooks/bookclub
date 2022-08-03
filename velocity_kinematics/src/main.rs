use nalgebra as na;
use motion::Twist;

fn main() {
    let v1 = Twist {
        angular: na::Vector3::<f64>::new(1., 2., 3.),
        linear: na::Vector3::<f64>::new(4., 5., 6.),
    };

    println!("{:?}", &v1);
}
