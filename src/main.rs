extern crate native;
extern crate kiss3d;
extern crate nalgebra;

use nalgebra::na::Vec3;
use nalgebra::na;
use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use kiss3d::light;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    // have camera start from higher position
    let eye = Vec3::new(5.0, 5.0, 10.0);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);


    let mut window = Window::new("Kiss3d: cube");
    let mut c      = window.add_cube(1.0, 1.0, 1.0);
    let mut ground = window.add_quad(10.0, 10.0, 1, 1);
    ground.set_local_rotation(Vec3::new((90.0f32).to_radians(), 0.0, 0.0));
    ground.set_color(0.2, 0.2, 0.2);

    c.set_color(1.0, 0.0, 0.0);
    c.set_local_translation(Vec3::new(0.0, 0.5, 0.0));

    window.set_light(light::StickToCamera);

    while window.render_with_camera(&mut arc_ball) {
        c.prepend_to_local_rotation(&Vec3::new(0.0f32, 0.014, 0.0));
        
        let curr_time = window.context().get_time() as f32;
        c.set_local_translation(Vec3::new(curr_time.sin(), 0.5f32, curr_time.cos()));
    }
}
