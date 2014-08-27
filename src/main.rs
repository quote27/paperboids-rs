extern crate native;
extern crate kiss3d;
extern crate nalgebra;

use std::rc::Rc;
use std::cell::RefCell;
use nalgebra::na::Vec3;
use nalgebra::na;
use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use kiss3d::resource::Mesh;
use kiss3d::light;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

//TODO: check out recorder demo to create mpg's of simulations

fn main() {
    // have camera start from higher position
    let eye = Vec3::new(5.0, 5.0, 10.0);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);

    let mut window = Window::new("Kiss3d: cube");
    window.set_framerate_limit(Some(120));

    let mut ground = window.add_quad(10.0, 10.0, 1, 1);
    ground.set_local_rotation(Vec3::new((90.0f32).to_radians(), 0.0, 0.0));
    ground.set_color(0.2, 0.2, 0.2);

    //let mut c = window.add_cube(1.0, 1.0, 1.0);
    //c.set_color(1.0, 0.0, 0.0);
    //c.set_local_translation(Vec3::new(0.0, 0.5, 0.0));

    let pmesh = plane_mesh();
    let mut p = window.add_mesh(pmesh.clone(), Vec3::new(0.5, 0.5, 0.5));
    p.set_color(1.0, 1.0, 1.0);
    p.enable_backface_culling(false);
    p.set_local_translation(Vec3::new(0.0f32, 2.0, 0.0));
    p.set_points_size(2.0); //wireframe mode for plane
    p.set_lines_width(1.0);
    p.set_surface_rendering_activation(false);


    let mut p2 = window.add_mesh(pmesh.clone(), Vec3::new(0.5, 0.5, 0.5));
    p2.set_color(1.0, 1.0, 1.0);
    p2.enable_backface_culling(false);
    p2.set_local_translation(Vec3::new(0.0f32, 2.0, 0.0));
    p2.set_points_size(2.0); //wireframe mode for plane
    p2.set_lines_width(1.0);
    p2.set_surface_rendering_activation(false);

    window.set_light(light::StickToCamera);

    while window.render_with_camera(&mut arc_ball) {
        draw_axis(&mut window);
        
        let curr_time = window.context().get_time() as f32;
        //c.prepend_to_local_rotation(&Vec3::new(0.0f32, 0.014, 0.0));
        //c.set_local_translation(Vec3::new(curr_time.sin(), 0.5f32, curr_time.cos()));

        let csin = curr_time.sin();
        let ccos = curr_time.cos();
        p.set_local_translation(Vec3::new(0.0, csin * 0.5 + 2.0, 0.0));
        p2.set_local_translation(Vec3::new(csin * 0.5 + 1.0, ccos * 0.5 + 1.0, csin * 0.5));
        if csin > 0.0 {
            p.prepend_to_local_rotation(&Vec3::new((0.1f32).to_radians() * csin, 0.0, 0.0));
            p2.prepend_to_local_rotation(&Vec3::new((0.1f32).to_radians() * ccos, 0.0, 0.0));
        } else {
            p.prepend_to_local_rotation(&Vec3::new((-0.1f32).to_radians() * -csin, 0.0, 0.0));
            p2.prepend_to_local_rotation(&Vec3::new((-0.1f32).to_radians() * -ccos, 0.0, 0.0));
        }
    }
}

fn plane_mesh() -> Rc<RefCell<Mesh>> {
    let vertices = vec!(
        Vec3::new(0.0, 0.0, 1.0), // front / nose
        Vec3::new(0.75, 0.0, -1.0), // left wing - 'port'
        Vec3::new(-0.75, 0.0, -1.0), // right wing - 'starboard'
        Vec3::new(0.0, 0.0, -1.0), // back midpoint between wings
        Vec3::new(0.0, -0.4, -1.0), // back bottom fin
    );

    let indices = vec!(
        Vec3::new(0u32, 1, 3),
        Vec3::new(0u32, 3, 2),
        Vec3::new(0u32, 4, 3),
    );

    Rc::new(RefCell::new(Mesh::new(vertices, indices, None, None, false)))
}

fn draw_axis(w: &mut Window) {
    let o: Vec3<f32> = na::zero();
    let x = Vec3::x();
    let y = Vec3::y();
    let z = Vec3::z();

    w.draw_line(&o, &x, &x);
    w.draw_line(&o, &y, &y);
    w.draw_line(&o, &z, &z);
}
