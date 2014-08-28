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
use kiss3d::scene::SceneNode;
use kiss3d::light;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

//TODO: check out recorder demo to create mpg's of simulations


struct Plane {
    pos: Vec3<f32>,
    vel: Vec3<f32>,
    acc: Vec3<f32>,

    node: SceneNode,
}
impl Plane {
    fn gen_mesh() -> Rc<RefCell<Mesh>> {
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

    fn new(sn: SceneNode) -> Plane {
        let mut node = sn;
        node.set_color(1.0, 1.0, 1.0);
        node.enable_backface_culling(false);
        node.set_local_translation(Vec3::new(0.0f32, 2.0, 0.0));
        node.set_points_size(1.0); //wireframe mode for plane
        node.set_lines_width(1.0);
        node.set_surface_rendering_activation(false);


        Plane {
            pos: Vec3::new(0.0, 2.0, 0.0),
            vel: Vec3::new(0.5, 0.0, 1.0),
            acc: Vec3::new(0.0, 0.0, 0.0),
            node: node,
        }
    }

    fn update(&mut self, dt: f32) {
        let dtv = Vec3::new(dt, dt, dt);
        self.vel = self.vel + self.acc * dtv;
        self.pos = self.pos + self.vel * dtv;
        self.node.set_local_translation(self.pos);
        self.node.look_at_z(&self.pos, &(self.pos + self.vel), &Vec3::y());
    }

    fn push(&mut self, v: &Vec3<f32>) {
        self.vel = self.vel + *v;
    }
}



fn main() {
    // have camera start from higher position
    let eye = Vec3::new(5.0, 5.0, 10.0);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);

    let mut window = Window::new("Kiss3d: cube");
    window.set_framerate_limit(Some(60));

    let mut ground = window.add_quad(10.0, 10.0, 1, 1);
    ground.set_local_rotation(Vec3::new((90.0f32).to_radians(), 0.0, 0.0));
    ground.set_color(0.2, 0.2, 0.2);

    //let mut c = window.add_cube(1.0, 1.0, 1.0);
    //c.set_color(1.0, 0.0, 0.0);
    //c.set_local_translation(Vec3::new(0.0, 0.5, 0.0));

    let pmesh = Plane::gen_mesh();
    let mut p = Plane::new(window.add_mesh(pmesh.clone(), Vec3::new(0.2, 0.2, 0.2)));

    let mut p2 = window.add_mesh(pmesh.clone(), Vec3::new(0.5, 0.5, 0.5));
    p2.set_color(1.0, 1.0, 1.0);
    p2.enable_backface_culling(false);
    p2.set_local_translation(Vec3::new(0.0f32, 2.0, 0.0));
    p2.set_points_size(1.0); //wireframe mode for plane
    p2.set_lines_width(1.0);
    p2.set_surface_rendering_activation(false);

    window.set_light(light::StickToCamera);

    let mut last_time = window.context().get_time() as f32;
    let mut curr_time;
    while window.render_with_camera(&mut arc_ball) {
        draw_axis(&mut window);

        curr_time = window.context().get_time() as f32;
        //c.prepend_to_local_rotation(&Vec3::new(0.0f32, 0.014, 0.0));
        //c.set_local_translation(Vec3::new(curr_time.sin(), 0.5f32, curr_time.cos()));

        p.update(curr_time - last_time);

        let csin = curr_time.sin();
        let ccos = curr_time.cos();
        p2.set_local_translation(Vec3::new(csin * 0.5 + 1.0, ccos * 0.5 + 1.0, csin * 0.5));
        if csin > 0.0 {
            p2.prepend_to_local_rotation(&Vec3::new((0.1f32).to_radians() * ccos, 0.0, 0.0));
        } else {
            p2.prepend_to_local_rotation(&Vec3::new((-0.1f32).to_radians() * -ccos, 0.0, 0.0));
        }

        p.acc.x =
            if p.pos.x > 5.0 {
                -1.0
            } else if p.pos.x < -5.0 {
                1.0
            } else {
                0.0
            };

        p.acc.z =
            if p.pos.z > 5.0 {
                -1.0
            } else if p.pos.z < -5.0 {
                1.0
            } else {
                0.0
            };

        last_time = curr_time;
    }
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
