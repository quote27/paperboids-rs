extern crate native;
extern crate kiss3d;
extern crate nalgebra;
extern crate debug; //to print out type of variable at runtime

use std::rc::Rc;
use std::cell::RefCell;
use std::rand;
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

// reference: http://www.vergenet.net/~conrad/boids/pseudocode.html

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
        node.set_points_size(1.0); //wireframe mode for plane
        node.set_lines_width(1.0);
        node.set_surface_rendering_activation(false);

        let x = 25.0 - rand::random::<f32>() * 50.0;
        let y = 25.0 - rand::random::<f32>() * 20.0;
        let z = 25.0 - rand::random::<f32>() * 50.0;

        let vx = 10.0 - rand::random::<f32>() * 20.0;
        let vy = 10.0 - rand::random::<f32>() * 20.0;
        let vz = 10.0 - rand::random::<f32>() * 20.0;

        Plane {
            pos: Vec3::new(x, y, z),
            vel: Vec3::new(vx, vy, vz),
            acc: Vec3::new(0.0, 0.0, 0.0),
            node: node,
        }
    }

    fn update(&mut self, dt: f32) {
        self.vel = self.vel + self.acc * dt;

        let world_scale = 0.2;
        let max_speed = 20.0 * world_scale;
        let min_speed = 2.0 * world_scale;
        let curr_speed = na::norm(&self.vel);
        if curr_speed > max_speed {
            self.vel = self.vel / curr_speed * max_speed;
        } else if curr_speed < min_speed {
            self.vel = self.vel / curr_speed * min_speed;
        }

        self.pos = self.pos + self.vel * dt;
        self.node.look_at_z(&self.pos, &(self.pos + self.vel), &Vec3::y());
    }
} 

fn flock_center_v(ps: &Vec<Plane>, i: uint, look_radius: f32) -> Vec3<f32> {
    let mut neighbors = 0u;
    let perceived_center = ps.iter()
        .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
        .fold(na::zero::<Vec3<f32>>(), |a, ref p| { neighbors += 1; a + p.pos });
    neighbors -= 1; // this is because the bird itself is caught by the filter

    if neighbors == 0 {
        na::zero()
    } else {
        na::normalize(&(((perceived_center - ps[i].pos) / neighbors as f32) - ps[i].pos))
    }
}

fn keep_away_v(ps: &Vec<Plane>, i: uint, collide_radius: f32) -> Vec3<f32> {
    let mut keep_away: Vec3<f32> = na::zero();
    let mut neighbors  = 0u;
    for j in range(0, ps.len()) {
        if i != j {
            let disp = ps[j].pos - ps[i].pos;
            let dist = na::norm(&disp);
            if dist < collide_radius {
                let disp_norm_weight = disp / (dist * dist);
                keep_away = keep_away - disp_norm_weight;
                neighbors += 1;
            }
        }
    }
    if neighbors > 0 {
        keep_away = na::normalize(&(keep_away / neighbors as f32));
    }
    keep_away

}

fn match_speed_v(ps: &Vec<Plane>, i: uint, look_radius: f32) -> Vec3<f32> {
    let mut center_count = 0u;

    let perceived_velocity = ps.iter()
        .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
        .fold(na::zero::<Vec3<f32>>(), |a, ref p| { center_count += 1; a + p.vel });
    center_count -= 1;

    if center_count == 0 {
        na::zero()
    } else {
        na::normalize(&(perceived_velocity / center_count as f32))
    }
}

fn bounds_v(ps: &Vec<Plane>, i: uint) -> Vec3<f32> {
    let mut bounds: Vec3<f32> = na::zero();
    let p = &ps[i];

    bounds.x =
        if p.pos.x > 50.0 {
            -1.0
        } else if p.pos.x < -50.0 {
            1.0
        } else {
            0.0
        };

    bounds.y =
        if p.pos.y > 20.0 {
            -1.0
        } else if p.pos.y < 5.0 {
            1.0
        } else {
            0.0
        };

    bounds.z =
        if p.pos.z > 50.0 {
            -1.0
        } else if p.pos.z < -50.0 {
            1.0
        } else {
            0.0
        };

    if bounds != na::zero() {
        na::normalize(&bounds)
    } else {
        bounds
    }
}

fn main() {
    let world_scale = 0.2;
    // have camera start from higher position
    let eye = Vec3::new(50.0, 50.0, 100.0);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);

    let mut window = Window::new("Kiss3d: cube");
    window.set_framerate_limit(Some(60));
    window.set_light(light::StickToCamera);

    let mut ground = window.add_quad(100.0, 100.0, 1, 1);
    ground.set_local_rotation(Vec3::new((90.0f32).to_radians(), 0.0, 0.0));
    ground.set_color(0.1, 0.1, 0.1);

    let pmesh = Plane::gen_mesh();

    let num_planes = 500;

    let mut ps = Vec::new();
    for i in range(0i, num_planes) {
        ps.push(Plane::new(window.add_mesh(pmesh.clone(), Vec3::new(world_scale, world_scale, world_scale))));
    }

    let w1 = 8.0;
    let w2 = 12.0;
    let w3 = 8.0;
    let w4 = 20.0;

    let debug = false;

    let look_radius = 20.0 * world_scale;
    let collide_radius = 10.0 * world_scale; // TODO: figure out collide radius

    let mut last_time = window.context().get_time() as f32;
    let mut curr_time;
    while window.render_with_camera(&mut arc_ball) {
        curr_time = window.context().get_time() as f32;

        draw_axis(&mut window);

        //let flock_total_pos = ps.iter().fold(na::zero::<Vec3<f32>>(), |a, ref p| a + p.pos);

        for i in range(0, ps.len()) {
            let mut r1_scaled = flock_center_v(&ps, i, look_radius);
            r1_scaled.x *= w1;
            r1_scaled.y *= w1;
            r1_scaled.z *= w1;
            if debug { window.draw_line(&ps[i].pos, &(ps[i].pos + r1_scaled), &Vec3::new(1.0, 0.0, 0.0)); }

            let mut r2_scaled = keep_away_v(&ps, i, collide_radius);
            r2_scaled.x *= w2;
            r2_scaled.y *= w2;
            r2_scaled.z *= w2;
            if debug { window.draw_line(&ps[i].pos, &(ps[i].pos + r2_scaled), &Vec3::new(0.0, 1.0, 0.0)); }

            let mut r3_scaled = match_speed_v(&ps, i, look_radius);
            r3_scaled.x *= w3;
            r3_scaled.y *= w3;
            r3_scaled.z *= w3;
            if debug { window.draw_line(&ps[i].pos, &(ps[i].pos + r3_scaled), &Vec3::new(0.0, 1.0, 1.0)); }

            let mut r4_scaled = bounds_v(&ps, i);
            r4_scaled.x *= w4;
            r4_scaled.y *= w4;
            r4_scaled.z *= w4;
            if debug { window.draw_line(&ps[i].pos, &(ps[i].pos + r4_scaled), &Vec3::new(1.0, 1.0, 0.0)); }

            ps.get_mut(i).acc = r1_scaled + r2_scaled + r3_scaled + r4_scaled;
        }

        for p in ps.mut_iter() {
            p.update(curr_time - last_time);
        }

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
