extern crate native;
extern crate kiss3d;
extern crate nalgebra;
extern crate debug; //to print out type of variable at runtime

use std::rc::Rc;
use std::cell::RefCell;
use std::rand;
use nalgebra::na::{Vec2, Vec3};
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

// TODO: check out recorder demo to create mpg's of simulations

// TODO: opt; instead of filtering at every rule, do one filter pass and call each of the rules

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
        enable_wireframe(&mut node);

        let x = 25.0 - rand::random::<f32>() * 50.0;
        let y = 25.0 - rand::random::<f32>() * 20.0;
        let z = 25.0 - rand::random::<f32>() * 50.0;

        let vx = 10.0 - rand::random::<f32>() * 20.0;
        let vy = 10.0 - rand::random::<f32>() * 20.0;
        let vz = 10.0 - rand::random::<f32>() * 20.0;

        Plane {
            pos: Vec3::new(x, y, z),
            vel: Vec3::new(vx, vy, vz),
            acc: na::zero(),
            node: node,
        }
    }

    fn update(&mut self, dt: f32, world_scale: f32) {
        // TODO: figure out where to put these constants
        let max_speed = 25.0 * world_scale;
        let min_speed = 4.0 * world_scale;

        self.vel = self.vel + self.acc * dt;
        let curr_speed = na::norm(&self.vel);
        if curr_speed > max_speed {
            self.vel = self.vel / curr_speed * max_speed;
        } else if curr_speed < min_speed {
            self.vel = self.vel / curr_speed * min_speed;
        }

        self.pos = self.pos + self.vel * dt;
        self.node.look_at_z(&self.pos, &(self.pos + self.vel), &Vec3::y()); // TODO: figure out up vector for banking
    }
} 

//    let perceived_center = ps.iter()
//        .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
//        .fold(na::zero::<Vec3<f32>>(), |a, ref p| { neighbors += 1; a + p.pos });

fn bounds_v(p: &Plane) -> Vec3<f32> {
    let mut bounds: Vec3<f32> = na::zero();

    bounds.x =
        if p.pos.x > 50.0 {
            -1.0
        } else if p.pos.x < -50.0 {
            1.0
        } else {
            0.0
        };

    bounds.y =
        if p.pos.y > 40.0 {
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
        bounds // a zero vector
    }
}

fn avoid_spheres_v(p: &Plane, collide_radius2: f32, sphere_pos: &Vec3<f32>, sphere_r: f32) -> Vec3<f32> {
    let disp = *sphere_pos - p.pos;
    let dist2 = na::sqnorm(&disp) - sphere_r * sphere_r; //subtract radius of sphere
    if dist2 < collide_radius2 {
        -disp / dist2
    } else {
        na::zero()
    }
}

fn avoid_cylinder_v(p: &Plane, collide_radius2: f32, cyl_pos: &Vec3<f32>, cyl_h: f32, cyl_r: f32) -> Vec3<f32> {
    let hh = cyl_h / 2.0;
    if p.pos.y < cyl_pos.y + hh && p.pos.y > cyl_pos.y - hh {
        let p2d = Vec2::new(p.pos.x, p.pos.z);
        let c2d = Vec2::new(cyl_pos.x, cyl_pos.z);

        let disp = c2d - p2d;
        let dist2 = na::sqnorm(&disp) - cyl_r; //subtract radius of sphere
        if dist2 < collide_radius2 {
            let v = -disp / dist2;
            Vec3::new(v.x, 0.0, v.y)
        } else {
            na::zero()
        }
    } else {
        na::zero()
    }
}


fn main() {
    let mut window = Window::new("Kiss3d: cube");
    window.set_framerate_limit(Some(60));
    window.set_light(light::StickToCamera);

    let debug = false;
    let follow_first_bird = false;
    let world_dim = Vec3::new(100.0f32, 100.0, 100.0);
    let world_scale = 0.5;
    let look_radius = 20.0 * world_scale;
    let collide_radius = 8.0 * world_scale; // TODO: figure out a good collide radius

    let look_radius2 = look_radius * look_radius; // can avoid squareroot for dist calculations
    let collide_radius2 = collide_radius * collide_radius;

    let eye = Vec3::new(world_dim.x/2.0, world_dim.y/2.0, world_dim.z);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);

    let weights: [f32, ..5] = [
        30.0, // avoid obstacles
        12.0, // collision avoidance
        8.0,  // flock centering
        8.0,  // match velocity
        20.0, // bounds push
    ];

    let num_planes = 500;

    // TODO: make ground cooler - random heightmap?
    let mut ground = window.add_quad(world_dim.x, world_dim.z, 1, 1);
    ground.set_local_rotation(Vec3::new(std::num::Float::frac_pi_2(), 0.0, 0.0));
    ground.set_color(0.1, 0.1, 0.1);

    // TODO: obstacle meshes
//    let sph_radius = 2.0 * world_scale;
//    let mut sph1 = window.add_sphere(sph_radius);
//    let sph1_pos = Vec3::new(0.0, 15.0, 0.0);
//    sph1.set_local_translation(sph1_pos);
//    sph1.set_color(1.0, 0.0, 0.0);
//    enable_wireframe(&mut sph1);
//
//    let cyl_radius = 2.0 * world_scale;
//    let cyl_height = 40.0;
//    let mut cyl1 = window.add_cylinder(cyl_radius, cyl_height);
//    let cyl1_pos = Vec3::new(-20.0, cyl_height / 2.0, -20.0);
//    cyl1.set_local_translation(cyl1_pos);
//    cyl1.set_color(1.0, 0.0, 0.0);
//    enable_wireframe(&mut cyl1);

    let pmesh = Plane::gen_mesh();
    let mut ps = Vec::new();
    for i in range(0i, num_planes) {
        ps.push(Plane::new(window.add_mesh(pmesh.clone(), Vec3::new(world_scale, world_scale, world_scale))));
    }

    let mut last_time = window.context().get_time();
    let mut curr_time;
    let mut times: [f64, ..6];
    let mut t_tmp;
    while window.render_with_camera(&mut arc_ball) {
        let f_start = window.context().get_time();
        curr_time = window.context().get_time();

        //let flock_total_pos = ps.iter().fold(na::zero::<Vec3<f32>>(), |a, ref p| a + p.pos);
        times = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        for i in range(0, ps.len()) {
            t_tmp = window.context().get_time();
            // rules - overloaded vecter for rules to accumulate and average:
            // 4. avoid obstacles
            // 1. keep away
            // 0. center
            // 2. follow
            // 3. bounds
            let mut rules: [Vec3<f32>, ..5] = [na::zero(), na::zero(), na::zero(), na::zero(), na::zero()];
            let mut neighbors = 0u;
            let mut colliders = 0u;

            {
                let p = &ps[i];

                for j in range(0, ps.len()) {
                    if i == j { continue; }

                    let o = &ps[j];

                    let disp = o.pos - p.pos;
                    let dist2 = na::sqnorm(&disp);

                    if dist2 < look_radius2 {
                        neighbors += 1;

                        // fly to center
                        rules[2] = rules[2] + o.pos;

                        // fly in the same direction
                        rules[3] = rules[3] + o.vel;

                        // avoid others
                        if dist2 < collide_radius2 {
                            colliders += 1;
                            rules[1] = rules[1] - (disp / dist2);
                        }
                    }
                }

                if neighbors > 0 {
                    rules[2] = na::normalize(&(rules[2] / neighbors as f32 - p.pos)) * weights[2];
                    rules[3] = na::normalize(&(rules[3] / neighbors as f32)) * weights[3];
                }
                if colliders > 0 {
                    rules[1] = na::normalize(&(rules[1] / colliders as f32)) * weights[1];
                }

                rules[4] = bounds_v(p) * weights[4];

//                rules[0] =
//                    avoid_spheres_v(p, collide_radius2, &sph1_pos, sph_radius) +
//                    avoid_cylinder_v(p, collide_radius2, &cyl1_pos, cyl_height, cyl_radius) +
//                if rules[0] != na::zero() {
//                    rules[0] = na::normalize(&rules[0]) * weights[0];
//                }

                if debug {
                    window.draw_line(&p.pos, &(p.pos + rules[0]), &Vec3::new(1.0, 0.0, 0.0));
                    window.draw_line(&p.pos, &(p.pos + rules[1]), &Vec3::new(0.0, 1.0, 0.0));
                    window.draw_line(&p.pos, &(p.pos + rules[2]), &Vec3::new(0.0, 1.0, 1.0));
                    window.draw_line(&p.pos, &(p.pos + rules[3]), &Vec3::new(1.0, 1.0, 0.0));
                    window.draw_line(&p.pos, &(p.pos + rules[4]), &Vec3::new(1.0, 0.0, 1.0));
                }
            }
            times[0] += window.context().get_time() - t_tmp;


            let mut mag = 0.0; // magnitude
            let max_mag = 50.0;

            ps.get_mut(i).acc = na::zero();

            for r in range(0, 5) {
                // TODO: minor optimization? use non-sqrt norm
                let m = na::norm(&rules[r]);

                if m == 0.0 { continue; }

                if mag + m > max_mag {
                    // rebalance last rule
                    rules[r] = rules[r] * ((max_mag - mag) / m);
                    ps.get_mut(i).acc = ps.get_mut(i).acc + rules[r];
                    break;
                }

                mag += m;
                ps.get_mut(i).acc = ps.get_mut(i).acc + rules[r];
            }
            times[1] += window.context().get_time() - t_tmp;
        }

        t_tmp = window.context().get_time();
        let dt  = (curr_time - last_time) as f32;
        for p in ps.mut_iter() {
            p.update(dt, world_scale);
        }
        times[2] = window.context().get_time() - t_tmp;

        if follow_first_bird {
            arc_ball.look_at_z(ps[0].pos, ps[0].pos + ps[0].vel);
        }

        draw_axis(&mut window);

        let f_end = window.context().get_time();
        println!("frame: {}, sub: {} {} {} {} {}, update: {}", f_end - f_start, times[0], times[1], times[2], times[3], times[4], times[5]);
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

fn enable_wireframe(n: &mut SceneNode) {
    n.set_points_size(1.0); //wireframe mode for plane
    n.set_lines_width(1.0);
    n.set_surface_rendering_activation(false);
}
