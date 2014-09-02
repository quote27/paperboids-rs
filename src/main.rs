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
            acc: na::zero(),
            node: node,
        }
    }

    fn update(&mut self, dt: f32) {
        let world_scale = 0.5; // TODO: figure out where to put these constants
        let max_speed = 20.0 * world_scale;
        let min_speed = 2.0 * world_scale;

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
    let mut neighbors = 0u;
    let perceived_velocity = ps.iter()
        .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
        .fold(na::zero::<Vec3<f32>>(), |a, ref p| { neighbors += 1; a + p.vel });
    neighbors -= 1; // this is because the bird itself is caught in the filter

    if neighbors == 0 {
        na::zero()
    } else {
        na::normalize(&(perceived_velocity / neighbors as f32))
    }
}

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
        bounds
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
    let collide_radius = 10.0 * world_scale; // TODO: figure out a good collide radius

    let look_radius2 = look_radius * look_radius; // can avoid squareroot for dist calculations
    let collide_radius2 = collide_radius * collide_radius;

    let eye = Vec3::new(world_dim.x/2.0, world_dim.y/2.0, world_dim.z);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);

    let weights: [f32, ..5] = [
        0.0,  // flock centering
        12.0, // collision avoidance
        8.0,  // match velocity
        20.0, // bounds push
        20.0, // avoid sphere
    ];

    let num_planes = 500;

    // TODO: make ground cooler - random heightmap?
    let mut ground = window.add_quad(world_dim.x, world_dim.z, 1, 1);
    ground.set_local_rotation(Vec3::new(std::num::Float::frac_pi_2(), 0.0, 0.0));
    ground.set_color(0.1, 0.1, 0.1);

    // TODO: obstacle meshes
    let sph_radius = 2.0;
    let mut sph1 = window.add_sphere(sph_radius);
    let sph1_pos = Vec3::new(0.0, 15.0, 0.0);
    sph1.set_local_translation(sph1_pos);
    sph1.set_color(1.0, 0.0, 0.0);
    sph1.set_points_size(1.0); //wireframe mode for plane
    sph1.set_lines_width(1.0);
    sph1.set_surface_rendering_activation(false);

    let mut sph2 = window.add_sphere(sph_radius);
    let sph2_pos = Vec3::new(30.0, 10.0, 0.0);
    sph2.set_local_translation(sph2_pos);
    sph2.set_color(1.0, 0.0, 0.0);
    sph2.set_points_size(1.0); //wireframe mode for plane
    sph2.set_lines_width(1.0);
    sph2.set_surface_rendering_activation(false);

    let mut cyl1 = window.add_cylinder(2.0, 40.0);
    let cyl1_pos = Vec3::new(-20.0, 20.0, -20.0);
    cyl1.set_local_translation(cyl1_pos);
    cyl1.set_color(1.0, 0.0, 0.0);
    cyl1.set_points_size(1.0); //wireframe mode for plane
    cyl1.set_lines_width(1.0);
    cyl1.set_surface_rendering_activation(false);

    let mut cyl2 = window.add_cylinder(2.0, 40.0);
    let cyl2_pos = Vec3::new(-20.0, 20.0, 45.0);
    cyl2.set_local_translation(cyl2_pos);
    cyl2.set_color(1.0, 0.0, 0.0);
    cyl2.set_points_size(1.0); //wireframe mode for plane
    cyl2.set_lines_width(1.0);
    cyl2.set_surface_rendering_activation(false);


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
            let mut neighbors = 0u;
            let mut colliders = 0u;

            // rules - overloaded vecter for rules to accumulate and average:
            // 0. center
            // 1. keep away
            // 2. follow
            // 3. bounds
            // 4. avoid obstacles
            let mut rules: [Vec3<f32>, ..5] = [na::zero(), na::zero(), na::zero(), na::zero(), na::zero()];

            {
                let p = &ps[i];

                for j in range(0, ps.len()) {
                    if i == j { continue; }

                    let o = &ps[j];

                    let disp = o.pos - p.pos;
                    let dist2 = na::sqnorm(&disp);

                    if dist2 < look_radius2 {
                        neighbors += 1;

                        // rule 0 - fly to center
                        rules[0] = rules[0] + o.pos;

                        // rule 2 - fly in the same direction
                        rules[2] = rules[2] + o.vel;

                        // rule 1 - avoid others
                        if dist2 < collide_radius2 {
                            colliders += 1;
                            rules[1] = rules[1] - (disp / dist2);
                        }
                    }
                }

                if neighbors > 0 {
                    rules[0] = na::normalize(&(rules[0] / neighbors as f32 - p.pos)) * weights[0];
                    rules[2] = na::normalize(&(rules[2] / neighbors as f32)) * weights[2];
                }
                if colliders > 0 {
                    rules[1] = na::normalize(&(rules[1] / colliders as f32)) * weights[1];
                }

                rules[3] = bounds_v(p) * weights[3];

                rules[4] =
                    avoid_spheres_v(p, collide_radius2, &sph1_pos, sph_radius) +
                    avoid_spheres_v(p, collide_radius2, &sph2_pos, sph_radius) +
                    avoid_cylinder_v(p, collide_radius2, &cyl1_pos, 40.0, 2.0) +
                    avoid_cylinder_v(p, collide_radius2, &cyl2_pos, 40.0, 2.0);
                if rules[4] != na::zero() {
                    rules[4] = na::normalize(&rules[4]) * weights[4];
                }

                if debug {
                    window.draw_line(&p.pos, &(p.pos + rules[0]), &Vec3::new(1.0, 0.0, 0.0));
                    window.draw_line(&p.pos, &(p.pos + rules[1]), &Vec3::new(0.0, 1.0, 0.0));
                    window.draw_line(&p.pos, &(p.pos + rules[2]), &Vec3::new(0.0, 1.0, 1.0));
                    window.draw_line(&p.pos, &(p.pos + rules[3]), &Vec3::new(1.0, 1.0, 0.0));
                    window.draw_line(&p.pos, &(p.pos + rules[4]), &Vec3::new(1.0, 0.0, 1.0));
                }
            }

            let mut mag = 0.0; // magnitude
            let max_mag2 = 100.0 * 100.0f32;

            ps.get_mut(i).acc = na::zero();

            for r in range(0, 5) {
                let m = na::sqnorm(&rules[r]);

                // if mag + m > max_mag2 {
                //     break;
                // }

                mag += m;
                ps.get_mut(i).acc = ps.get_mut(i).acc + rules[r];
            }
            //rules[0] + rules[1] + rules[2] + rules[3] + rules[4];


//            t_tmp = window.context().get_time();
//            let r1_scaled = flock_center_v(&ps, i, look_radius) * weights[0];
//            times[0] += window.context().get_time() - t_tmp;
//
//            t_tmp = window.context().get_time();
//            let r2_scaled = keep_away_v(&ps, i, collide_radius) * weights[1];
//            times[1] += window.context().get_time() - t_tmp;
//
//            t_tmp = window.context().get_time();
//            let r3_scaled = match_speed_v(&ps, i, look_radius) * weights[2];
//            times[2] += window.context().get_time() - t_tmp;
//
//            t_tmp = window.context().get_time();
//            let r4_scaled = bounds_v(&ps[i]) * weights[3];
//            times[3] += window.context().get_time() - t_tmp;
//
//            t_tmp = window.context().get_time();
//            let mut r5_scaled = avoid_spheres_v(&ps, i, collide_radius, &sph1_pos, sph_radius) * weights[4];
//            r5_scaled = r5_scaled + avoid_spheres_v(&ps, i, collide_radius, &sph2_pos, sph_radius) * weights[4];
//            r5_scaled = r5_scaled + avoid_cylinder_v(&ps, i, collide_radius, &cyl1_pos, 40.0, 2.0) * weights[4];
//            r5_scaled = r5_scaled + avoid_cylinder_v(&ps, i, collide_radius, &cyl2_pos, 40.0, 2.0) * weights[4];
//            times[4] += window.context().get_time() - t_tmp;
//
//            if debug {
//                window.draw_line(&ps[i].pos, &(ps[i].pos + r1_scaled), &Vec3::new(1.0, 0.0, 0.0));
//                window.draw_line(&ps[i].pos, &(ps[i].pos + r2_scaled), &Vec3::new(0.0, 1.0, 0.0));
//                window.draw_line(&ps[i].pos, &(ps[i].pos + r3_scaled), &Vec3::new(0.0, 1.0, 1.0));
//                window.draw_line(&ps[i].pos, &(ps[i].pos + r4_scaled), &Vec3::new(1.0, 1.0, 0.0));
//            }
//
//            ps.get_mut(i).acc = r1_scaled + r2_scaled + r3_scaled + r4_scaled + r5_scaled;
        }

        t_tmp = window.context().get_time();
        let dt  = (curr_time - last_time) as f32;
        for p in ps.mut_iter() {
            p.update(dt);
        }
        times[5] = window.context().get_time() - t_tmp;

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
