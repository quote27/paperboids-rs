extern crate native;
extern crate time;
extern crate kiss3d;
extern crate nalgebra;
extern crate debug; //to print out type of variable at runtime

use std::rc::Rc;
use std::cell::RefCell;
use std::rand;
use std::sync::{Arc, Barrier, Future, RWLock};
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

#[deriving(Send,Sync,Show)]
struct Plane {
    pos: Vec3<f32>,
    vel: Vec3<f32>,
    acc: Vec3<f32>,
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

    fn new(bbox: &AABB) -> Plane {
        let x = bbox.l.x + rand::random::<f32>() * bbox.xlen();
        let y = bbox.l.y + rand::random::<f32>() * bbox.ylen();
        let z = bbox.l.z + rand::random::<f32>() * bbox.zlen();

        let vx = 10.0 - rand::random::<f32>() * 20.0;
        let vy = 10.0 - rand::random::<f32>() * 20.0;
        let vz = 10.0 - rand::random::<f32>() * 20.0;

        Plane {
            pos: Vec3::new(x, y, z),
            vel: Vec3::new(vx, vy, vz),
            acc: na::zero(),
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
    }

    fn update_node(&mut self, node: &mut SceneNode) {
        // TODO: figure out up vector for banking
        node.look_at_z(&self.pos, &(self.pos + self.vel), &Vec3::y()); 
    }
} 

//    let perceived_center = ps.iter()
//        .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
//        .fold(na::zero::<Vec3<f32>>(), |a, ref p| { neighbors += 1; a + p.pos });

fn bounds_v(p: &Plane, bbox: &AABB) -> Vec3<f32> {
    let mut bounds: Vec3<f32> = na::zero();

    bounds.x =
        if p.pos.x > bbox.h.x {
            -1.0
        } else if p.pos.x < bbox.l.x {
            1.0
        } else {
            0.0
        };

    bounds.y =
        if p.pos.y > bbox.h.y {
            -1.0
        } else if p.pos.y < bbox.l.y {
            1.0
        } else {
            0.0
        };

    bounds.z =
        if p.pos.z > bbox.h.z {
            -1.0
        } else if p.pos.z < bbox.l.z {
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
    let show_z_order = false;
    let world_box = AABB::new(na::zero(), Vec3::new(100.0f32, 40.0, 100.0));
    let world_scale = 0.2;
    let look_radius = 15.0 * world_scale;
    let collide_radius = 8.0 * world_scale; // TODO: figure out a good collide radius

    let look_radius2 = look_radius * look_radius; // can avoid squareroot for dist calculations
    let collide_radius2 = collide_radius * collide_radius;

    // camera setup
    let eye = Vec3::new(0.0, world_box.h.y / 2.0, -world_box.h.z);
    let at = Vec3::new(world_box.h.x / 2.0, 0.0, world_box.h.z / 2.0);
    let mut arc_ball = ArcBall::new(eye, at);

    let weights: [f32, ..5] = [
        30.0, // avoid obstacles
        12.0, // collision avoidance
        8.0,  // flock centering
        8.0,  // match velocity
        20.0, // bounds push
    ];

    let num_planes = 5000u;

    // TODO: make ground cooler - random heightmap?
    let mut ground = window.add_quad(world_box.xlen(), world_box.zlen(), 1, 1);
    ground.set_local_rotation(Vec3::new(std::num::Float::frac_pi_2(), 0.0, 0.0));
    ground.set_local_translation(Vec3::new(world_box.xlen() / 2.0, 0.0, world_box.zlen() / 2.0));
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

    // shrink the world bouonding box to prevent birds from spawning / moving outside the world
    let mut fly_bbox = world_box.clone();
    fly_bbox.scale_center(0.8);
    let fly_bbox = fly_bbox;

    let pmesh = Plane::gen_mesh();
    let mut ps = Vec::with_capacity(num_planes);
    let mut pnodes = Vec::with_capacity(num_planes);
    for _ in range(0, num_planes) {
        ps.push(Plane::new(&fly_bbox));

        let mut node = window.add_mesh(pmesh.clone(), Vec3::new(world_scale, world_scale, world_scale));
        node.set_color(1.0, 1.0, 1.0);
        node.enable_backface_culling(false);
        enable_wireframe(&mut node);
        if debug { // cylindar to represent look radius
            let mut cyl = node.add_cylinder(look_radius, 0.1);
            cyl.set_local_translation(Vec3::new(0.0, -0.1, 0.0));
            cyl.set_color(0.2, 0.2, 0.2);
            enable_wireframe(&mut cyl);
        }

        pnodes.push(node);
    }

    let shared_ps = Arc::new(RWLock::new(ps));

    let threads = 4u;
    let work_size = num_planes / threads;

    let mut last_time = time::precise_time_ns();
    let mut curr_time;

    let mut frame_count = 0u;
    let mut frame_times: [f64, ..7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // spot 0 is total
    let mut thread_times: Vec<f64> = Vec::with_capacity(threads);
    for i in range(0, threads) {
        thread_times.push(0.0);
    }

    while window.render_with_camera(&mut arc_ball) {
        let mut frame_t = Timer::new();
        frame_t.start();
        curr_time = time::precise_time_ns();

        let mut sorted; // (plane id, morton value)
        {
            let lock = shared_ps.read();
            let ps = &*lock;
            sorted = z_order_planes(&*lock);

            if show_z_order {
                for i in range(0, sorted.len()-1) {
                    let (id1, _) = sorted[i];
                    let (id2, _) = sorted[i+1];
                    let p1 = ps[id1];
                    let p2 = ps[id2];

                    window.draw_line(&p1.pos, &p2.pos, &Vec3::new(0.0, 1.0, 1.0));
                }
            }
        }
        let sorted = sorted;
        frame_times[6] += frame_t.stop();

        //println!("sort: {}", frame_t.stop());

        let (tx, rx) = channel();

        for tid in range(0, threads) {
            //let b = barrier.clone();
            let child_ps = shared_ps.clone();
            let tx = tx.clone();

            spawn(proc() {
                let mut thread_t = Timer::new();
                thread_t.start();
                let lock_ps = child_ps.read();
                let ps = lock_ps.as_slice();

                let start_id = tid * work_size;
                let work_size =
                    if tid == threads - 1 {
                        work_size + num_planes % threads
                    } else {
                        work_size
                    };

                let mut acc_list = Vec::with_capacity(work_size);

                for i in range(start_id, start_id + work_size) {
                    let mut rules: [Vec3<f32>, ..5] = [na::zero(), na::zero(), na::zero(), na::zero(), na::zero()];
                    let mut neighbors = 0u;
                    let mut colliders = 0u;

                    {
                        let p = &ps[i];

                        for j in range(0, num_planes) {
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

                        rules[4] = bounds_v(p, &fly_bbox) * weights[4];

                        //rules[0] =
                        //    avoid_spheres_v(p, collide_radius2, &sph1_pos, sph_radius) +
                        //    avoid_cylinder_v(p, collide_radius2, &cyl1_pos, cyl_height, cyl_radius);
                        //if rules[0] != na::zero() {
                        //    rules[0] = na::normalize(&rules[0]) * weights[0];
                        //}

                        // TODO: figure out debug when in thread mode
                        //if debug {
                        //    window.draw_line(&p.pos, &(p.pos + rules[0]), &Vec3::new(1.0, 0.0, 0.0));
                        //    window.draw_line(&p.pos, &(p.pos + rules[1]), &Vec3::new(0.0, 1.0, 0.0));
                        //    window.draw_line(&p.pos, &(p.pos + rules[2]), &Vec3::new(0.0, 1.0, 1.0));
                        //    window.draw_line(&p.pos, &(p.pos + rules[3]), &Vec3::new(1.0, 1.0, 0.0));
                        //    window.draw_line(&p.pos, &(p.pos + rules[4]), &Vec3::new(1.0, 0.0, 1.0));
                        //}
                    }

                    let mut mag = 0.0; // magnitude
                    let max_mag = 50.0;

                    let mut acc: Vec3<f32> = na::zero();

                    for r in range(0, 5) {
                        // TODO: minor optimization? use non-sqrt norm
                        let m = na::norm(&rules[r]);

                        if m == 0.0 { continue; }

                        if mag + m > max_mag {
                            // rebalance last rule
                            rules[r] = rules[r] * ((max_mag - mag) / m);
                            acc = acc + rules[r];
                            break;
                        }

                        mag += m;
                        acc = acc + rules[r];
                    }

                    acc_list.push(acc);
                    //unsafe {
                    //    ps.get_mut(i).unwrap().acc = acc;
                    //}
                }

                //b.wait();
                tx.send((tid, thread_t.stop(), acc_list));
            });
        }
        frame_times[1] = frame_t.stop();

        let dt  = (curr_time - last_time) as f32 / 1e9; // in seconds

        let mut update_birds_t = Timer::new();
        let mut update_birds_inner_t = Timer::new();
        let mut update_birds_inner2_t = Timer::new();
        update_birds_t.start();
        for _ in range(0, threads) {
            let (tid, thread_time, acc_list) = rx.recv();
            let start_id = tid * work_size;
            *thread_times.get_mut(tid) = thread_times[tid] + thread_time;

            update_birds_inner_t.start();
            for i in range(start_id, start_id + acc_list.len()) {
                let mut ps = shared_ps.write();
                {
                    update_birds_inner2_t.start();
                    let p = ps.get_mut(i);
                    p.acc = acc_list[i - start_id];
                    p.update(dt, world_scale);
                    p.update_node(pnodes.get_mut(i));
                    frame_times[5] += update_birds_inner2_t.stop();
                }
            }
            frame_times[4] += update_birds_inner_t.stop();
        }
        frame_times[3] += update_birds_t.stop();
        frame_times[2] += frame_t.stop();

        if debug {
            let ps = shared_ps.read();
            for i in range(0, num_planes) {
                let p = (*ps)[i];
                println!("{}: pos: {}, vel: {}", i, p.pos, p.vel);
                if p.pos == na::zero() {
                    println!("zero position vector: id: {}", i);
                }
                if !p.pos.x.is_finite() || !p.pos.y.is_finite() || !p.pos.z.is_finite() {
                    println!("non finite position vector: id: {}, vec: {}", i, (p.pos.x, p.pos.y, p.pos.z));
                }
            }
        }

        if follow_first_bird {
            let ps = shared_ps.read();
            let p = (*ps)[0];
            arc_ball.look_at_z(p.pos, p.pos + p.vel);
        }

        //draw_axis(&mut window);

        frame_times[0] += frame_t.stop();
        //println!("frame: {}, sub: {} {} {} {} {}, update: {}", f_end - f_start, times[0], times[1], times[2], times[3], times[4], times[5]);
        frame_count += 1;
        if frame_count % 60 == 0 {
            for i in range(0, 7) {
                frame_times[i] /= frame_count as f64;
            }
            for t in thread_times.mut_iter() {
                *t = *t / frame_count as f64;
            }
            println!("avg last 60 frames: {:.2} - steps: {:.2} {:.2} - breakdown: {:.2} {:.2} {:.2} {:.2}", frame_times[0], frame_times[1], frame_times[2], frame_times[3], frame_times[4], frame_times[5], frame_times[6]);
            print!("threads: ");
            for t in thread_times.mut_iter() {
                print!("{} ", *t);
                *t = 0.0;
            }
            println!("");
            frame_count = 0;
            for i in range(0, 7) {
                frame_times[i] = 0.0;
            }
        }
        last_time = curr_time;
    }

    // {
    //     let ps = shared_ps.read();
    //     for p in ps.iter() {
    //         println!("{}", p);
    //     }
    // }
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

struct Timer { s: u64, e: u64, }
impl Timer {
    fn new() -> Timer {
        Timer { s: 0, e: 0 }
    }

    #[inline(always)]
    fn start(&mut self) {
        self.s = time::precise_time_ns();
    }

    #[inline(always)]
    fn stop(&mut self) -> f64 {
        self.e = time::precise_time_ns();
        self.elapsedms()
    }

    #[inline(always)]
    fn elapsedms(&self) -> f64 {
        (self.e - self.s) as f64 / 1e6 //nanoseconds -> ms
    }
}

#[deriving(Clone)]
struct AABB { l: Vec3<f32>, h: Vec3<f32> }
impl AABB {
    fn new(low: Vec3<f32>, high: Vec3<f32>) -> AABB {
        AABB { l: low, h: high, }
    }

    #[inline(always)]
    fn xlen(&self) -> f32 { self.h.x - self.l.x }

    #[inline(always)]
    fn ylen(&self) -> f32 { self.h.y - self.l.y }

    #[inline(always)]
    fn zlen(&self) -> f32 { self.h.z - self.l.z }

    fn center(&self) -> Vec3<f32> {
        self.l + (self.h - self.l) / 2.0f32
    }

    // scales but pins to lower corner
    fn scale(&mut self, scale: f32) {
        self.h.x = self.l.x + self.xlen() * scale;
        self.h.y = self.l.y + self.ylen() * scale;
        self.h.z = self.l.z + self.zlen() * scale;
    }

    fn scale_center(&mut self, scale: f32) {
        let xl = self.xlen();
        let yl = self.ylen();
        let zl = self.zlen();

        let diffv = Vec3::new(xl - xl * scale, yl - yl * scale, zl - zl * scale) * 0.5f32;

        self.scale(scale);
        self.trans(&diffv);
    }

    fn trans(&mut self, trans: &Vec3<f32>) {
        self.h = self.h + *trans;
        self.l = self.l + *trans;
    }
}

#[inline(always)]
fn min(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

#[inline(always)]
fn max(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

// logic from http://devblogs.nvidia.com/parallelforall/thinking-parallel-part-iii-tree-construction-gpu/
fn morton_3d(p: &Vec3<f32>) -> u32 {
    let x = min(max(p.x * 10.0, 0.0), 1023.0f32);
    let y = min(max(p.y * 10.0, 0.0), 1023.0f32);
    let z = min(max(p.z * 10.0, 0.0), 1023.0f32);

    let xx = expand_bits(x as u32);
    let yy = expand_bits(y as u32);
    let zz = expand_bits(z as u32);

    xx * 4 + yy * 2 + zz
}

fn expand_bits(u: u32) -> u32 {
    let mut u = u;
    u = (u * 0x00010001u32) & 0xFF0000FFu32;
    u = (u * 0x00000101u32) & 0x0F00F00Fu32;
    u = (u * 0x00000011u32) & 0xC30C30C3u32;
    u = (u * 0x00000005u32) & 0x49249249u32;
    u
}

fn z_order_planes(ps: &Vec<Plane>) -> Vec<(uint, u32)> {
    let mut zord_id: Vec<(uint, u32)> = Vec::with_capacity(ps.len());

    for i in range(0, ps.len()) {
        zord_id.push((i, morton_3d(&ps[i].pos)));
    }

    zord_id.sort_by(|&(_, am), &(_, bm)| am.cmp(&bm));

    zord_id
}
