extern crate clock_ticks;
extern crate native;
extern crate kiss3d;
extern crate cgmath;
extern crate getopts;

use std::rc::Rc;
use std::cell::RefCell;
use std::rand;
use std::sync::{Arc, RWLock};
use std::num::Float;
use std::fmt::{Show, Formatter, FormatError};
use getopts::{optopt, optflag, getopts, Matches};
use nalgebra::{Vec2, Vec3, Pnt3};
use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use kiss3d::light;

use utils::{Timer, TimeMap, AABB, min, max};
use octree::{Octree, Octnode, OctnodeId};

mod octree;
mod utils;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

// reference: http://www.vergenet.net/~conrad/boids/pseudocode.html

// TODO: check out recorder demo to create mpg's of simulations

// TODO: opt; instead of filtering at every rule, do one filter pass and call each of the rules

// TODO: note: don't have to divide a vector by number of elements if its just going to be normalized in the end [ex: collision]

struct BoidId(uint);

impl Show for BoidId {
    fn fmt(&self, f: &mut Formatter) -> Result<(), FormatError> {
        let BoidId(id) = *self;
        f.write(format!("{}", id).as_bytes())
    }
}

#[deriving(Send,Sync,Show)]
struct Boid {
    pos: Vec3<f32>,
    vel: Vec3<f32>,
    acc: Vec3<f32>,
}

impl Boid {
    fn new(bbox: &AABB) -> Boid {
        let x = bbox.l.x + rand::random::<f32>() * bbox.xlen();
        let y = bbox.l.y + rand::random::<f32>() * bbox.ylen();
        let z = bbox.l.z + rand::random::<f32>() * bbox.zlen();

        let vx = 10.0 - rand::random::<f32>() * 20.0;
        let vy = 10.0 - rand::random::<f32>() * 20.0;
        let vz = 10.0 - rand::random::<f32>() * 20.0;

        Boid {
            pos: Vec3::new(x, y, z),
            vel: Vec3::new(vx, vy, vz),
            acc: nalgebra::zero(),
        }
    }

    fn update(&mut self, dt: f32, world_scale: f32) {
        // TODO: figure out where to put these speed constants
        let max_speed = 25.0 * world_scale;
        let min_speed = 4.0 * world_scale;

        self.vel = self.vel + self.acc * dt;
        let curr_speed = nalgebra::norm(&self.vel);
        if curr_speed > max_speed {
            self.vel = self.vel / curr_speed * max_speed;
        } else if curr_speed < min_speed {
            self.vel = self.vel / curr_speed * min_speed;
        }

        self.pos = self.pos + self.vel * dt;
    }
} 

fn bounds_v(p: &Boid, bbox: &AABB) -> Vec3<f32> {
    let mut bounds: Vec3<f32> = nalgebra::zero();

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

    if bounds != nalgebra::zero() {
        nalgebra::normalize(&bounds)
    } else {
        bounds // a zero vector
    }
}

fn avoid_spheres_v(p: &Boid, collide_radius2: f32, sphere_pos: &Vec3<f32>, sphere_r: f32) -> Vec3<f32> {
    let disp = *sphere_pos - p.pos;
    let dist2 = nalgebra::sqnorm(&disp) - sphere_r * sphere_r; //subtract radius of sphere
    if dist2 < collide_radius2 {
        -disp / dist2
    } else {
        nalgebra::zero()
    }
}

fn avoid_cylinder_v(p: &Boid, collide_radius2: f32, cyl_pos: &Vec3<f32>, cyl_h: f32, cyl_r: f32) -> Vec3<f32> {
    let hh = cyl_h / 2.0;
    if p.pos.y < cyl_pos.y + hh && p.pos.y > cyl_pos.y - hh {
        let p2d = Vec2::new(p.pos.x, p.pos.z);
        let c2d = Vec2::new(cyl_pos.x, cyl_pos.z);

        let disp = c2d - p2d;
        let dist2 = nalgebra::sqnorm(&disp) - cyl_r * cyl_r; //subtract radius of sphere
        if dist2 < collide_radius2 {
            let v = -disp / dist2;
            Vec3::new(v.x, 0.0, v.y)
        } else {
            nalgebra::zero()
        }
    } else {
        nalgebra::zero()
    }
}

fn main() {
    let args = parse_arguments();

    let mut window = Window::new("paperboids (kiss3d)");
    window.set_framerate_limit(Some(60));
    window.set_light(light::StickToCamera);

    let debug = args.opt_present("debug");
    let follow_first_bird = args.opt_present("followfirst");
    let show_z_order = args.opt_present("showzorder");
    let show_look_radius = args.opt_present("showlookradius");
    //let show_octree = opt_uint("showoctree");
    let show_octree = 0u; //opt_uint("showoctree");

    let world_box = AABB::new(nalgebra::zero(), Vec3::new(100.0f32, 100.0, 100.0));

    let world_scale = opt_f32(&args, "worldscale", 1.0);
    let look_radius = opt_f32(&args, "lookradius", 30.0 * world_scale);
    // TODO: figure out a good collide radius
    let collide_radius = opt_f32(&args, "collideradius", 8.0 * world_scale);

    let look_radius2 = look_radius * look_radius; // can avoid squareroot for dist calculations
    let collide_radius2 = collide_radius * collide_radius;

    // camera setup
    let eye = Pnt3::new(0.0, world_box.h.y, -world_box.h.z);
    let at = Pnt3::new(world_box.h.x / 2.0, world_box.h.y / 2.0, world_box.h.z / 2.0);
    let mut arc_ball = ArcBall::new(eye, at);

    let default_weights  = [
        30.0, // avoid obstacles
        12.0, // collision avoidance
        8.0,  // flock centering
        9.0,  // match velocity
        20.0, // bounds push
    ];
    let weights =  opt_weights(&args, "weights", default_weights, 5);
    let shared_weights = Arc::new(weights);

    let max_mag = 100.0;

    let num_planes = opt_uint(&args, "boids", 100);

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
//    let cyl_height = world_box.ylen();
//    let mut cyl1 = window.add_cylinder(cyl_radius, cyl_height);
//    let cyl1_pos = Vec3::new(30.0, cyl_height / 2.0, 40.0);
//    cyl1.set_local_translation(cyl1_pos);
//    cyl1.set_color(1.0, 0.0, 0.0);
//    enable_wireframe(&mut cyl1);

    // shrink the world bouonding box to prevent birds from spawning / moving outside the world
    let mut fly_bbox = world_box.clone();
    fly_bbox.scale_center(0.8);
    let fly_bbox = fly_bbox;

    let pmesh = gen_plane_mesh();
    let octmesh = gen_octagon_mesh();
    let mut ps = Vec::with_capacity(num_planes);
    let mut pnodes = Vec::with_capacity(num_planes);
    for _ in range(0, num_planes) {
        ps.push(Boid::new(&fly_bbox));

        let mut node = window.add_mesh(pmesh.clone(), Vec3::new(world_scale, world_scale, world_scale));
        node.set_color(1.0, 1.0, 1.0);
        node.enable_backface_culling(false);
        enable_wireframe(&mut node);
        if show_look_radius { // cylindar to represent look radius
            let lr = look_radius / world_scale; //undo scale, as parent is scaled
            let mut oct = node.add_mesh(octmesh.clone(), Vec3::new(lr, lr, lr));
            oct.set_local_translation(Vec3::new(0.0, -0.1, 0.0));
            oct.set_color(0.2, 0.2, 0.2);
            enable_wireframe(&mut oct);
        }

        pnodes.push(node);
    }

    let shared_ps = Arc::new(RWLock::new(ps));

    let octree = Octree::new(world_box);
    let shared_octree = Arc::new(RWLock::new(octree));

    let threads = opt_uint(&args, "threads", 4);
    let work_size = num_planes / threads;

    // timing values - usage: time_map.insert(OctreeBuild, timer.elapsedms());
    let _frame = "00.frame";
    let _zsort = "01.zsort";
    let _octree_build = "02.octree_build";
    let _octree_update = "03.octree_update";
    let _update_birds = "04.0.update_birds";
    let _update_birds_inner_wait = "04.1.update_birds_inner_wait";
    let _update_birds_inner = "04.2.update_birds_inner";

    let mut time_map = TimeMap::new();

    let mut last_time = clock_ticks::precise_time_ns();
    let mut curr_time;

    let mut frame_count = 0u;
    let mut thread_times: Vec<f64> = Vec::from_fn(threads, |_| 0.0);

    let mut frame_t = Timer::new();
    let mut frame_avg = 0.0;
    let mut section_t = Timer::new();

    println!("starting main loop");
    while window.render_with_camera(&mut arc_ball) {
        frame_t.start();
        curr_time = clock_ticks::precise_time_ns();

        // ZSort start
        section_t.start();
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

                    window.draw_line(p1.pos.as_pnt(), p2.pos.as_pnt(), &Pnt3::new(0.0, 1.0, 1.0));
                }
            }
        }
        let sorted = sorted;
        time_map.update(_zsort, section_t.stop());

        // Octree build start
        {
            let mut octree = shared_octree.write();
            octree.reset(world_box);
            let ps = shared_ps.read();
            section_t.start();
            octree.insert(&*ps);
            time_map.update(_octree_build, section_t.stop());

            section_t.start();
            octree.update(&*ps);
            time_map.update(_octree_update, section_t.stop());

            if show_octree > 0 {
                for o in octree.pool.iter() {
                    match o.state {
                        octree::Node => {
                            if show_octree >= 2 { tmp_draw_aabb(&mut window, &o.b) }
                        },
                        octree::Leaf => {
                            if show_octree == 1 || show_octree == 3 { tmp_draw_aabb(&mut window, &o.b) }
                        },
                        _ => {},
                    }
                }
            }
        }

        let (tx, rx) = channel();

        for tid in range(0, threads) {
            //let b = barrier.clone();
            let child_ps = shared_ps.clone();
            let child_octree = shared_octree.clone();
            let child_weights = shared_weights.clone();
            let tx = tx.clone();

            spawn(proc() {
                let mut thread_t = Timer::new();
                thread_t.start();
                let lock_ps = child_ps.read();
                let ps = lock_ps.as_slice();
                let weights = child_weights.as_slice();

                let octree = child_octree.read();

                let start_id = tid * work_size;
                let work_size =
                    if tid == threads - 1 {
                        work_size + num_planes % threads
                    } else {
                        work_size
                    };

                let mut acc_list = Vec::with_capacity(work_size);

                for i in range(start_id, start_id + work_size) {
                    let mut rules: [Vec3<f32>, ..5] = [nalgebra::zero(), nalgebra::zero(), nalgebra::zero(), nalgebra::zero(), nalgebra::zero()];
                    // let mut neighbors = 0u;
                    // let mut colliders = 0u;

                    {
                        let p = &ps[i];

                        // for j in range(0, num_planes) {
                        //     if i == j { continue; }

                        //     let o = &ps[j];

                        //     let disp = o.pos - p.pos;
                        //     let dist2 = nalgebra::sqnorm(&disp);

                        //     if dist2 < look_radius2 {
                        //         neighbors += 1;

                        //         // fly to center
                        //         rules[2] = rules[2] + o.pos;

                        //         // fly in the same direction
                        //         rules[3] = rules[3] + o.vel;

                        //         // avoid others
                        //         if dist2 < collide_radius2 {
                        //             colliders += 1;
                        //             rules[1] = rules[1] - (disp / dist2);
                        //         }
                        //     }
                        // }

                        // if neighbors > 0 {
                        //     rules[2] = nalgebra::normalize(&(rules[2] / neighbors as f32 - p.pos)) * weights[2];
                        //     rules[3] = nalgebra::normalize(&(rules[3] / neighbors as f32)) * weights[3];
                        // }
                        // if colliders > 0 {
                        //     rules[1] = nalgebra::normalize(&(rules[1] / colliders as f32)) * weights[1];
                        // }

                        //let (r1, r2, r3) = calc_rules(ps, num_planes, i, look_radius2, collide_radius2);
                        let (r1, r2, r3) = calc_rules_octree(ps, num_planes, i, &*octree, look_radius, look_radius2, collide_radius2);
                        rules[1] = r1 * weights[1];
                        rules[2] = r2 * weights[2];
                        rules[3] = r3 * weights[3];

                        rules[4] = bounds_v(p, &fly_bbox) * weights[4];

                        //rules[0] =
                        //    avoid_spheres_v(p, collide_radius2, &sph1_pos, sph_radius) +
                        //    avoid_cylinder_v(p, collide_radius2, &cyl1_pos, cyl_height, cyl_radius);
                        //if rules[0] != nalgebra::zero() {
                        //    rules[0] = nalgebra::normalize(&rules[0]) * weights[0];
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

                    let mut acc: Vec3<f32> = nalgebra::zero();

                    for r in range(0, 5) {
                        // TODO: minor optimization? use non-sqrt norm
                        let m = nalgebra::norm(&rules[r]);

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

        let dt  = (curr_time - last_time) as f32 / 1e9; // in seconds

        //let mut update_birds_t = Timer::new();
        //let mut update_birds_inner_t = Timer::new();
        //let mut update_birds_inner2_t = Timer::new();
        //let mut update_birds_inner_v = 0.0;
        //let mut update_birds_inner2_v = 0.0;
        //update_birds_t.start();
        for _ in range(0, threads) {
            let (tid, thread_time, acc_list) = rx.recv();
            let start_id = tid * work_size;
            thread_times[tid] += thread_time;

            //update_birds_inner_t.start();
            for i in range(start_id, start_id + acc_list.len()) {
                let mut ps = shared_ps.write();
                {
                    //update_birds_inner2_t.start();
                    let p = &mut ps[i];
                    p.acc = acc_list[i - start_id];
                    p.update(dt, world_scale);
                    update_scenenode(p, &mut pnodes[i]);
                    //update_birds_inner2_v += update_birds_inner2_t.stop();
                }
            }
            //update_birds_inner_v += update_birds_inner_t.stop();
        }
        //time_map.update(_update_birds, update_birds_t.stop());
        //time_map.update(_update_birds_inner_wait, update_birds_inner_v);
        //time_map.update(_update_birds_inner, update_birds_inner2_v);

        if debug {
            let ps = shared_ps.read();
            for i in range(0, num_planes) {
                let p = ps[i];
                //println!("{}: pos: {}, vel: {}", i, p.pos, p.vel);
                if p.pos == nalgebra::zero() {
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
            arc_ball.look_at_z(*p.pos.as_pnt(), *(p.pos + p.vel).as_pnt());
        }

        draw_axis(&mut window);

        time_map.update(_frame, frame_t.stop());
        frame_avg += frame_t.elapsedms();
        frame_count += 1;
        if frame_count % 60 == 0 {
            time_map.avg(frame_count);
            for t in thread_times.iter_mut() {
                *t = *t / frame_count as f64;
            }
            frame_avg /= frame_count as f64;
            println!("{:.2} // {}", frame_avg, time_map);
            print!("      // threads: ");
            for t in thread_times.iter_mut() {
                print!("{} ", *t);
                *t = 0.0;
            }
            println!("");
            {
                print!("      // octree stats: ");
                let octree = shared_octree.read();
                octree.stats();
            }
            frame_count = 0;
            frame_avg = 0.0;
            time_map.clear();
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
    let o = Pnt3::new(0.0, 0.0, 0.0);
    let x = Pnt3::new(1.0, 0.0, 0.0);
    let y = Pnt3::new(0.0, 1.0, 0.0);
    let z = Pnt3::new(0.0, 0.0, 1.0);

    w.draw_line(&o, &x, &x);
    w.draw_line(&o, &y, &y);
    w.draw_line(&o, &z, &z);
}

fn enable_wireframe(n: &mut SceneNode) {
    n.set_points_size(1.0); //wireframe mode for plane
    n.set_lines_width(1.0);
    n.set_surface_rendering_activation(false);
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

fn z_order_planes(ps: &Vec<Boid>) -> Vec<(uint, u32)> {
    let mut zord_id: Vec<(uint, u32)> = Vec::with_capacity(ps.len());

    for i in range(0, ps.len()) {
        zord_id.push((i, morton_3d(&ps[i].pos)));
    }

    zord_id.sort_by(|&(_, am), &(_, bm)| am.cmp(&bm));

    zord_id
}

fn tmp_draw_aabb(w: &mut Window, b: &AABB) {
    let y = Pnt3::new(1.0, 1.0, 0.0);
    let c: [Pnt3<f32>, ..8] = [
        Pnt3::new(b.l.x, b.l.y, b.l.z),
        Pnt3::new(b.h.x, b.l.y, b.l.z),
        Pnt3::new(b.l.x, b.h.y, b.l.z),
        Pnt3::new(b.h.x, b.h.y, b.l.z),

        Pnt3::new(b.l.x, b.l.y, b.h.z),
        Pnt3::new(b.h.x, b.l.y, b.h.z),
        Pnt3::new(b.l.x, b.h.y, b.h.z),
        Pnt3::new(b.h.x, b.h.y, b.h.z),
    ];

    w.draw_line(&c[0], &c[1], &y);
    w.draw_line(&c[1], &c[3], &y);
    w.draw_line(&c[3], &c[2], &y);
    w.draw_line(&c[2], &c[0], &y);

    w.draw_line(&c[0], &c[4], &y);
    w.draw_line(&c[2], &c[6], &y);
    w.draw_line(&c[1], &c[5], &y);
    w.draw_line(&c[3], &c[7], &y);

    w.draw_line(&c[4], &c[5], &y);
    w.draw_line(&c[5], &c[7], &y);
    w.draw_line(&c[7], &c[6], &y);
    w.draw_line(&c[6], &c[4], &y);
}

// returns normalized results
fn calc_rules(ps: &[Boid], num_planes: uint, i: uint, look_radius2: f32, collide_radius2: f32) -> (Vec3<f32>, Vec3<f32>, Vec3<f32>) {
    let p = &ps[i];
    let mut r1: Vec3<f32> = nalgebra::zero();
    let mut r2: Vec3<f32> = nalgebra::zero();
    let mut r3: Vec3<f32> = nalgebra::zero();
    let mut neighbors = 0u;
    let mut colliders = 0u;

    for j in range(0, num_planes) {
        if i == j { continue; }

        let o = &ps[j];

        let disp = o.pos - p.pos;
        let dist2 = nalgebra::sqnorm(&disp);

        if dist2 < look_radius2 {
            neighbors += 1;

            // fly to center
            r2 = r2 + o.pos;

            // fly in the same direction
            r3 = r3 + o.vel;

            // avoid others
            if dist2 < collide_radius2 {
                colliders += 1;
                r1 = r1 - (disp / dist2);
            }
        }
    }

    if neighbors > 0 {
        r2 = nalgebra::normalize(&(r2 / neighbors as f32 - p.pos));
        r3 = nalgebra::normalize(&r3);
    }
    if colliders > 0 {
        r1 = nalgebra::normalize(&r1);
    }
    (r1, r2, r3)
}

struct TraversalConst<'a, 'b, 'c> {
    //ps: &'a [Boid],
    //num_planes: uint,
    p: &'b Boid,
    //pid: uint,
    octree: &'c Octree,
    look_radius: f32,
    look_radius2: f32,
    collide_radius2: f32,
    theta: f32,
}

struct TraversalRecur {
    r1: Vec3<f32>,
    r2: Vec3<f32>,
    r3: Vec3<f32>,
    neighbors: uint,
    colliders: uint,
    leaves: uint,
    nodes: uint,
    small_nodes: uint,
}

fn calc_rules_octree(ps: &[Boid], num_planes: uint, pid: uint, octree: &Octree, look_radius: f32, look_radius2: f32, collide_radius2: f32) -> (Vec3<f32>, Vec3<f32>, Vec3<f32>) {
    let p = &ps[pid];
    let tc = TraversalConst {
        //ps: ps,
        //num_planes: num_planes,
        p: p,
        //pid: pid,
        octree: octree,
        look_radius: look_radius,
        look_radius2: look_radius2,
        collide_radius2: collide_radius2,
        theta: 1.0, // TODO: figure out this value
    };

    let mut tr = TraversalRecur {
        r1: nalgebra::zero(),
        r2: nalgebra::zero(),
        r3: nalgebra::zero(),
        neighbors: 0u,
        colliders: 0u,
        leaves: 0u,
        nodes: 0u,
        small_nodes: 0u,
    };

    traverse_octree(&tc, &mut tr, tc.octree.root);

    if tr.neighbors > 0 {
        tr.r2 = nalgebra::normalize(&(tr.r2 / tr.neighbors as f32 - p.pos));
        tr.r3 = nalgebra::normalize(&tr.r3);
    }
    if tr.colliders > 0 {
        tr.r1 = nalgebra::normalize(&tr.r1);
    }

    //println!("leaves: {}, nodes: {}, small node: {}", tr.leaves, tr.nodes, tr.small_nodes);
    (tr.r1, tr.r2, tr.r3)
}

fn traverse_octree(tc: &TraversalConst, tr: &mut TraversalRecur, curr: OctnodeId) {
    if !curr.is_set() { //TODO: should be impossible, need to verify
        return;
    }

    let o = tc.octree.get_node(curr);
    let dv = o.c - tc.p.pos;
    let d = nalgebra::norm(&dv);

   if o.is_leaf() {
       if d < 1e-6 { return } // skip self

       single_interact(tc, tr, o, &dv, d);
       tr.leaves += 1;
   } else if d / o.width() >= tc.theta {
       // close enough, use averages
       single_interact(tc, tr, o, &dv, d);
       tr.nodes += 1;
   } else if o.width() < tc.look_radius / 4.0 {
       // arbitrary value, but if the box is inside the look radius, don't merge down
       single_interact(tc, tr, o, &dv, d);
       tr.small_nodes += 1;
   } else {
       for i in range(0, 8) {
           let cid = o.child[i];
           if cid.is_set() {
               traverse_octree(tc, tr, cid);
           }
       }
   }
}

fn single_interact(tc: &TraversalConst, tr: &mut TraversalRecur, o: &Octnode, dv: &Vec3<f32>, d: f32) {
    let d2 = d*d;

    if d2 < tc.look_radius2 {
        tr.neighbors += 1;

        tr.r2 = tr.r2 + o.c;
        tr.r3 = tr.r3 + o.v;

        if d2 < tc.collide_radius2 {
            tr.colliders += 1;
            tr.r1 = tr.r1 - *dv / d2;
        }
    }
}

fn parse_arguments() -> Matches {
    let args = std::os::args();

    let opts = [
        optflag("d", "debug", "enable debug mode"),
        optflag("", "showzorder", "show z-order curve"),
        optflag("", "showlookradius", "show look radius for each bird"),
        optopt("", "showoctree", "show octree, 0: off, 1: leaves, 2: nodes, 3: full tree", "0"),

        optflag("", "followfirst", "follow the first bird"),

        optopt("s", "worldscale", "set world scale", "0.5"),
        optopt("", "lookradius", "set bird look radius", "30.0"),
        optopt("", "collideradius", "set collision check radius", "8.0"),

        optopt("t", "threads", "number of threads to use", "4"),
        optopt("b", "boids", "number of boids to use", "100"),
        optopt("w", "weights", "five (5) comma delimited weights", "30.0,12.0,8.0,8.0,20.0"),
        optopt("m", "mode", "0: original, 1: octree lookup, 2: barnes hut", "2"),
    ];

    match getopts(args.tail(), opts) {
        Ok(m) => m,
        Err(f) => panic!(f.to_string()),
    }
}

fn opt_f32(args: &Matches, opt: &str, default: f32) -> f32 {
    match args.opt_str(opt) {
        Some(v) => match from_str::<f32>(v.as_slice()) {
            Some(v) => v,
            None => default,
        },
        None => default,
    }
}

fn opt_uint(args: &Matches, opt: &str, default: uint) -> uint {
    match args.opt_str(opt) {
        Some(v) => match from_str::<uint>(v.as_slice()) {
            Some(v) => v,
            None => default,
        },
        None => default,
    }
}

fn opt_weights(args: &Matches, opt: &str, default: [f32, ..5], expected_len: uint) -> Vec<f32> {
    let arr_string = match args.opt_str(opt) {
        Some(v) => v,
        None => String::from_str(""),
    };
    let elem: Vec<&str> = arr_string.as_slice().split(',').collect();

    let mut result = Vec::with_capacity(elem.len());

    let mut error = false;
    if elem.len() == expected_len {
        // try to parse, if fail set error to true
        for e in elem.iter() {
            let f = match from_str::<f32>(e.as_slice()) {
                Some(v) => v,
                None => { error = true; break; },
            };
            result.push(f);
        }
    } else {
        error = true;
    }

    if error {
        result = default.to_vec();
    }

    result
}


fn gen_plane_mesh() -> Rc<RefCell<Mesh>> {
    let vertices = vec!(
        Pnt3::new(0.0, 0.0, 1.0), // front / nose
        Pnt3::new(0.75, 0.0, -1.0), // left wing - 'port'
        Pnt3::new(-0.75, 0.0, -1.0), // right wing - 'starboard'
        Pnt3::new(0.0, 0.0, -1.0), // back midpoint between wings
        Pnt3::new(0.0, -0.4, -1.0), // back bottom fin
        );

    let indices = vec!(
        Vec3::new(0u32, 1, 3),
        Vec3::new(0u32, 3, 2),
        Vec3::new(0u32, 4, 3),
        );

    Rc::new(RefCell::new(Mesh::new(vertices, indices, None, None, false)))
}

fn gen_octagon_mesh() -> Rc<RefCell<Mesh>> {
    let vertices = vec!(
        Pnt3::new(0.0, 0.0, 0.0),
        Pnt3::new(1.0, 0.0, 0.0),
        Pnt3::new(0.71, 0.0, 0.71),
        Pnt3::new(0.0, 0.0, 1.0),
        Pnt3::new(-0.71, 0.0, 0.71),
        Pnt3::new(-1.0, 0.0, 0.0),
        Pnt3::new(-0.71, 0.0, -0.71),
        Pnt3::new(0.0, 0.0, -1.0),
        Pnt3::new(0.71, 0.0, -0.71),
        );

    let indices = vec!(
        Vec3::new(0u32, 1, 2),
        Vec3::new(0u32, 2, 3),
        Vec3::new(0u32, 3, 4),
        Vec3::new(0u32, 4, 5),
        Vec3::new(0u32, 5, 6),
        Vec3::new(0u32, 6, 7),
        Vec3::new(0u32, 7, 8),
        Vec3::new(0u32, 8, 1),
        );

    Rc::new(RefCell::new(Mesh::new(vertices, indices, None, None, false)))
}

fn update_scenenode(b: &Boid, node: &mut SceneNode) {
    // TODO: figure out up vector for banking
    node.look_at_z(b.pos.as_pnt(), (b.pos + b.vel).as_pnt(), &Vec3::y());
}
