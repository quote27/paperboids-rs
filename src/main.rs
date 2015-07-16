#![feature(negate_unsigned)]
#![allow(mutable_transmutes)]
extern crate gl;
extern crate glfw;
extern crate cgmath;
extern crate time;
extern crate rand;

use std::thread;
use std::sync::mpsc;
use std::sync::Arc;
use std::mem;
use gl::types::*;
use glfw::{Action, Context, Key};
use cgmath::*;
use shaders::{Shader, Program};
use mesh::Mesh;
use timer::{Timer, TimeMap};
use aabb::AABB;
use boids::Boid;
use octree::{Octree, Octnode};

mod shaders;
mod mesh;
mod timer;
mod aabb;
mod boids;
mod octree;

// todo list
// TODO: migrate octree build and lookup
// TODO: add command line options
// TODO: debug - draw aabb for octree
// TODO: draw axis
// TODO: figure out why birds seem to be moving faster than before

static VS_SRC: &'static str = "
#version 330 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
layout (location = 2) in mat4 model_inst;

out vec3 o_color;

uniform mat4 view;
uniform mat4 proj;

void main() {
    o_color = color;
    gl_Position = proj * view * model_inst * vec4(position, 1.0);
}";

static FS_SRC: &'static str = "
#version 330 core
in vec3 o_color;
out vec4 out_color;

void main() {
    out_color = vec4(o_color, 1.0);
}";

fn main() {
    println!("paperboids begin");
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    glfw.window_hint(glfw::WindowHint::ContextVersion(3, 2));
    glfw.window_hint(glfw::WindowHint::OpenGlForwardCompat(true));
    glfw.window_hint(glfw::WindowHint::OpenGlProfile(glfw::OpenGlProfileHint::Core));
    glfw.window_hint(glfw::WindowHint::Resizable(true));

    let (mut window, events) = glfw.create_window(300, 300, "paperboids", glfw::WindowMode::Windowed)
        .expect("failed to create glfw window");

    gl::load_with(|s| window.get_proc_address(s));

    window.set_key_polling(true);
    window.set_framebuffer_size_polling(true);
    window.make_current();
    //glfw.set_swap_interval(0); // set this to 0 to unlock frame rate

    // config variables
    let threads = 4;
    let world_bounds = AABB::new(Vector3::zero(), Vector3::new(100.0, 100.0, 100.0));
    let world_scale = 1.0;
    let look_radius = 30.0 * world_scale;
    let look_radius2 = look_radius * look_radius;
    let collide_radius = 8.0 * world_scale;
    let collide_radius2 = collide_radius * collide_radius;
    let max_mag = 100.0;
    let num_boids = 1;
    let work_size = num_boids / threads;

    let default_weights  = vec![
        30.0, // avoid obstacles
        12.0, // collision avoidance
        8.0,  // flock centering
        9.0,  // match velocity
        20.0, // bounds push
    ];
    let weights = default_weights; // opt_weights(&args, "weights", default_weights, 5);
    let shared_weights = Arc::new(weights);

    let mut fly_bbox = world_bounds.clone();
    fly_bbox.scale_center(0.5);
    let fly_bbox = fly_bbox;

    unsafe {
        gl::Enable(gl::DEPTH_TEST);
    }

    println!("creating shaders");
    let shaders_v = vec![
        Shader::from_str(gl::VERTEX_SHADER, &VS_SRC),
        Shader::from_str(gl::FRAGMENT_SHADER, &FS_SRC),
    ];
    gl_error_str("shaders created");

    println!("creating program");
    let prog = Program::new(&shaders_v);
    gl_error_str("program created");


    println!("use program");
    prog.use_prog();
    let pos_a = prog.get_attrib("position") as GLuint;
    let color_a = prog.get_attrib("color") as GLuint;
    let model_inst_a = prog.get_attrib("model_inst") as GLuint;


    println!("setting up models");
    let model_default_scale_mat = Matrix4::identity(); // Matrix4::from(Matrix3::from_value(world_scale));

    println!("generating {} boids", num_boids);
    let mut bs = Vec::with_capacity(num_boids);
    for _ in 0..num_boids {
        bs.push(Boid::random_new(&world_bounds))
    }

    let mut model_inst = Vec::with_capacity(bs.len());
    for b in bs.iter() {
        model_inst.push(b.model() * model_default_scale_mat);
    }
    let mut plane_mesh = gen_paperplane_mesh();
    plane_mesh.setup(pos_a, color_a, model_inst_a);
    plane_mesh.update_inst(&model_inst);

    println!("setting up octree");
    let octree = Octree::new(world_bounds);

    println!("setting up arc wrappers for: bs, model_inst, octree");
    let shared_bs = Arc::new(bs);
    let shared_model_inst = Arc::new(model_inst);
    let shared_octree = Arc::new(octree);

    // other models
    let cube_model_inst = vec![
        Matrix4::from_translation(&world_bounds.center()) *
            Matrix4::from(Matrix3::from_value(world_bounds.xlen())),
        Matrix4::from_translation(&fly_bbox.center()) *
            Matrix4::from(Matrix3::from_value(fly_bbox.xlen())),
    ];
    let mut cube_mesh = gen_cube_mesh();
    cube_mesh.setup(pos_a, color_a, model_inst_a);
    cube_mesh.update_inst(&cube_model_inst);

    let axis_model_inst = vec![
        //Matrix4::from_translation(&world_bounds.center()) *
        Matrix4::from_translation(&Vector3::new(0.5, 0.5, 0.5)) *
            Matrix4::from(Matrix3::from_value(10.0)),
    ];
    let mut axis_mesh = gen_axis_mesh();
    axis_mesh.setup(pos_a, color_a, model_inst_a);
    axis_mesh.update_inst(&axis_model_inst);

    println!("setting up uniforms");
    let alpha_u = prog.get_unif("alpha");
    let view_u = prog.get_unif("view");
    let proj_u = prog.get_unif("proj");

    let mut view_angle = deg(0.0f32);
    let mut view_angle_update = true;

    let mut proj_m4 = perspective(deg(45.0), 800.0 / 600.0, 0.01, 1000.0);
    let eye = Point3::new(world_bounds.h.x * 2.0, world_bounds.h.y * 1.5, world_bounds.h.z * 2.0);
    let target = Point3::from_vec(&world_bounds.center());
    let mut view_m4 = Matrix4::look_at(&eye, &target, &Vector3::unit_y());

    proj_u.upload_m4f(&proj_m4);
    view_u.upload_m4f(&view_m4);
    alpha_u.upload_1f(1.0);

    println!("setting up timers");
    let mut tm = TimeMap::new();
    let mut frame_t = Timer::new();
    let mut section_t = Timer::new();
    let mut compute_t = Timer::new();
    let mut frame_total = 0.0;

    let tm_frame = "00.frame";
    let tm_events = "01.events";
    let tm_compute = "02.0.compute";
    let tm_compute_octree_build = "02.1.octree_build";
    let tm_compute_rules = "02.2.rules";
    let tm_compute_update = "02.3.update";
    let tm_compute_update_inst = "02.4.update_inst";
    let tm_draw_inst ="03.draw_inst";

    let mut pause = true;
    let mut frame_count = 0;

    frame_t.start();
    println!("starting main loop");
    while !window.should_close() {
        let tlastframe = frame_t.stop();
        frame_t.start();

        section_t.start();
        glfw.poll_events();
        for (_, event) in glfw::flush_messages(&events) {
            match event {
                glfw::WindowEvent::Key(Key::Escape, _, Action::Press, _) => {
                    window.set_should_close(true);
                }
                glfw::WindowEvent::Key(Key::P, _, Action::Press, _) => {
                    pause = !pause;
                }
                glfw::WindowEvent::FramebufferSize(w, h) => {
                    unsafe { gl::Viewport(0, 0, w, h); }
                    proj_m4 = perspective(deg(45.0), w as f32 / h as f32, 0.01, 1000.0);
                    proj_u.upload_m4f(&proj_m4);
                }
                glfw::WindowEvent::Key(Key::Left, _, Action::Press, _) => {
                    view_angle = view_angle + deg(180.0);
                    view_angle_update = true;
                }
                glfw::WindowEvent::Key(Key::Left, _, Action::Repeat, _) => {
                    view_angle = view_angle + deg(180.0);
                    view_angle_update = true;
                }
                glfw::WindowEvent::Key(Key::Right, _, Action::Press, _) => {
                    view_angle = view_angle - deg(180.0);
                    view_angle_update = true;
                }
                glfw::WindowEvent::Key(Key::Right, _, Action::Repeat, _) => {
                    view_angle = view_angle - deg(180.0);
                    view_angle_update = true;
                }
                _ => {}
            }
        }

        if view_angle_update {
            view_angle_update = false;

            let target2d = Vector2::new(target.x, target.z);
            let eye2d = Vector2::new(eye.x, eye.z);
            let dir = eye2d - target2d;

            let rot2d: Basis2<f32> = Rotation2::from_angle(view_angle.into());
            let new_dir = rot2d.rotate_vector(&dir);

            let new_eye2d = target2d + new_dir;
            let new_eye3d = Point3::new(new_eye2d.x, eye.y, new_eye2d.y);
            view_m4 = Matrix4::look_at(&new_eye3d, &target, &Vector3::unit_y());
            view_u.upload_m4f(&view_m4);
        }
        tm.update(tm_events, section_t.stop());

        unsafe {
            gl::ClearColor(0.0, 0.0, 0.0, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        compute_t.start();
        if !pause {
            section_t.start();
            let dt = (tlastframe as f32) * 1e-3; // convert from ms to sec

            // octree build step
            {
                // TODO: need to figure out a non-hack for read/write here
                // I could use a RWLock like before, but that won't help when
                // it comes to parallel builds
                let octree: &mut Octree = unsafe { mem::transmute(&*shared_octree) };
                octree.reset(world_bounds);

                let bs = shared_bs.clone(); // just need read access

                octree.insert(&*bs);
                octree.update(&*bs);

                // TODO: add octree debug drawing using cubes
            }
            tm.update(tm_compute_octree_build, section_t.stop());

            section_t.start();
            // simulation run step
            let (tx, rx) = mpsc::channel();
            for tid in 0..threads {
                let thread_tx = tx.clone();
                let thread_weights = shared_weights.clone();
                let thread_bs = shared_bs.clone();
                let thread_octree = shared_octree.clone();

                thread::spawn(move || {
                    let bs = thread_bs;
                    let octree = thread_octree;
                    let weights = thread_weights;

                    let start_id = tid * work_size;
                    let work_size =
                        if tid == threads - 1 {
                            work_size + num_boids % threads
                        } else {
                            work_size
                        };

                    let mut rules = vec![Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero()];
                    for i in start_id..(start_id + work_size) {
                        rules[0] = Vector3::zero();

                        {
                            //let (r1, r2, r3) = calc_rules(&bs, i, look_radius2, collide_radius2);
                            let (r1, r2, r3) = calc_rules_octree(&bs, i, &*octree, look_radius, look_radius2, collide_radius2);
                            rules[1] = r1.mul_s(weights[1]);
                            rules[2] = r2.mul_s(weights[2]);
                            rules[3] = r3.mul_s(weights[3]);

                            rules[4] = bounds_v(&bs[i], &fly_bbox).mul_s(weights[4]);
                        }

                        let mut mag = 0.0;
                        let mut acc = Vector3::zero();

                        for r in rules.iter() {
                            let m = r.length();

                            // TODO: change this to epsilon
                            if m == 0.0 { continue; }

                            if mag + m > max_mag {
                                // rebalance last rule
                                let r = r.mul_s((max_mag - mag) / m);
                                acc = acc + r;
                                break;
                            }

                            mag += m;
                            acc = acc + *r;
                        }

                        // i know this is hacky, but can't think of another way right now =\
                        unsafe {
                            let b = &bs[i];
                            let b: &mut Boid = mem::transmute(b);
                            b.acc = acc;
                        }
                    }

                    thread_tx.send(0u32);
                });
            }

            for _ in 0..threads {
                rx.recv();
            }
            tm.update(tm_compute_rules, section_t.stop());

            section_t.start();
            for tid in 0..threads {
                let thread_tx = tx.clone();
                let thread_bs = shared_bs.clone();
                let thread_model_inst = shared_model_inst.clone();

                thread::spawn(move || {
                    let bs = thread_bs;
                    let model_inst = thread_model_inst;

                    let start_id = tid * work_size;
                    let work_size =
                        if tid == threads - 1 {
                            work_size + num_boids % threads
                        } else {
                            work_size
                        };

                    for i in start_id..(start_id + work_size) {
                        unsafe {
                            let b = &bs[i];
                            let b: &mut Boid = mem::transmute(b);
                            b.update(dt, world_scale);

                            let m = &model_inst[i];
                            let m: &mut Matrix4<f32> = mem::transmute(m);
                            *m = b.model() * model_default_scale_mat;
                        }
                    }

                    thread_tx.send(0u32);
                });
            }

            for _ in 0..threads {
                rx.recv();
            }
            tm.update(tm_compute_update, section_t.stop());


            section_t.start();
            plane_mesh.update_inst(&shared_model_inst);
            tm.update(tm_compute_update_inst, section_t.stop());
        }

        tm.update(tm_compute, compute_t.stop());

        // draw planes
        section_t.start();
        plane_mesh.draw_inst(shared_model_inst.len() as GLint);
        tm.update(tm_draw_inst, section_t.stop());

        cube_mesh.draw_inst(cube_model_inst.len() as GLint);
        axis_mesh.draw_inst(axis_model_inst.len() as GLint);

        window.swap_buffers();
        frame_count += 1;
        tm.update(tm_frame, frame_t.stop());
        frame_total += frame_t.elapsedms();

        if frame_count % 60 == 0 {
            tm.avg(60);
            let frame_avg = frame_total / 60.0;
            println!("{:.2} // {}", frame_avg, tm);
            frame_total = 0.0;
            tm.clear();
        }
    }

    println!("paperboids end");
}

fn gl_error() {
    let er = unsafe { gl::GetError() };
    if er != 0 {
        println!("gl error? {}", er);
    }
}

fn gl_error_str(s: &str) {
    let er = unsafe { gl::GetError() };
    if er != 0 {
        println!("gl error? {} - {}", er, s);
    }
}

fn gen_paperplane_mesh() -> Mesh {
    // z+ is front
    // y+ is top
    let vertices = vec![
        // vertex        // color
        0.0,  0.0,  1.0, 1.0, 1.0, 1.0f32, // front / nose
        0.75, 0.0, -1.0, 1.0, 1.0, 1.0,    // left wing / 'port'
       -0.75, 0.0, -1.0, 1.0, 1.0, 1.0,    // right wing / 'starboard
        0.0,  0.0, -1.0, 1.0, 1.0, 1.0,    // back midpoint between wings
        0.0, -0.4, -1.0, 1.0, 1.0, 1.0,    // back bottom fin
    ];
    let vertex_size = 6;

    // triangles
    // let elements = vec![
    //     0u32, 1, 3,
    //     0, 3, 2,
    //     0, 4, 3,
    // ];

    // lines
    let elements = vec![ 0u32, 1, 2, 0, 3, 4 ];

    Mesh::new("paperplane", vertices, elements, vertex_size)
}

fn gen_square_mesh() -> Mesh {
    let vertices = vec![
        // vertex        // color
        0.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  1.0,  0.0, 1.0, 1.0, 1.0f32,
        0.0,  1.0,  0.0, 1.0, 1.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = vec![ 0u32, 1, 1, 2, 2, 3, 3, 0 ];
    Mesh::new("square", vertices, elements, vertex_size)
}

fn gen_axis_mesh() -> Mesh {
    let vertices = vec![
        // vertex        // color
        0.0,  0.0,  0.0, 1.0, 0.0, 0.0f32,
        1.0,  0.0,  0.0, 1.0, 0.0, 0.0f32,
        0.0,  0.0,  0.0, 0.0, 1.0, 0.0f32,
        0.0,  1.0,  0.0, 0.0, 1.0, 0.0f32,
        0.0,  0.0,  0.0, 0.0, 0.0, 1.0f32,
        0.0,  0.0,  1.0, 0.0, 0.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = vec![ 0u32, 1, 2, 3, 4, 5 ];

    Mesh::new("axis", vertices, elements, vertex_size)
}

fn gen_cube_mesh() -> Mesh {
    let vertices = vec![
        // vertex        // color
       -0.5, -0.5, -0.5, 1.0, 1.0, 1.0f32,
        0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
        0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
        0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
       -0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5, -0.5, 1.0, 1.0, 1.0,

       -0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
        0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
        0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
        0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5,  0.5, 1.0, 1.0, 1.0,

       -0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5,  0.5,  0.5, 1.0, 1.0, 1.0,

        0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
        0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
        0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
        0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
        0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
        0.5,  0.5,  0.5, 1.0, 1.0, 1.0,

       -0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
        0.5, -0.5, -0.5, 1.0, 1.0, 1.0,
        0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
        0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5, -0.5, -0.5, 1.0, 1.0, 1.0,

       -0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
        0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
        0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
        0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5,  0.5,  0.5, 1.0, 1.0, 1.0,
       -0.5,  0.5, -0.5, 1.0, 1.0, 1.0,
    ];
    let vertex_size = 6;
    let elements = vec![
        0u32, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35,
    ];

    Mesh::new("cube", vertices, elements, vertex_size)
}


fn bounds_v(b: &Boid, bbox: &AABB) -> Vector3<f32> {
    let mut bounds = Vector3::zero();

    bounds.x =
        if b.pos.x > bbox.h.x {
            -1.0
        } else if b.pos.x < bbox.l.x {
            1.0
        } else {
            0.0
        };

    bounds.y =
        if b.pos.y > bbox.h.y {
            -1.0
        } else if b.pos.y < bbox.l.y {
            1.0
        } else {
            0.0
        };

    bounds.z =
        if b.pos.z > bbox.h.z {
            -1.0
        } else if b.pos.z < bbox.l.z {
            1.0
        } else {
            0.0
        };

    if bounds != Vector3::zero() {
        bounds.normalize_self();
    }
    bounds
}

//
// Original algorithm
//

// returns normalized results
fn calc_rules(bs: &Vec<Boid>, bi: usize, look_radius2: f32, collide_radius2: f32) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    let b = &bs[bi];
    let mut r1 = Vector3::zero();
    let mut r2 = Vector3::zero();
    let mut r3 = Vector3::zero();
    let mut neighbors = 0u32;
    let mut colliders = 0u32;

    for j in 0..bs.len() {
        if bi == j { continue; }

        let o = &bs[j];

        let disp = o.pos - b.pos;
        let dist2 = disp.length2();

        if dist2 < look_radius2 {
            neighbors += 1;

            // fly to center
            r2 = r2 + o.pos;

            // fly in the same direction
            r3 = r3 + o.vel;

            // avoid others
            if dist2 < collide_radius2 {
                colliders += 1;
                r1 = r1 - (disp.div_s(dist2));
            }
        }
    }

    if neighbors > 0 {
        r2 = r2.div_s(neighbors as f32) - b.pos;
        r2.normalize_self();
        r3.normalize_self();
    }
    if colliders > 0 {
        r1.normalize_self();
    }
    (r1, r2, r3)
}

//
// Barnes-hut version
//

struct TraversalConst<'b, 'c> {
    b: &'b Boid,
    octree: &'c Octree,
    look_radius: f32,
    look_radius2: f32,
    collide_radius2: f32,
    theta: f32,
}

struct TraversalRecur {
    r1: Vector3<f32>,
    r2: Vector3<f32>,
    r3: Vector3<f32>,
    neighbors: usize,
    colliders: usize,
    leaves: usize,
    nodes: usize,
    small_nodes: usize,
}

fn calc_rules_octree(bs: &Vec<Boid>, boid_id: usize, octree: &Octree, look_radius: f32, look_radius2: f32, collide_radius2: f32) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    let b = &bs[boid_id];
    let tc = TraversalConst {
        b: b,
        octree: octree,
        look_radius: look_radius,
        look_radius2: look_radius2,
        collide_radius2: collide_radius2,
        theta: 1.0, // TODO: figure out this value
    };

    let mut tr = TraversalRecur {
        r1: Vector3::zero(),
        r2: Vector3::zero(),
        r3: Vector3::zero(),
        neighbors: 0,
        colliders: 0,
        leaves: 0,
        nodes: 0,
        small_nodes: 0,
    };

    if tc.octree.root != -1 as usize {
        traverse_octree(&tc, &mut tr, tc.octree.root);
    }

    if tr.neighbors > 0 {
        tr.r2 = (tr.r2.div_s(tr.neighbors as f32) - b.pos).normalize();
        tr.r3.normalize_self();
    }
    if tr.colliders > 0 {
        tr.r1.normalize_self();
    }

    //println!("leaves: {}, nodes: {}, small node: {}", tr.leaves, tr.nodes, tr.small_nodes);
    (tr.r1, tr.r2, tr.r3)
}

fn traverse_octree(tc: &TraversalConst, tr: &mut TraversalRecur, curr: usize) {
    let o = tc.octree.get_node(curr);
    let dv = o.c - tc.b.pos;
    let d = dv.length();

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
        for i in 0..8 {
            let child_id = o.child[i];
            if child_id != -1 as usize {
                traverse_octree(tc, tr, child_id);
            }
        }
    }
}

fn single_interact(tc: &TraversalConst, tr: &mut TraversalRecur, o: &Octnode, dv: &Vector3<f32>, d: f32) {
    let d2 = d*d;

    if d2 < tc.look_radius2 {
        tr.neighbors += 1;

        tr.r2 = tr.r2 + o.c;
        tr.r3 = tr.r3 + o.v;

        if d2 < tc.collide_radius2 {
            tr.colliders += 1;
            tr.r1 = tr.r1 - dv.div_s(d2);
        }
    }
}
