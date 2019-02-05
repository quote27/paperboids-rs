#![allow(mutable_transmutes)]
extern crate cgmath;
extern crate gl;
extern crate glfw;
extern crate gltf;
extern crate rand;
extern crate time;

use aabb::AABB;
use boids::Boid;
use cgmath::*;
use gl::types::*;
use glfw::{Action, Context, Key, Modifiers};
use mesh::Mesh;
use octree::{Octnode, Octree};
use shaders::{Program, Shader};
use std::fs;
use std::io;
use std::mem;
use std::path::Path;
use std::sync::mpsc;
use std::sync::Arc;
use std::thread;
use std::usize;
use timer::{TimeMap, Timer};

mod aabb;
mod boids;
mod mesh;
mod octree;
mod shaders;
mod timer;

// todo list
// TODO: add command line options

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

    let (_width, _height) = (600, 600);

    glfw.window_hint(glfw::WindowHint::ContextVersion(3, 2));
    glfw.window_hint(glfw::WindowHint::OpenGlForwardCompat(true));
    glfw.window_hint(glfw::WindowHint::OpenGlProfile(
        glfw::OpenGlProfileHint::Core,
    ));
    glfw.window_hint(glfw::WindowHint::Resizable(true));

    let (mut window, events) = glfw
        .create_window(_width, _height, "paperboids", glfw::WindowMode::Windowed)
        .expect("failed to create glfw window");

    // TODO: `as *const _` needed to convert glfw's output
    gl::load_with(|s| window.get_proc_address(s) as *const _);

    window.set_key_polling(true);
    window.set_scroll_polling(true);
    window.set_framebuffer_size_polling(true);
    window.make_current();
    //glfw.set_swap_interval(0); // set this to 0 to unlock frame rate

    // config variables
    let threads = 8;
    let world_bounds = AABB::new(Vector3::zero(), Vector3::new(100.0, 100.0, 100.0));
    let world_scale = 0.5;
    let look_radius = 30.0 * world_scale;
    let look_radius2 = look_radius * look_radius;
    let collide_radius = 8.0 * world_scale;
    let collide_radius2 = collide_radius * collide_radius;
    let max_mag = 100.0;
    let mut num_boids = 100;
    let mut work_size = num_boids / threads;

    let default_weights = vec![
        //30.0, // avoid obstacles
        20.0, // avoid obstacles
        12.0, // collision avoidance
        8.0,  // flock centering
        9.0,  // match velocity
        20.0, // bounds push
    ];
    let weights = default_weights; // opt_weights(&args, "weights", default_weights, 5);
    let shared_weights = Arc::new(weights);

    let mut fly_bbox = world_bounds.clone();
    fly_bbox.scale_center(0.8);
    let fly_bbox = fly_bbox;

    unsafe {
        gl::Enable(gl::DEPTH_TEST);
    }

    let shaders_v = vec![
        Shader::from_str(gl::VERTEX_SHADER, &VS_SRC),
        Shader::from_str(gl::FRAGMENT_SHADER, &FS_SRC),
    ];
    gl_error_str("shaders created");

    let prog = Program::new(&shaders_v);
    gl_error_str("program created");

    prog.use_prog();
    let pos_a = prog.get_attrib("position") as GLuint;
    let color_a = prog.get_attrib("color") as GLuint;
    let model_inst_a = prog.get_attrib("model_inst") as GLuint;

    let model_default_scale_mat = Matrix4::from(Matrix3::from_value(world_scale));
    let pred_model_default_scale_mat = Matrix4::from(Matrix3::from_value(world_scale * 5.0));

    let mut bs = Vec::with_capacity(num_boids);
    for _ in 0..num_boids {
        bs.push(Boid::random_new(&fly_bbox))
    }

    let mut model_inst = Vec::with_capacity(bs.len());
    for b in bs.iter() {
        model_inst.push(b.model() * model_default_scale_mat);
    }
    let mut plane_mesh = gen_paperplane_mesh();
    plane_mesh.setup(pos_a, color_a, model_inst_a);
    plane_mesh.update_inst(&model_inst);

    let octree = Octree::new(world_bounds);

    let shared_bs = Arc::new(bs);
    let shared_model_inst = Arc::new(model_inst);
    let shared_octree = Arc::new(octree);

    // antagonist *later will be a sphere)
    let mut predator_boid = Boid::random_new(&fly_bbox);
    let predator_model_inst = vec![predator_boid.model() * pred_model_default_scale_mat];
    // let mut predator_mesh = gen_cube_mesh(&Vector3::new(1.0, 0.0, 0.0));
    let mut predator_mesh = gen_paperplane_mesh_color(&Vector3::new(1.0, 0.0, 0.0));
    predator_mesh.setup(pos_a, color_a, model_inst_a);
    predator_mesh.update_inst(&predator_model_inst);
    {
        predator_boid.min_speed = 10.0 * world_scale;
    }

    // tree
    let tree_center_base = Vector3::new(world_bounds.xlen() / 2.0, 0.0, world_bounds.zlen() / 2.0);
    let tree_origin_shift = Vector3::new(15.0, 0.0, -18.0);
    println!(
        "tree origin shift: {:?} -> scaled down {:?}",
        tree_origin_shift,
        tree_origin_shift / (world_bounds.zlen() / 2.0)
    );
    let tree_origin_shift_mat = Matrix4::from_translation(tree_origin_shift);
    // NOTE: all translations are based off of world_bounds.zlen() / 2.0
    // haven't figured out how to move it to the origin pre-scaled yet
    let tree_scale = Matrix4::from(Matrix3::from_value(world_bounds.xlen() / 2.0));
    let (mut tree_mesh, tree_mesh_translation) = load_tree_mesh(&Vector3::new(0.0, 1.0, 0.0), 0);
    let tree_model_inst = vec![tree_scale];
    tree_mesh.setup(pos_a, color_a, model_inst_a);
    tree_mesh.update_inst(&tree_model_inst);

    let (mut trunk_mesh, trunk_mesh_translation) = load_tree_mesh(&Vector3::new(1.0, 0.6, 0.4), 1);
    let trunk_model_inst = vec![tree_scale];
    trunk_mesh.setup(pos_a, color_a, model_inst_a);
    trunk_mesh.update_inst(&trunk_model_inst);

    println!("tree mesh translation: {:?}", tree_mesh_translation);

    let tree_origin_shift_scaled_down = Vector3::new(0.3, 0.0, -0.36);

    let tree_model_transform = vec![
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(1.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(2.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(3.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(4.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(5.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(6.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(7.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(8.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(9.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(10.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(12.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(14.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(16.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(18.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(20.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(25.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(30.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(40.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(50.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(60.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(70.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(80.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(90.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
        Matrix4::from_translation(tree_center_base)
            * Matrix4::from(Matrix3::from_value(100.0))
            * Matrix4::from_translation(tree_origin_shift_scaled_down),
    ];
    let mut tree_inst = vec![tree_model_transform[0]];

    // other models
    let cube_model_inst = vec![
        Matrix4::from_translation(world_bounds.center())
            * Matrix4::from(Matrix3::from_value(world_bounds.xlen())),
        Matrix4::from_translation(fly_bbox.center())
            * Matrix4::from(Matrix3::from_value(fly_bbox.xlen())),
    ];
    let mut cube_mesh = gen_cube_mesh(&Vector3::new(0.5, 0.5, 0.5));
    cube_mesh.setup(pos_a, color_a, model_inst_a);
    cube_mesh.update_inst(&cube_model_inst);

    let axis_model_inst = vec![
        Matrix4::from_translation(Vector3::new(0.5, 0.5, 0.5))
            * Matrix4::from(Matrix3::from_value(world_bounds.xlen() / 10.0)),
    ];
    let mut axis_mesh = gen_axis_mesh();
    axis_mesh.setup(pos_a, color_a, model_inst_a);
    axis_mesh.update_inst(&axis_model_inst);

    let mut debug_lines_inst = vec![];
    for _ in 0..shared_bs.len() {
        debug_lines_inst.push(Matrix4::identity());
    }
    let debug_lines_inst = debug_lines_inst;
    let mut debug_lines_mesh = gen_debug_line_mesh(shared_bs.len());
    debug_lines_mesh.setup(pos_a, color_a, model_inst_a);
    debug_lines_mesh.update_inst(&debug_lines_inst);

    let mut debug_octree_inst = vec![];
    for _ in 0..shared_bs.len() {
        debug_octree_inst.push(Matrix4::identity());
    }
    let mut debug_octree_mesh = gen_cube_mesh(&Vector3::new(1.0, 1.0, 0.0));
    debug_octree_mesh.setup(pos_a, color_a, model_inst_a);
    debug_octree_mesh.update_inst(&debug_octree_inst);

    let alpha_u = prog.get_unif("alpha");
    let view_u = prog.get_unif("view");
    let proj_u = prog.get_unif("proj");

    let mut horiz_view_angle = Deg(0.0f32);
    let mut vert_view_height = world_bounds.h.y * 1.5;
    let mut view_update = true;

    let mut proj_m4 = perspective(Deg(45.0), _width as f32 / _height as f32, 0.01, 1000.0);
    let eye = Point3::new(
        world_bounds.h.x * 2.0,
        world_bounds.h.y * 1.5,
        world_bounds.h.z * 2.0,
    );
    let target = Point3::from_vec(world_bounds.center());
    let mut view_m4 = Matrix4::look_at(eye, target, Vector3::unit_y());

    proj_u.upload_m4f(&proj_m4);
    view_u.upload_m4f(&view_m4);
    alpha_u.upload_1f(1.0);

    let mut tm = TimeMap::new();
    let mut frame_t = Timer::new();
    let mut section_t = Timer::new();
    let mut compute_t = Timer::new();
    let mut frame_total = 0.0;

    let tm_frame = "00.frame";
    let tm_events = "01.e";
    let tm_compute = "02.0.c";
    let tm_compute_octree_build = "02.1.obuild";
    let tm_compute_rules = "02.2.rules";
    let tm_compute_update = "02.3.u_boid";
    let tm_compute_update_inst = "02.4.u_inst";
    let tm_draw_inst = "03.draw_i";

    let mut pause = true;
    let mut debug = false;
    let mut debug_verbose = false;
    let mut enable_octree = false;
    let mut debug_octree_level = 0; // 0: no debug, 1: leaves, 2: nodes, 3: all
    let mut frame_count = 0;
    let mut tree_frame_count = 0;

    frame_t.start();
    while !window.should_close() {
        if debug_verbose {
            println!("{}: frame start", frame_count);
        }
        let tlastframe = frame_t.stop();
        frame_t.start();

        section_t.start(); // events
        if debug_verbose {
            println!("{}: glfw calling poll_events", frame_count);
        }
        glfw.poll_events();
        if debug_verbose {
            println!("{}: glfw returned from poll_events", frame_count);
        }
        for (_, event) in glfw::flush_messages(&events) {
            match event {
                glfw::WindowEvent::FramebufferSize(w, h) => {
                    unsafe {
                        gl::Viewport(0, 0, w, h);
                    }
                    proj_m4 = perspective(Deg(45.0), w as f32 / h as f32, 0.01, 1000.0);
                    proj_u.upload_m4f(&proj_m4);
                }

                glfw::WindowEvent::Key(Key::Escape, _, Action::Press, _) => {
                    window.set_should_close(true);
                }
                glfw::WindowEvent::Key(Key::P, _, Action::Press, _) => {
                    pause = !pause;
                }
                glfw::WindowEvent::Key(Key::D, _, Action::Press, _) => {
                    debug = !debug;
                }
                glfw::WindowEvent::Key(Key::V, _, Action::Press, _) => {
                    debug_verbose = !debug_verbose;
                }
                glfw::WindowEvent::Key(Key::O, _, Action::Press, mods) => {
                    if mods.contains(Modifiers::Shift) {
                        debug_octree_level = (debug_octree_level + 1) % 4;
                    } else {
                        enable_octree = !enable_octree;
                    }
                }

                glfw::WindowEvent::Scroll(xoff, yoff) => {
                    let off_epsilon = 0.15;
                    let shift = window.get_key(Key::LeftShift) == Action::Press;
                    let (xoff, yoff) = if !shift {
                        (xoff as f32, yoff as f32)
                    } else {
                        (yoff as f32, xoff as f32)
                    };

                    if debug_verbose {
                        println!(
                            "{}: {} scroll: x: {}, y: {}",
                            frame_count,
                            if shift { "reverse" } else { "" },
                            xoff,
                            yoff
                        );
                    }

                    if xoff.abs() > off_epsilon {
                        horiz_view_angle = horiz_view_angle + Deg(2.0 * xoff);
                        view_update = true;
                    }

                    if yoff.abs() > off_epsilon {
                        vert_view_height =
                            vert_view_height + 0.5 * (world_bounds.xlen() / 10.0) * yoff;
                        view_update = true;
                    }
                }

                glfw::WindowEvent::Key(Key::Left, _, Action::Press, mode)
                | glfw::WindowEvent::Key(Key::Left, _, Action::Repeat, mode) => {
                    horiz_view_angle += if mode.contains(Modifiers::Shift) {
                        Deg(5.0)
                    } else {
                        Deg(1.0)
                    };
                    view_update = true;
                }
                glfw::WindowEvent::Key(Key::Right, _, Action::Press, mode)
                | glfw::WindowEvent::Key(Key::Right, _, Action::Repeat, mode) => {
                    horiz_view_angle -= if mode.contains(Modifiers::Shift) {
                        Deg(5.0)
                    } else {
                        Deg(1.0)
                    };
                    view_update = true;
                }

                glfw::WindowEvent::Key(Key::Up, _, Action::Press, _)
                | glfw::WindowEvent::Key(Key::Up, _, Action::Repeat, _) => {
                    vert_view_height = vert_view_height + 0.5 * (world_bounds.xlen() / 10.0);
                    view_update = true;
                }
                glfw::WindowEvent::Key(Key::Down, _, Action::Press, _)
                | glfw::WindowEvent::Key(Key::Down, _, Action::Repeat, _) => {
                    vert_view_height = vert_view_height - 0.5 * (world_bounds.xlen() / 10.0);
                    view_update = true;
                }

                glfw::WindowEvent::Key(Key::Equal, _, Action::Press, mods)
                | glfw::WindowEvent::Key(Key::Equal, _, Action::Repeat, mods) => {
                    let num_add = if mods.contains(Modifiers::Shift) {
                        10
                    } else {
                        1
                    };
                    unsafe {
                        let msmi: &mut Vec<Matrix4<f32>> =
                            mem::transmute(&*shared_model_inst.clone());
                        let mbs: &mut Vec<Boid> = mem::transmute(&*shared_bs.clone());

                        for _ in 0..num_add {
                            let b = Boid::random_new(&fly_bbox);
                            msmi.push(b.model() * model_default_scale_mat);
                            mbs.push(b);
                        }
                    }
                    num_boids = shared_bs.len();
                    work_size = num_boids / threads;
                    resize_debug_line_mesh(&mut debug_lines_mesh, num_boids);
                    if debug_verbose {
                        println!(
                            "{}: pushed new boid: num: {}, work_size: {}",
                            frame_count, num_boids, work_size
                        );
                    }
                }

                glfw::WindowEvent::Key(Key::Minus, _, Action::Press, mods)
                | glfw::WindowEvent::Key(Key::Minus, _, Action::Repeat, mods) => {
                    if shared_bs.len() > 1 {
                        let num_remove = if mods.contains(Modifiers::Shift) {
                            10
                        } else {
                            1
                        };
                        let num_remove = if num_remove > shared_bs.len() - 1 {
                            shared_bs.len() - 1
                        } else {
                            num_remove
                        };
                        unsafe {
                            let mbs: &mut Vec<Boid> = mem::transmute(&*shared_bs.clone());
                            let msmi: &mut Vec<Matrix4<f32>> =
                                mem::transmute(&*shared_model_inst.clone());
                            for _ in 0..num_remove {
                                mbs.pop();
                                msmi.pop();
                            }
                        }
                        num_boids = shared_bs.len();
                        work_size = num_boids / threads;
                        resize_debug_line_mesh(&mut debug_lines_mesh, num_boids);
                        if debug_verbose {
                            println!(
                                "{}: removed boid: num: {}, work_size: {}",
                                frame_count, num_boids, work_size
                            );
                        }
                    }
                }

                _ => {}
            }
        }

        if debug_verbose {
            println!("{}: event parse", frame_count);
        }

        if view_update {
            view_update = false;

            let target2d = Vector2::new(target.x, target.z);
            let eye2d = Vector2::new(eye.x, eye.z);
            let dir = eye2d - target2d;

            let rot2d: Basis2<f32> = Rotation2::<f32>::from_angle(horiz_view_angle);
            let new_dir = rot2d.rotate_vector(dir);

            let new_eye2d = target2d + new_dir;
            let new_eye3d = Point3::new(new_eye2d.x, vert_view_height, new_eye2d.y);
            view_m4 = Matrix4::look_at(new_eye3d, target, Vector3::unit_y());
            view_u.upload_m4f(&view_m4);
        }
        tm.update(tm_events, section_t.stop());
        if debug_verbose {
            println!("{}: event parse + view update", frame_count);
        }

        unsafe {
            gl::ClearColor(0.0, 0.0, 0.0, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        compute_t.start();
        if !pause {
            if debug_verbose {
                println!("{}: compute start", frame_count);
            }
            let dt = (tlastframe as f32) * 1e-3; // convert from ms to sec

            // octree build step
            if enable_octree {
                section_t.start();
                // TODO: need to figure out a non-hack for read/write here
                // I could use a RWLock like before, but that won't help when
                // it comes to parallel builds
                let octree: &mut Octree = unsafe { mem::transmute(&*shared_octree) };
                if debug_verbose {
                    println!("{}: octree reset", frame_count);
                }
                octree.reset(world_bounds);

                let bs = shared_bs.clone(); // just need read access

                if debug_verbose {
                    println!("{}: octree insert", frame_count);
                }
                octree.insert(&*bs); // FIXME: overflow in insert logic
                if debug_verbose {
                    println!("{}: octree update", frame_count);
                }
                octree.update(&*bs);

                // TODO: add octree debug drawing using cubes
                if debug_octree_level > 0 {
                    debug_octree_inst.clear();
                    debug_octree_inst.reserve(octree.pool.len());

                    for o in octree.pool.iter() {
                        let draw = match o.state {
                            octree::OctnodeState::Node => debug_octree_level >= 2,
                            octree::OctnodeState::Leaf => {
                                debug_octree_level == 1 || debug_octree_level == 3
                            }
                            _ => false,
                        };

                        if draw {
                            debug_octree_inst.push(
                                Matrix4::from_translation(o.bbox.center())
                                    * Matrix4::from(Matrix3::from_value(o.bbox.xlen())),
                            );
                        }
                    }

                    debug_octree_mesh.update_inst(&debug_octree_inst);
                }

                if debug_verbose {
                    println!("{}: octree fin", frame_count);
                }
                tm.update(tm_compute_octree_build, section_t.stop());
            }

            if debug_verbose {
                println!("{}: sim start: {} boids", frame_count, shared_bs.len());
            }
            section_t.start();

            // move predator
            {
                let bounds_v = bounds_v(&predator_boid, &fly_bbox) * 20.0; // weights[4]
                predator_boid.acc = bounds_v;
                predator_boid.update(dt, world_scale);

                unsafe {
                    let m = &predator_model_inst[0];
                    let m: &mut Matrix4<f32> = mem::transmute(m);
                    *m = predator_boid.model() * pred_model_default_scale_mat;
                }
            }

            let predator_pos = predator_boid.pos;

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
                    let work_size = if tid == threads - 1 {
                        work_size + num_boids % threads
                    } else {
                        work_size
                    };

                    let mut rules = vec![
                        Vector3::zero(),
                        Vector3::zero(),
                        Vector3::zero(),
                        Vector3::zero(),
                        Vector3::zero(),
                    ];
                    for i in start_id..(start_id + work_size) {
                        //rules[0] = Vector3::zero();
                        {
                            let b = &bs[i];
                            let disp = predator_pos - b.pos;
                            let dist2 = disp.magnitude2();

                            if dist2 < look_radius2 {
                                rules[0] = -disp;
                                rules[0] = rules[0] * weights[0];
                            }
                        }

                        {
                            let (r1, r2, r3) = if enable_octree {
                                calc_rules_octree(
                                    &bs,
                                    i,
                                    &*octree,
                                    look_radius,
                                    look_radius2,
                                    collide_radius2,
                                )
                            } else {
                                calc_rules(&bs, i, look_radius2, collide_radius2)
                            };
                            rules[1] = r1 * weights[1];
                            rules[2] = r2 * weights[2];
                            rules[3] = r3 * weights[3];

                            rules[4] = bounds_v(&bs[i], &fly_bbox) * weights[4];
                        }

                        let mut mag = 0.0;
                        let mut acc = Vector3::zero();

                        for r in rules.iter() {
                            let m = r.magnitude();

                            // TODO: change this to epsilon
                            if m == 0.0 {
                                continue;
                            }

                            if mag + m > max_mag {
                                // rebalance last rule
                                let r = r * ((max_mag - mag) / m);
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
            if debug_verbose {
                println!("{}: sim stop", frame_count);
            }

            if debug_verbose {
                println!("{}: update start", frame_count);
            }
            section_t.start();
            for tid in 0..threads {
                let thread_tx = tx.clone();
                let thread_bs = shared_bs.clone();
                let thread_model_inst = shared_model_inst.clone();

                thread::spawn(move || {
                    let bs = thread_bs;
                    let model_inst = thread_model_inst;

                    let start_id = tid * work_size;
                    let work_size = if tid == threads - 1 {
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
            if debug_verbose {
                println!("{}: update stop", frame_count);
            }

            if debug {
                let bs = shared_bs.clone();
                for i in 0..bs.len() {
                    let b = &bs[i];

                    let i = i * 5;

                    let pos = b.pos;
                    let pos_vel = pos + b.vel;

                    // boid origin
                    let ip = i * debug_lines_mesh.vertex_size;
                    debug_lines_mesh.vertices[ip] = pos.x;
                    debug_lines_mesh.vertices[ip + 1] = pos.y;
                    debug_lines_mesh.vertices[ip + 2] = pos.z;

                    // velocity vector
                    let iv = (i + 1) * debug_lines_mesh.vertex_size;
                    debug_lines_mesh.vertices[iv] = pos_vel.x;
                    debug_lines_mesh.vertices[iv + 1] = pos_vel.y;
                    debug_lines_mesh.vertices[iv + 2] = pos_vel.z;

                    // lookat vectors
                    let dir = b.vel;
                    let up = Vector3::unit_y();

                    let dir = dir.normalize();
                    let pos_dir = pos + dir;

                    let side = up.cross(dir).normalize();
                    let pos_side = pos + side;

                    let up = dir.cross(side).normalize();
                    let pos_up = pos + up;

                    // lookat forward/dir vector
                    let ifv = (i + 2) * debug_lines_mesh.vertex_size;
                    debug_lines_mesh.vertices[ifv] = pos_dir.x;
                    debug_lines_mesh.vertices[ifv + 1] = pos_dir.y;
                    debug_lines_mesh.vertices[ifv + 2] = pos_dir.z;

                    // lookat side vector
                    let isv = (i + 3) * debug_lines_mesh.vertex_size;
                    debug_lines_mesh.vertices[isv] = pos_side.x;
                    debug_lines_mesh.vertices[isv + 1] = pos_side.y;
                    debug_lines_mesh.vertices[isv + 2] = pos_side.z;

                    // lookat up vector
                    let iuv = (i + 4) * debug_lines_mesh.vertex_size;
                    debug_lines_mesh.vertices[iuv] = pos_up.x;
                    debug_lines_mesh.vertices[iuv + 1] = pos_up.y;
                    debug_lines_mesh.vertices[iuv + 2] = pos_up.z;
                }

                debug_lines_mesh.update_verts();
            }

            if debug_verbose {
                println!("{}: plane update inst start", frame_count);
            }
            section_t.start();
            plane_mesh.update_inst(&shared_model_inst);
            predator_mesh.update_inst(&predator_model_inst);
            tm.update(tm_compute_update_inst, section_t.stop());
            if debug_verbose {
                println!("{}: plane update inst fin", frame_count);
            }
        }

        tm.update(tm_compute, compute_t.stop());

        // draw planes
        section_t.start();
        plane_mesh.draw_inst(shared_model_inst.len() as GLint);
        tm.update(tm_draw_inst, section_t.stop());

        predator_mesh.draw_inst(predator_model_inst.len() as GLint);

        {
            // loop through tree positions
            if frame_count % 10 == 0 {
                tree_frame_count = (tree_frame_count + 1) % tree_model_transform.len();
                println!("tree frame: {}", tree_frame_count);
                tree_inst = vec![tree_model_transform[tree_frame_count]];
                tree_mesh.update_inst(&tree_inst);
                trunk_mesh.update_inst(&tree_inst);
            }
            tree_mesh.draw_inst(tree_inst.len() as GLint);
            trunk_mesh.draw_inst(tree_inst.len() as GLint);
        }

        cube_mesh.draw_inst(cube_model_inst.len() as GLint);
        axis_mesh.draw_inst(axis_model_inst.len() as GLint);
        if debug {
            debug_lines_mesh.draw_inst(debug_lines_inst.len() as GLint);
        }
        if enable_octree && (debug_octree_level > 0) {
            debug_octree_mesh.draw_inst(debug_octree_inst.len() as GLint);
        }

        window.swap_buffers();
        frame_count += 1;
        tm.update(tm_frame, frame_t.stop());
        frame_total += frame_t.elapsedms();

        if frame_count % 60 == 0 {
            tm.avg(60);
            let frame_avg = frame_total / 60.0;
            if !pause {
                println!("{:.2} // {}", frame_avg, tm);
            }
            frame_total = 0.0;
            tm.clear();
        }
        if debug_verbose {
            println!("{}: frame end", frame_count);
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
        0.0, 0.0, 1.0, 1.0, 1.0, 1.0f32, // front / nose
        0.75, 0.0, -1.0, 1.0, 1.0, 1.0, // left wing / 'port'
        -0.75, 0.0, -1.0, 1.0, 1.0, 1.0, // right wing / 'starboard
        0.0, 0.0, -1.0, 1.0, 1.0, 1.0, // back midpoint between wings
        0.0, -0.4, -1.0, 1.0, 1.0, 1.0, // back bottom fin
    ];
    let vertex_size = 6;

    // triangles
    // let elements = vec![
    //     0u32, 1, 3,
    //     0, 3, 2,
    //     0, 4, 3,
    // ];

    // lines
    let elements = vec![0u32, 1, 2, 0, 3, 4];

    Mesh::new("paperplane", vertices, elements, vertex_size, gl::LINE_LOOP)
}

fn gen_paperplane_mesh_color(color: &Vector3<f32>) -> Mesh {
    // z+ is front
    // y+ is top
    let vertices = vec![
        // vertex        // color
        0.0, 0.0, 1.0, color.x, color.y, color.z, // front / nose
        0.75, 0.0, -1.0, color.x, color.y, color.z, // left wing / 'port'
        -0.75, 0.0, -1.0, color.x, color.y, color.z, // right wing / 'starboard
        0.0, 0.0, -1.0, color.x, color.y, color.z, // back midpoint between wings
        0.0, -0.4, -1.0, color.x, color.y, color.z, // back bottom fin
    ];
    let vertex_size = 6;

    // triangles
    // let elements = vec![
    //     0u32, 1, 3,
    //     0, 3, 2,
    //     0, 4, 3,
    // ];

    // lines
    let elements = vec![0u32, 1, 2, 0, 3, 4];

    Mesh::new("paperplane", vertices, elements, vertex_size, gl::LINE_LOOP)
}
fn gen_square_mesh() -> Mesh {
    let vertices = vec![
        // vertex        // color
        0.0, 0.0, 0.0, 1.0, 1.0, 1.0f32, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0f32, 1.0, 1.0, 0.0, 1.0, 1.0,
        1.0f32, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = vec![0u32, 1, 2, 3];
    Mesh::new("square", vertices, elements, vertex_size, gl::LINE_LOOP)
}

fn gen_axis_mesh() -> Mesh {
    let vertices = vec![
        // vertex        // color
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0f32, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0f32, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0f32, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0f32, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0f32, 0.0, 0.0, 1.0,
        0.0, 0.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = vec![0u32, 1, 2, 3, 4, 5];

    Mesh::new("axis", vertices, elements, vertex_size, gl::LINES)
}

fn gen_cube_mesh(color: &Vector3<f32>) -> Mesh {
    let vertices = vec![
        // vertex        // color
        -0.5, -0.5, -0.5, color.x, color.y, color.z, -0.5, -0.5, 0.5, color.x, color.y, color.z,
        0.5, -0.5, 0.5, color.x, color.y, color.z, 0.5, -0.5, -0.5, color.x, color.y, color.z,
        -0.5, 0.5, -0.5, color.x, color.y, color.z, -0.5, 0.5, 0.5, color.x, color.y, color.z, 0.5,
        0.5, 0.5, color.x, color.y, color.z, 0.5, 0.5, -0.5, color.x, color.y, color.z,
    ];

    let vertex_size = 6;
    let elements = vec![0u32, 1, 2, 3, 0, 4, 5, 6, 7, 4, 5, 1, 2, 6, 7, 3];

    Mesh::new("cube", vertices, elements, vertex_size, gl::LINE_LOOP)
}

fn load_tree_mesh(color: &Vector3<f32>, item: u32) -> (Mesh, Matrix4<f32>) {
    // let tree_blocks_file = Path::new("data/tree_blocks.gltf");
    let tree_blocks_file = Path::new("data/tree_oak.gltf");
    let (document, buffers, images) = gltf::import(tree_blocks_file).unwrap();

    let mut vertices: Vec<f32> = vec![];
    let mut elements: Vec<u32> = vec![];

    let mut count = 0;

    for mesh in document.meshes() {
        for primitive in mesh.primitives() {
            if count != item {
                count += 1;
                continue;
            }
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));

            if let Some(iter) = reader.read_positions() {
                for vert_pos in iter {
                    vertices.extend_from_slice(&vert_pos);
                    vertices.push(color[0]);
                    vertices.push(color[1]);
                    vertices.push(color[2]);
                }
            }
            if let Some(readindices) = reader.read_indices() {
                println!("got read indices");
                let iter = readindices.into_u32();
                for idx in iter {
                    elements.push(idx);
                }
            }
            break;
        }
        break;
    }

    println!(
        "tree mesh: num verts: [{} / 6 = {}], num elements: {}",
        vertices.len(),
        vertices.len() / 6,
        elements.len()
    );

    let vertex_size = 6;
    let mesh = Mesh::new("tree", vertices, elements, vertex_size, gl::LINE_LOOP);

    let mut translation = <Matrix4<f32> as cgmath::One>::one();

    if let Some(scene) = document.default_scene() {
        println!("default scene provided");
        let mut nodes = scene.nodes();

        let node = nodes.next().unwrap();
        let t = node.transform().matrix();

        translation = Matrix4::new(
            t[0][0], t[0][1], t[0][2], -t[0][3], // row 0
            t[1][0], t[1][1], t[1][2], -t[1][3], // row 0
            t[2][0], t[2][1], t[2][2], -t[2][3], // row 0
            t[3][0], t[3][1], t[3][2], t[3][3], // row 0
        );
    }

    (mesh, translation)
}

fn gen_debug_line_mesh(num_boids: usize) -> Mesh {
    let vertex_size = 6;
    let vertices = vec![];
    let elements = vec![];

    let mut m = Mesh::new("debug_lines", vertices, elements, vertex_size, gl::LINES);
    resize_debug_line_mesh(&mut m, num_boids);
    m
}

fn resize_debug_line_mesh(mesh: &mut Mesh, num_boids: usize) {
    let num_verts = mesh.vertex_size * 5 * num_boids;
    let num_elem = 2 * 4 * num_boids;

    // println!("resize line mesh: num boids: {}, verts: {} -> {}, elem: {} -> {}", num_boids, mesh.vertices.len(), num_verts, mesh.elements.len(), num_elem);

    if mesh.vertices.len() > num_verts {
        // shrink
        mesh.vertices.truncate(num_verts);
        mesh.elements.truncate(num_elem);
    // println!("resize line mesh: shrinking, new verts len: {}, elem len: {}", mesh.vertices.len(), mesh.elements.len());
    } else {
        // grow
        let old_num_boids = mesh.vertices.len() / (mesh.vertex_size * 5);
        // println!("resize line mesh: growing, old boids len: {}", old_num_boids);

        for _ in old_num_boids..num_boids {
            // origin of the boid
            mesh.vertices.push(0.0); // vertex
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0);
            mesh.vertices.push(1.0); // color
            mesh.vertices.push(1.0);
            mesh.vertices.push(1.0);

            // velocity vector
            mesh.vertices.push(0.0); // vertex
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0);
            mesh.vertices.push(1.0); // color
            mesh.vertices.push(1.0);
            mesh.vertices.push(0.0);

            // lookat forward vector
            mesh.vertices.push(0.0); // vertex
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0); // color
            mesh.vertices.push(0.0);
            mesh.vertices.push(1.0);

            // lookat side vector
            mesh.vertices.push(0.0); // vertex
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0);
            mesh.vertices.push(1.0); // color
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0);

            // lookat up vector
            mesh.vertices.push(0.0); // vertex
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0);
            mesh.vertices.push(0.0); // color
            mesh.vertices.push(1.0);
            mesh.vertices.push(0.0);
        }

        for i in old_num_boids..num_boids {
            let i = i as u32 * 5;
            // velocity
            mesh.elements.push(i);
            mesh.elements.push(i + 1);

            // lookat forward
            mesh.elements.push(i);
            mesh.elements.push(i + 2);

            // lookat side
            mesh.elements.push(i);
            mesh.elements.push(i + 3);

            // lookat up
            mesh.elements.push(i);
            mesh.elements.push(i + 4);
        }

        // println!("resize line mesh: growing, new verts len: {}, elem len: {}", mesh.vertices.len(), mesh.elements.len());
    }

    mesh.update_verts();
    mesh.update_elem();
}

fn bounds_v(b: &Boid, bbox: &AABB) -> Vector3<f32> {
    let mut bounds = Vector3::zero();

    bounds.x = if b.pos.x > bbox.h.x {
        -1.0
    } else if b.pos.x < bbox.l.x {
        1.0
    } else {
        0.0
    };

    bounds.y = if b.pos.y > bbox.h.y {
        -1.0
    } else if b.pos.y < bbox.l.y {
        1.0
    } else {
        0.0
    };

    bounds.z = if b.pos.z > bbox.h.z {
        -1.0
    } else if b.pos.z < bbox.l.z {
        1.0
    } else {
        0.0
    };

    if bounds != Vector3::zero() {
        bounds.normalize()
    } else {
        bounds
    }
}

//
// Original algorithm
//

// returns normalized results
fn calc_rules(
    bs: &Vec<Boid>,
    bi: usize,
    look_radius2: f32,
    collide_radius2: f32,
) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    let b = &bs[bi];
    let mut r1 = Vector3::zero();
    let mut r2 = Vector3::zero();
    let mut r3 = Vector3::zero();
    let mut neighbors = 0u32;
    let mut colliders = 0u32;

    for j in 0..bs.len() {
        if bi == j {
            continue;
        }

        let o = &bs[j];

        let disp = o.pos - b.pos;
        let dist2 = disp.magnitude2();

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
        r2 = r2 / (neighbors as f32) - b.pos;
        r2 = r2.normalize();
        r3 = r3.normalize();
    }
    if colliders > 0 {
        r1 = r1.normalize();
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

fn calc_rules_octree(
    bs: &Vec<Boid>,
    boid_id: usize,
    octree: &Octree,
    look_radius: f32,
    look_radius2: f32,
    collide_radius2: f32,
) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
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

    if tc.octree.root != usize::MAX {
        traverse_octree(&tc, &mut tr, tc.octree.root);
    }

    if tr.neighbors > 0 {
        tr.r2 = (tr.r2 / (tr.neighbors as f32) - b.pos).normalize();
        tr.r3 = tr.r3.normalize();
    }
    if tr.colliders > 0 {
        tr.r1 = tr.r1.normalize();
    }

    //println!("leaves: {}, nodes: {}, small node: {}", tr.leaves, tr.nodes, tr.small_nodes);
    (tr.r1, tr.r2, tr.r3)
}

fn traverse_octree(tc: &TraversalConst, tr: &mut TraversalRecur, curr: usize) {
    let o = tc.octree.get_node(curr);
    let dv = o.c - tc.b.pos;
    let d = dv.magnitude();

    if o.is_leaf() {
        if d < 1e-6 {
            return;
        } // skip self

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
            if child_id != usize::MAX {
                traverse_octree(tc, tr, child_id);
            }
        }
    }
}

fn single_interact(
    tc: &TraversalConst,
    tr: &mut TraversalRecur,
    o: &Octnode,
    dv: &Vector3<f32>,
    d: f32,
) {
    let d2 = d * d;

    if d2 < tc.look_radius2 {
        tr.neighbors += 1;

        tr.r2 = tr.r2 + o.c;
        tr.r3 = tr.r3 + o.v;

        if d2 < tc.collide_radius2 {
            tr.colliders += 1;
            tr.r1 = tr.r1 - dv / d2;
        }
    }
}
