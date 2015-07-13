extern crate gl;
extern crate glfw;
extern crate cgmath;
extern crate time;
extern crate rand;

use time::precise_time_ns;
use gl::types::*;
use glfw::{Action, Context, Key};
use cgmath::*;
use shaders::{Shader, Program};
use mesh::Mesh;
use timer::{Timer, TimeMap};
use aabb::AABB;
use boids::Boid;

mod shaders;
mod mesh;
mod timer;
mod aabb;
mod boids;

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

    println!("creating shaders");
    let shaders_v = vec![
        Shader::from_str(gl::VERTEX_SHADER, &VS_SRC),
        Shader::from_str(gl::FRAGMENT_SHADER, &FS_SRC),
    ];
    gl_error_str("shaders created");

    println!("creating program");
    let prog = Program::new(&shaders_v);
    gl_error_str("program created");

    // config variables
    let world_bounds = AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(5.0, 5.0, 5.0));
    let world_scale = 0.1;
    let look_radius = 30.0 * world_scale;
    let look_radius2 = look_radius * look_radius;
    let collide_radius = 8.0 * world_scale;
    let collide_radius2 = collide_radius * collide_radius;
    let max_mag = 100.0;

    let default_weights  = vec![
        30.0, // avoid obstacles
        12.0, // collision avoidance
        8.0,  // flock centering
        9.0,  // match velocity
        20.0, // bounds push
    ];
    let weights =  default_weights; // opt_weights(&args, "weights", default_weights, 5);

    let mut fly_bbox = world_bounds.clone();
    fly_bbox.scale_center(0.8);
    let fly_bbox = fly_bbox;

    let num_boids = 10;
    println!("generating {} boids", num_boids);

    let mut bs = Vec::with_capacity(num_boids);
    for _ in 0..num_boids {
        bs.push(Boid::random_new(&world_bounds))
    }

    let model_default_scale_mat = Matrix4::from(Matrix3::from_value(world_scale));

    // convert positions into model matrices
    let mut model_inst = Vec::with_capacity(bs.len());
    for b in bs.iter() {
        model_inst.push(b.model() * model_default_scale_mat);
    }


    println!("use program");
    prog.use_prog();

    let pos_a = prog.get_attrib("position") as GLuint;
    let color_a = prog.get_attrib("color") as GLuint;
    let model_inst_a = prog.get_attrib("model_inst") as GLuint;

    let mut plane_mesh = gen_paperplane_mesh();
    plane_mesh.setup(pos_a, color_a, model_inst_a);
    plane_mesh.update_inst(&model_inst);


    unsafe {
        gl::Enable(gl::DEPTH_TEST);
    }

    println!("setting up uniforms");

    let alpha_u = prog.get_unif("alpha");
    let view_u = prog.get_unif("view");
    let proj_u = prog.get_unif("proj");

    let mut proj_m4 = perspective(deg(45.0), 800.0 / 600.0, 1.0, 10.0);
    let view_m4 = Matrix4::look_at(&Point3::new(8.0, 8.0, 0.0), &Point3::from_vec(&world_bounds.center()), &Vector3::new(0.0, 1.0, 0.0));

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
    let tm_compute_shared_mat = "02.1.shared_mat";
    let tm_compute_vec_build = "02.2.vec_build";
    let tm_compute_update_inst = "02.3.update_inst";
    let tm_draw_inst ="03.draw_inst";

    let mut pause = true;
    let mut frame_count = 0;
    let mut rot_angle = 0.0;

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
                    proj_m4 = perspective(deg(45.0), w as f32 / h as f32, 1.0, 10.0);
                    proj_u.upload_m4f(&proj_m4);
                }
                _ => {}
            }
        }
        tm.update(tm_events, section_t.stop());

        unsafe {
            gl::ClearColor(0.0, 0.0, 0.0, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        compute_t.start();
        if !pause {
            section_t.start();
            // starting with brute force boids algorithm

            let dt = (tlastframe as f32) * 1e-3; // convert from ms to sec

            for bi in 0..bs.len() {
                let mut rules: Vec<Vector3<f32>> = vec![Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero(), Vector3::zero()];

                {
                    let (r1, r2, r3) = calc_rules(&bs, bi, look_radius2, collide_radius2);
                    rules[1] = r1.mul_s(weights[1]);
                    rules[2] = r2.mul_s(weights[2]);
                    rules[3] = r3.mul_s(weights[3]);

                    rules[4] = bounds_v(&bs[bi], &fly_bbox).mul_s(weights[4]);
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

                {
                    let b = &mut bs[bi];
                    b.acc = acc;
                    b.update(dt, world_scale);
                }
            }


            tm.update(tm_compute_shared_mat, section_t.stop());

            section_t.start();
            model_inst.clear();
            for b in bs.iter() {
                model_inst.push(b.model() * model_default_scale_mat);
            }
            tm.update(tm_compute_vec_build, section_t.stop());

            section_t.start();
            plane_mesh.update_inst(&model_inst);
            tm.update(tm_compute_update_inst, section_t.stop());
        }
        tm.update(tm_compute, compute_t.stop());

        // draw planes
        section_t.start();
        plane_mesh.draw_inst(model_inst.len() as GLint);
        tm.update(tm_draw_inst, section_t.stop());

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
        0.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  0.0,  0.0, 1.0, 0.0, 0.0f32,
        0.0,  1.0,  0.0, 0.0, 1.0, 0.0f32,
        0.0,  0.0,  1.0, 0.0, 0.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = vec![ 0u32, 1, 0, 2, 0, 3 ];

    Mesh::new("axis", vertices, elements, vertex_size)
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
