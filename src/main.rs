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

mod shaders;
mod mesh;
mod timer;

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
    gl_error();

    println!("creating program");
    let prog = Program::new(&shaders_v);
    gl_error();

    //
    // meshes
    //
    /* // axis [x/y/z marker]
    let vertices = [
        // vertex        // color
        0.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  0.0,  0.0, 1.0, 0.0, 0.0f32,
        0.0,  1.0,  0.0, 0.0, 1.0, 0.0f32,
        0.0,  0.0,  1.0, 0.0, 0.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = [ 0u32, 1, 0, 2, 0, 3 ];
    // */

    /* // squares
    let vertices = [
        // vertex        // color
        0.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  1.0,  0.0, 1.0, 1.0, 1.0f32,
        0.0,  1.0,  0.0, 1.0, 1.0, 1.0f32,
    ];
    let vertex_size = 6;
    let elements = [ 0u32, 1, 1, 2, 2, 3, 3, 0 ];
    // */

    // paperplane
    let vertices = vec![
        // vertex        // color
        0.0,  0.0,  1.0, 1.0, 1.0, 1.0f32, // front / nose
        0.75, 0.0, -1.0, 1.0, 1.0, 1.0,    // left wing / 'port'
       -0.75, 0.0, -1.0, 1.0, 1.0, 1.0,    // right wing / 'starboard
        0.0,  0.0, -1.0, 1.0, 1.0, 1.0,    // back midpoint between wings
        0.0, -0.4, -1.0, 1.0, 1.0, 1.0,    // back bottom fin
    ];
    let vertex_size = 6;

    // triangle
    let elements = vec![
        0u32, 1, 3,
        0, 3, 2,
        0, 4, 3,
    ];

    // lines
    let elements = vec![
        0u32, 1, 2, 0, 3, 4,
    ];


    println!("generating model instance matrices");
    let num_inst = 1000;
    let mut model_positions = Vec::with_capacity(num_inst);
    {
        use rand::Rng;
        let mut rand = rand::weak_rng();

        for _ in 0..num_inst {
            model_positions.push(Vector3::new(
                    rand.next_f32() * 5.0 - 2.5,
                    rand.next_f32() * 5.0 - 2.5,
                    rand.next_f32() * 5.0 - 2.5,
                    ));
        }
    }

    let model_default_scale_mat = Matrix4::from(Matrix3::from_value(0.1));

    // convert positions into model matrices
    let mut model_inst = Vec::with_capacity(model_positions.len());
    for p in model_positions.iter() {
        model_inst.push(Matrix4::from_translation(p) * model_default_scale_mat);
    }


    println!("use program");
    prog.use_prog();

    let pos_a = prog.get_attrib("position") as GLuint;
    let color_a = prog.get_attrib("color") as GLuint;
    let model_inst_a = prog.get_attrib("model_inst") as GLuint;

    let mut plane_mesh = Mesh::new("paperplane", vertices, elements, vertex_size);
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
    let view_m4 = Matrix4::look_at(&Point3::new(5.0, 5.0, 2.0), &Point3::new(0.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));

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
    let tm_clear = "02.clear";
    let tm_compute = "03.0.compute";
    let tm_compute_shared_mat = "03.1.shared_mat";
    let tm_compute_vec_build = "03.2.vec_build";
    let tm_compute_update_inst = "03.3.update_inst";
    let tm_draw_inst ="04.draw_inst";

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

        section_t.start();
        unsafe {
            gl::ClearColor(0.0, 0.0, 0.0, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }
        tm.update(tm_clear, section_t.stop());

        compute_t.start();
        if !pause {
            section_t.start();
            // calculate rotation based on time, update each model matrix
            rot_angle += 180.0 * (tlastframe as f32 * 1e-3);

            let rot180 = Basis3::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), deg(rot_angle).into());
            let rot180_m4 = Matrix4::from(*rot180.as_ref());

            let shared_model = rot180_m4 * model_default_scale_mat;
            tm.update(tm_compute_shared_mat, section_t.stop());

            section_t.start();
            model_inst.clear();
            for p in model_positions.iter() {
                model_inst.push(Matrix4::from_translation(p) * shared_model);
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
