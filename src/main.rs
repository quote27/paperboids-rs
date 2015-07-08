extern crate gl;
extern crate glfw;
extern crate cgmath;
extern crate time;
extern crate image;

use time::precise_time_s;
use gl::types::*;
use glfw::{Action, Context, Key};
use image::GenericImage;
use cgmath::*;
use std::mem;
use std::ptr;
use shaders::{Shader, Program, Uniform};

mod shaders;

static VS_SRC: &'static str = "
#version 150 core
in vec3 position;
in vec3 color;

out vec3 o_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform vec3 override_color;

void main() {
    o_color = color * override_color;
    gl_Position = proj * view * model * vec4(position, 1.0);
}";

static FS_SRC: &'static str = "
#version 150 core
in vec3 o_color;
out vec4 out_color;

uniform float alpha;

void main() {
    out_color = vec4(o_color, 1.0) * alpha;
}";

fn main() {
    println!("open.gl tutorial begin");
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    glfw.window_hint(glfw::WindowHint::ContextVersion(3, 2));
    glfw.window_hint(glfw::WindowHint::OpenGlForwardCompat(true));
    glfw.window_hint(glfw::WindowHint::OpenGlProfile(glfw::OpenGlProfileHint::Core));
    glfw.window_hint(glfw::WindowHint::Resizable(false));

    let (mut window, events) = glfw.create_window(300, 300, "open.gl tutorial", glfw::WindowMode::Windowed)
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

    println!("creating vertex array object (vao)");
    let mut vao = 0;
    unsafe {
        gl::GenVertexArrays(1, &mut vao);
        gl::BindVertexArray(vao);
    }
    gl_error();


    let vertices = [
        // vertex        // color
        0.0,  0.0,  1.0, 1.0, 1.0, 1.0f32,
        0.75, 0.0, -1.0, 1.0, 1.0, 1.0,
       -0.75, 0.0, -1.0, 1.0, 1.0, 1.0,
        0.0,  0.0, -1.0, 1.0, 1.0, 1.0,
        0.0, -0.4, -1.0, 1.0, 1.0, 1.0,
    ];
    let vertex_size = 6;

    let elements = [
        0, 1, 3u32,
        0, 3, 2,
        0, 4, 3,
    ];

    /*
    let vertices = [
        // vertex        // color
        0.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  0.0,  0.0, 1.0, 0.0, 0.0f32,
        0.0,  1.0,  0.0, 0.0, 1.0, 0.0f32,
        0.0,  0.0,  1.0, 0.0, 0.0, 1.0f32,
    ];
    let vertex_size = 6;

    let elements = [
        0, 1u32,
        0, 2,
        0, 3,
    ];
    */


    // upload data to card
    println!("vertices: creating vertex buffer object (vbo)");
    let mut vbo = 0;
    unsafe {
        gl::GenBuffers(1, &mut vbo);
        gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
        gl::BufferData(gl::ARRAY_BUFFER, (vertices.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&vertices[0]), gl::STATIC_DRAW);
    }
    gl_error();

    // upload data to card
    println!("elements: creating vertex buffer object (ebo)");
    let mut ebo = 0;
    unsafe {
        gl::GenBuffers(1, &mut ebo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, (elements.len() * mem::size_of::<u32>()) as GLsizeiptr, mem::transmute(&elements[0]), gl::STATIC_DRAW);
    }
    gl_error();


    prog.use_prog();

    let pos_attr = prog.get_attrib("position");
    println!("position: attribute: {}", pos_attr);
    gl_error();

    println!("position: setting vertex attribute pointer and enabling enabling vertex attrib array");
    unsafe {
        let pos_attr_u = pos_attr as GLuint;
        println!("  enable vertex attrib array");
        gl::EnableVertexAttribArray(pos_attr_u);
        gl_error();
        println!("  vertex attrib pointer");
        gl::VertexAttribPointer(pos_attr_u, 3, gl::FLOAT, gl::FALSE, (vertex_size * mem::size_of::<f32>()) as GLint, ptr::null());
        gl_error();
    }

    let color_attr = prog.get_attrib("color");
    println!("color: attribute: {}", pos_attr);
    gl_error();

    println!("color: setting vertex attribute pointer and enabling enabling vertex attrib array");
    unsafe {
        let color_attr_u = color_attr as GLuint;
        println!("  enable vertex attrib array");
        gl::EnableVertexAttribArray(color_attr_u);
        gl_error();
        println!("  vertex attrib pointer");
        gl::VertexAttribPointer(color_attr_u, 3, gl::FLOAT, gl::FALSE, (vertex_size * mem::size_of::<f32>()) as GLint, mem::transmute(3 * mem::size_of::<f32>()));
        gl_error();
    }

    unsafe {
        gl::Enable(gl::DEPTH_TEST);
    }

    let alpha_u = prog.get_unif("alpha");
    let override_color_u = prog.get_unif("override_color");
    let model_u = prog.get_unif("model");
    let view_u = prog.get_unif("view");
    let proj_u = prog.get_unif("proj");

    let mut proj_m4 = perspective(deg(45.0), 800.0 / 600.0, 1.0, 10.0);
    let view_m4 = Matrix4::look_at(&Point3::new(2.5, 2.5, 2.0), &Point3::new(0.0, 0.0, 0.0), &Vector3::new(0.0, 1.0, 0.0));
    let scale_m3 = Matrix3::from_value(0.1);
    let model_m4 = Matrix4::identity() * Matrix4::from(scale_m3);

    proj_u.upload_m4f(&proj_m4);
    view_u.upload_m4f(&view_m4);
    model_u.upload_m4f(&model_m4);

    override_color_u.upload_3f(1.0, 1.0, 1.0);
    alpha_u.upload_1f(1.0);

    let mut rot_angle = 0.0;

    let t_start = precise_time_s();

    let mut tframe_start = t_start;
    let mut tframe_end;

    let mut pause = true;

    println!("starting main loop");
    while !window.should_close() {
        tframe_end = precise_time_s();
        let t_frame = tframe_end - tframe_start;
        tframe_start = tframe_end; // time of last frame

        let t_now = precise_time_s(); // time since beginning

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

        // clear
        unsafe {
            gl::ClearColor(0.0, 0.0, 0.0, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        if !pause {
            // update scene
            let t_diff = t_now - t_start;
            //alpha_u.upload_1f(((t_diff * 4.0).sin() as f32 + 1.0) / 2.0);

            rot_angle += 180.0 * t_frame as f32;

            let rot180 = Basis3::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), deg(rot_angle).into());
            let model_m4 = model_m4 * Matrix4::from(*rot180.as_ref());
            model_u.upload_m4f(&model_m4);
        }

        // draw graphics
        unsafe {
            gl::DrawElements(gl::LINE_STRIP, elements.len() as i32, gl::UNSIGNED_INT, ptr::null());
            gl_error();
        }

        // present graphics
        window.swap_buffers();
    }

    println!("open.gl tutorial end");
}

fn handle_window_event(window: &mut glfw::Window, event: glfw::WindowEvent) {
}

fn gl_error() {
    let er = unsafe { gl::GetError() };
    if er != 0 {
        println!("gl error? {}", er);
    }
}
