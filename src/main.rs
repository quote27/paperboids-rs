extern crate gl;
extern crate glfw;
extern crate cgmath;
extern crate time;
extern crate rand;

use time::precise_time_ns;
use gl::types::*;
use glfw::{Action, Context, Key};
use cgmath::*;
use std::mem;
use std::ptr;
use shaders::{Shader, Program};
use mesh::Mesh;

mod shaders;
mod mesh;

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


    /*
    println!("creating vertex array object (vao)");
    let mut vao = 0;
    unsafe {
        gl::GenVertexArrays(1, &mut vao);
        gl::BindVertexArray(vao);
    }
    gl_error();
    */

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
        0, 1, 3u32,
        0, 3, 2,
        0, 4, 3,
    ];

    let elements = vec![
        0, 1, 2u32,
        0, 4, 5,
    ];


    // generate instances
    let mut model_positions = Vec::with_capacity(1000);
    {
        use rand::Rng;
        let mut rand = rand::weak_rng();

        for _ in 0..100 {
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

    /*
    // axis [x/y/z marker]
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

    /*
    // squares
    let vertices = [
        // vertex        // color
        0.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  0.0,  0.0, 1.0, 1.0, 1.0f32,
        1.0,  1.0,  0.0, 1.0, 1.0, 1.0f32,
        0.0,  1.0,  0.0, 1.0, 1.0, 1.0f32,
    ];
    let vertex_size = 6;

    let elements = [
        0, 1u32,
        1, 2,
        2, 3,
        3, 0,
    ];
    */


    println!("use program");
    prog.use_prog();

    let mut plane_mesh = Mesh::new("paperplane", vertices, elements, vertex_size);
    let pos_attr = prog.get_attrib("position") as GLuint;
    let color_attr = prog.get_attrib("color") as GLuint;
    let model_inst_attr = prog.get_attrib("model_inst") as GLuint;

    plane_mesh.enable_attr(pos_attr, color_attr);
    plane_mesh.enable_instancing(model_inst_attr, &model_inst);


    /*
    println!("vertices: creating vertex buffer object (vbo)");
    let mut vbo = 0;
    unsafe {
        gl::GenBuffers(1, &mut vbo);
        gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
        gl::BufferData(gl::ARRAY_BUFFER, (vertices.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&vertices[0]), gl::STATIC_DRAW);
    }
    gl_error();

    println!("elements: creating vertex buffer object (ebo)");
    let mut ebo = 0;
    unsafe {
        gl::GenBuffers(1, &mut ebo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo);
        gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, (elements.len() * mem::size_of::<u32>()) as GLsizeiptr, mem::transmute(&elements[0]), gl::STATIC_DRAW);
    }
    gl_error();


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
    println!("color: attribute: {}", color_attr);
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


    println!("instance_mat: creating vertex buffer object (ibo)");
    let mut ibo = 0;
    unsafe {
        gl::GenBuffers(1, &mut ibo);
        gl::BindBuffer(gl::ARRAY_BUFFER, ibo);
        let model_inst_size = model_inst.len() * mem::size_of::<Matrix4<f32>>();
        println!("model inst size: {}", model_inst_size);
        gl::BufferData(gl::ARRAY_BUFFER, model_inst_size as GLsizeiptr, mem::transmute(&model_inst[0]), gl::STREAM_DRAW);

        let model_inst_attr = prog.get_attrib("model_inst") as GLuint;
        println!("model_inst_attr: {} {} {} {}", model_inst_attr, model_inst_attr + 1, model_inst_attr + 2, model_inst_attr + 3);

        let vec4_size = mem::size_of::<Vector4<f32>>();
        println!("vec4 size: {}", vec4_size);
        let mat4_size = mem::size_of::<Matrix4<f32>>();
        println!("mat4 size: {}", mat4_size);

        gl::EnableVertexAttribArray(model_inst_attr);
        gl::VertexAttribPointer(    model_inst_attr, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, ptr::null());
        gl::EnableVertexAttribArray(model_inst_attr + 1);
        gl::VertexAttribPointer(    model_inst_attr + 1, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(vec4_size));
        gl::EnableVertexAttribArray(model_inst_attr + 2);
        gl::VertexAttribPointer(    model_inst_attr + 2, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(2 * vec4_size));
        gl::EnableVertexAttribArray(model_inst_attr + 3);
        gl::VertexAttribPointer(    model_inst_attr + 3, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(3 * vec4_size));

        gl::VertexAttribDivisor(model_inst_attr, 1);
        gl::VertexAttribDivisor(model_inst_attr + 1, 1);
        gl::VertexAttribDivisor(model_inst_attr + 2, 1);
        gl::VertexAttribDivisor(model_inst_attr + 3, 1);
        gl_error();

        gl::BindBuffer(gl::ARRAY_BUFFER, 0);
    }
    */


    // inlined v2




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

    let mut rot_angle = 0.0;

    let t_start = precise_time_ns();

    let mut tframe_start = t_start;
    let mut tframe_end;

    let mut pause = true;

    let mut frame_count = 0;

    println!("starting main loop");
    while !window.should_close() {
        if frame_count == 5 {
            break;
        }
        tframe_end = precise_time_ns();
        let t_frame = tframe_end - tframe_start;
        tframe_start = tframe_end; // time of last frame

        let t_now = precise_time_ns(); // time since beginning

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

        let tsection_start = precise_time_ns();
        if !pause {
            // update scene
            // calculate rotation based on time, update each model matrix

            rot_angle += 180.0 * (t_frame as f32 * 1e-9);

            let rot180 = Basis3::from_axis_angle(&Vector3::new(0.0, 0.0, 1.0), deg(rot_angle).into());
            let rot180_m4 = Matrix4::from(*rot180.as_ref());

            let shared_model = rot180_m4 * model_default_scale_mat;

            let tsection_start = precise_time_ns();
            model_inst.clear();
            for p in model_positions.iter() {
                model_inst.push(Matrix4::from_translation(p) * shared_model);
            }
            if frame_count % 10 == 0 {
                println!("vector build: {}", (precise_time_ns() - tsection_start) as f64 * 1e-6);
            }

            plane_mesh.update_instancing(&model_inst);

            /*
            unsafe {
                gl::BindBuffer(gl::ARRAY_BUFFER, ibo);
                let model_inst_size = model_inst.len() * mem::size_of::<Matrix4<f32>>();
                gl::BufferData(gl::ARRAY_BUFFER, model_inst_size as GLsizeiptr, mem::transmute(&model_inst[0]), gl::STREAM_DRAW);
                gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            }
            */
        }
        if frame_count % 10 == 0 {
            println!("compute section: {}", (precise_time_ns() - tsection_start) as f64 * 1e-6);
        }

        // draw planes
        let tsection_start = precise_time_ns();
        /*
        unsafe {
            gl::DrawElementsInstanced(gl::LINE_LOOP, elements.len() as i32, gl::UNSIGNED_INT, ptr::null(), model_inst.len() as GLint);
            gl_error_str("main loop: draw elements");
        }
        */
        plane_mesh.draw_instanced(model_inst.len() as GLint);

        if frame_count % 10 == 0 {
            println!("draw elements instanced: {}", (precise_time_ns() - tsection_start) as f64 * 1e-6);
        }

        if frame_count % 10 == 0 {
            println!("internal frame before swap: {}", (precise_time_ns() - tframe_end) as f64 * 1e-6);
        }

        window.swap_buffers();
        if frame_count % 10 == 0 {
            println!("internal frame with swap: {}", (precise_time_ns() - tframe_end) as f64 * 1e-6);
            println!("frame: {}", t_frame as f64 * 1e-6);
        }
        frame_count += 1;
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
