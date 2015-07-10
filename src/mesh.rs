extern crate gl;
extern crate cgmath;
extern crate time;

use gl::types::*;
use cgmath::*;
use time::precise_time_ns;
use std::mem;
use std::ptr;

use ::{gl_error, gl_error_str};

pub struct Mesh {
    vao: GLuint,
    vbo: GLuint,
    ebo: GLuint,
    ibo: GLuint,

    vertices: Vec<f32>,
    vertex_size: usize,
    elements: Vec<u32>,
    name: String,
}

impl Mesh {
    pub fn new(name: &str, v: Vec<f32>, e: Vec<u32>, vert_size: usize) -> Mesh {
        println!("creating mesh: {}", name);

        let mut vao = 0;
        let mut vbo = 0;
        let mut ebo = 0;

        unsafe {
            println!("  gen vertex array object [vao]");
            gl::GenVertexArrays(1, &mut vao);
            gl::BindVertexArray(vao);

            println!("  gen array_buffer object [verts] [vbo]");
            gl::GenBuffers(1, &mut vbo);
            gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
            gl::BufferData(gl::ARRAY_BUFFER, (v.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&v[0]), gl::STATIC_DRAW);
            gl_error();

            println!("  gen element_array_buffer object [elements] [ebo]");
            gl::GenBuffers(1, &mut ebo);
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo);
            gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, (e.len() * mem::size_of::<u32>()) as GLsizeiptr, mem::transmute(&e[0]), gl::STATIC_DRAW);
            gl_error();

            println!("  unbinding stuff");
            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
            gl::BindVertexArray(0);
        }

        println!("finished mesh: {}", name);
        Mesh {
            vao: vao,
            vbo: vbo,
            ebo: ebo,
            ibo: 0,
            vertices: v,
            vertex_size: vert_size,
            elements: e,
            name: String::from(name),
        }
    }

    pub fn enable_attr(&mut self, pos_attr: GLuint, color_attr: GLuint) {
        println!("mesh: {}: enable_attr: pos_attr: {}, color_attr: {}", self.name, pos_attr, color_attr);

        unsafe {
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.vbo);

            gl::EnableVertexAttribArray(pos_attr);
            gl_error_str("mesh: enable_attr: pos: enable va");
            gl::VertexAttribPointer(pos_attr, 3, gl::FLOAT, gl::FALSE, (self.vertex_size * mem::size_of::<f32>()) as GLint, ptr::null());
            gl_error_str("mesh: enable_attr: pos: set va pointer");

            gl::EnableVertexAttribArray(color_attr);
            gl_error_str("mesh: enable_attr: color: enable va");
            gl::VertexAttribPointer(color_attr, 3, gl::FLOAT, gl::FALSE, (self.vertex_size * mem::size_of::<f32>()) as GLint, mem::transmute(3 * mem::size_of::<f32>()));
            gl_error_str("mesh: enable_attr: color: set va pointer");

            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl::BindVertexArray(0);
        }

        println!("mesh: {}: enable_attr: exit", self.name);
    }

    pub fn enable_instancing(&mut self, model_inst_attr: GLuint, model_inst: &Vec<Matrix4<f32>>) {
        println!("mesh: {}: enable_instancing: model_inst_attr: {}, num models: {}", self.name, model_inst_attr, model_inst.len());
        self.ibo = 0;

        unsafe {
            gl::BindVertexArray(self.vao);

            gl::GenBuffers(1, &mut self.ibo);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.ibo);
            gl_error_str("mesh: enable_instancing: bindbuffer");
            gl::BufferData(gl::ARRAY_BUFFER, model_inst_attr as GLsizeiptr, mem::transmute(&model_inst[0]), gl::STREAM_DRAW);
            gl_error_str("mesh: enable_instancing: bufferdata");

            let vec4_size = mem::size_of::<Vector4<f32>>();
            let mat4_size = mem::size_of::<Matrix4<f32>>();

            gl::EnableVertexAttribArray(model_inst_attr);
            gl_error_str("mesh: enable_instancing: enable va");
            gl::VertexAttribPointer(model_inst_attr, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, ptr::null());
            gl_error_str("mesh: enable_instancing: set va");
            gl::EnableVertexAttribArray(model_inst_attr + 1);
            gl::VertexAttribPointer(model_inst_attr + 1, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(vec4_size));
            gl::EnableVertexAttribArray(model_inst_attr + 2);
            gl::VertexAttribPointer(model_inst_attr + 2, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(2 * vec4_size));
            gl::EnableVertexAttribArray(model_inst_attr + 3);
            gl::VertexAttribPointer(model_inst_attr + 3, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(3 * vec4_size));

            gl::VertexAttribDivisor(model_inst_attr, 1);
            gl::VertexAttribDivisor(model_inst_attr + 1, 1);
            gl::VertexAttribDivisor(model_inst_attr + 2, 1);
            gl::VertexAttribDivisor(model_inst_attr + 3, 1);

            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl::BindVertexArray(0);
        }

        println!("mesh: {}: enable_instancing: exit", self.name);
    }

    pub fn update_instancing(&mut self, model_inst: &Vec<Matrix4<f32>>) {
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.ibo);

            let model_inst_size = model_inst.len() * mem::size_of::<Matrix4<f32>>();
            gl::BufferData(gl::ARRAY_BUFFER, model_inst_size as GLsizeiptr, mem::transmute(&model_inst[0]), gl::STREAM_DRAW);

            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl::BindVertexArray(0);
        }
    }

    pub fn draw_instanced(&mut self, num: GLint) {
        println!("draw: num: {}", num);
        unsafe {
            gl::BindVertexArray(self.vao);
            gl_error_str("mesh: draw: bind vao");
            gl::DrawElementsInstanced(gl::LINE_LOOP, self.elements.len() as i32, gl::UNSIGNED_INT, ptr::null(), num);
            gl_error_str("mesh: draw: element instanced");
            gl::BindVertexArray(0);
        }
    }
}
