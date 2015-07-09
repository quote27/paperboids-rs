extern crate gl;
extern crate cgmath;
extern crate time;

use gl::types::*;
use cgmath::*;
use shaders::Program;

pub struct Mesh {
    vao: GLuint,
    vbo: GLuint,
    ebo: GLuint,

    vertices: Vec<f32>,
    vertex_size: u32,
    elements: Vec<u32>,
    name: String,
}

impl Mesh {
    fn new(name: &str, v: Vec<f32>, e: Vec<u32>, vert_size: u32) -> Mesh {
        println!("creating mesh: {}", name);

        println!("  vertex array object [vao]");
        let mut vao = 0;
        unsafe {
            gl::GenVertexArrays(1, &mut vao);
            gl::BindVertexArray(vao);
        }
        gl_error();

        println!("  array_buffer object [verts] [vbo]");
        let mut vbo = 0;
        unsafe {
            gl::GenBuffers(1, &mut vbo);
            gl::BindBuffer(gl::ARRAY_BUFFER, vbo);
            gl::BufferData(gl::ARRAY_BUFFER, (v.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&vertices[0]), gl::STATIC_DRAW);
        }
        gl_error();

        println!("  element_array_buffer object [elements] [ebo]");
        let mut ebo = 0;
        unsafe {
            gl::GenBuffers(1, &mut ebo);
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo);
            gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, (elements.len() * mem::size_of::<u32>()) as GLsizeiptr, mem::transmute(&elements[0]), gl::STATIC_DRAW);
        }
        gl_error();

        println!("  unbinding stuff");
        unsafe {
            gl::BindBuffer(gl::ARRAY_BUFFER, 0);
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0);
            gl::BindVertexArray(0);
        }

        println!("finished mesh: {}", name);
        Mesh {
            vao: vao,
            vbo: vbo,
            ebo: ebo,
            vertices: v,
            elements: e,
            name: String::from_str(name),
        }
    }

    fn enable_attrib(&mut self, p: &Program) {
        println!("mesh: {}: enable_attribs enter", self.name);

        p.use_prog();

        unsafe {
            gl::BindVertexArray(self.vao);
        }

        unsafe {
            let pos_attr = prog.get_attrib("position") as GLuint;
            println!("  enable vertex attrib array");
            gl::EnableVertexAttribArray(pos_attr);
            gl_error();
            println!("  vertex attrib pointer");
            gl::VertexAttribPointer(pos_attr, 3, gl::FLOAT, gl::FALSE, (vertex_size * mem::size_of::<f32>()) as GLint, ptr::null());
            gl_error();
        }

        unsafe {
            let color_attr = prog.get_attrib("color") as GLuint;
            println!("  enable vertex attrib array");
            gl::EnableVertexAttribArray(color_attr);
            gl_error();
            println!("  vertex attrib pointer");
            gl::VertexAttribPointer(color_attr, 3, gl::FLOAT, gl::FALSE, (vertex_size * mem::size_of::<f32>()) as GLint, mem::transmute(3 * mem::size_of::<f32>()));
            gl_error();
        }

        println!("mesh: {}: enable_attrib exit", self.name);
    }
}
