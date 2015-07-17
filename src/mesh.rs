extern crate gl;
extern crate cgmath;
extern crate time;

use gl::types::*;
use cgmath::*;
use std::mem;
use std::ptr;
use ::gl_error_str;

/// Mesh structure containing vertex and element information.
///
/// Has helper functions to handle opengl logic [setting up buffers
/// and configuring instancing].
///
/// # Example
///
/// ```
/// let vertices = gen_verts();
/// let elements = gen_elem();
/// let vertex_size = 6; // number of floats in the vertex
///
/// let mut test_mesh = Mesh::new(vertices, elements, vertex_size);
/// test_mesh.setup(pos_a, color_a, model_inst_a); // attributes
/// test_mesh.update_inst(&model_inst_vec); // vec of model transform matrices
///
/// // do stuff
///
/// test_mesh.draw_inst(num); // calls gl::DrawElementsInstanced
/// ```
pub struct Mesh {
    vao: GLuint,
    vbo: GLuint,
    ebo: GLuint,
    ibo: GLuint,

    pub vertices: Vec<f32>,
    pub vertex_size: usize,
    elements: Vec<u32>,
    name: String,

    gl_type: GLenum,
}

impl Mesh {
    /// Generate a new mesh with the given vertices and elements.  Moves the vectors
    /// into the struct.  Does not set up any opengl logic.
    pub fn new(name: &str, v: Vec<f32>, e: Vec<u32>, vert_size: usize, gl_type: GLenum) -> Mesh {
        Mesh {
            vao: 0,
            vbo: 0,
            ebo: 0,
            ibo: 0,
            vertices: v,
            vertex_size: vert_size,
            elements: e,
            name: String::from(name),
            gl_type: gl_type,
        }
    }

    /// Setup vertex array / buffers / attributes for the mesh.
    pub fn setup(&mut self, pos_a: GLuint, color_a: GLuint, model_inst_a: GLuint) {
        println!("mesh: {}: setup: pos_a: {}, color_a: {}, model_inst_a: {}", self.name, pos_a, color_a, model_inst_a);

        unsafe {
            gl::GenVertexArrays(1, &mut self.vao);
            gl::BindVertexArray(self.vao);

            // setup vertex and element buffers
            gl::GenBuffers(1, &mut self.vbo);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.vbo);
            gl::BufferData(gl::ARRAY_BUFFER, (self.vertices.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&self.vertices[0]), gl::STATIC_DRAW);

            gl::GenBuffers(1, &mut self.ebo);
            gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, self.ebo);
            gl::BufferData(gl::ELEMENT_ARRAY_BUFFER, (self.elements.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&self.elements[0]), gl::STATIC_DRAW);

            // enable position and color attributes
            gl::EnableVertexAttribArray(pos_a);
            gl::VertexAttribPointer(pos_a, 3, gl::FLOAT, gl::FALSE, (self.vertex_size * mem::size_of::<f32>()) as GLint, ptr::null());

            gl::EnableVertexAttribArray(color_a);
            gl::VertexAttribPointer(color_a, 3, gl::FLOAT, gl::FALSE, (self.vertex_size * mem::size_of::<f32>()) as GLint, mem::transmute(3 * mem::size_of::<f32>()));

            // setup instancing buffer (but don't copy now)
            gl::GenBuffers(1, &mut self.ibo);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.ibo);

            let vec4_size = mem::size_of::<Vector4<f32>>();
            let mat4_size = mem::size_of::<Matrix4<f32>>();

            // enable model instancing attributes
            gl::EnableVertexAttribArray(model_inst_a);
            gl::VertexAttribPointer(model_inst_a, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, ptr::null());
            gl::EnableVertexAttribArray(model_inst_a + 1);
            gl::VertexAttribPointer(model_inst_a + 1, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(vec4_size));
            gl::EnableVertexAttribArray(model_inst_a + 2);
            gl::VertexAttribPointer(model_inst_a + 2, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(vec4_size * 2));
            gl::EnableVertexAttribArray(model_inst_a + 3);
            gl::VertexAttribPointer(model_inst_a + 3, 4, gl::FLOAT, gl::FALSE, mat4_size as GLint, mem::transmute(vec4_size * 3));

            gl::VertexAttribDivisor(model_inst_a, 1);
            gl::VertexAttribDivisor(model_inst_a + 1, 1);
            gl::VertexAttribDivisor(model_inst_a + 2, 1);
            gl::VertexAttribDivisor(model_inst_a + 3, 1);

            gl::BindVertexArray(0);
        }
        gl_error_str("mesh: setup");
    }

    /// Upload new vector of model transform matrices to the instance array buffer.
    pub fn update_inst(&mut self, model_inst: &Vec<Matrix4<f32>>) {
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.ibo);

            let model_inst_size = model_inst.len() * mem::size_of::<Matrix4<f32>>();
            gl::BufferData(gl::ARRAY_BUFFER, model_inst_size as GLsizeiptr, mem::transmute(&model_inst[0]), gl::STREAM_DRAW);

            gl::BindVertexArray(0);
        }
        gl_error_str("mesh: update_inst");
    }

    pub fn update_verts(&mut self) {
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::BindBuffer(gl::ARRAY_BUFFER, self.vbo);
            gl::BufferData(gl::ARRAY_BUFFER, (self.vertices.len() * mem::size_of::<f32>()) as GLsizeiptr, mem::transmute(&self.vertices[0]), gl::DYNAMIC_DRAW);
            gl::BindVertexArray(0);
        }
        gl_error_str("mesh: update_verts");
    }

    /// Draw `num` instances of the mesh.
    pub fn draw_inst(&mut self, num: GLint) {
        unsafe {
            gl::BindVertexArray(self.vao);
            gl::DrawElementsInstanced(gl::LINE_LOOP, self.elements.len() as i32, gl::UNSIGNED_INT, ptr::null(), num);
            gl::BindVertexArray(0);
        }
        gl_error_str("mesh: draw_inst");
    }
}
