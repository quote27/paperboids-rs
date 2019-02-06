extern crate cgmath;
extern crate gl;
extern crate glfw;

use cgmath::{Matrix4, Vector4};
use gl::types::*;
use std::ffi::CString;
use std::fmt;
use std::mem;
use std::ptr;

/// Load a file at the given source, return a String
/// with the contents
fn load_file(file_src: &str) -> String {
    use std::fs::File;
    use std::io::prelude::*;
    use std::path::Path;

    let path = Path::new(file_src);
    let mut f = File::open(&path).unwrap();
    let mut dat = String::new();
    match f.read_to_string(&mut dat) {
        Err(e) => panic!("failed to load file: {}", e),
        Ok(_) => println!("loaded {} successfully", path.to_str().unwrap()),
    }
    dat
}

/// Shader type.  Contains information relevant to opengl shaders.
pub struct Shader {
    /// file name of the shader (optional)
    file_name: String,
    /// shader contents in string form
    shader_str: String,
    /// shader type
    ty: GLenum,
    /// opengl shader handle
    s: GLuint,
}

impl Shader {
    /// Create a shader with type `ty` from file at `file_src`
    pub fn from_file(ty: GLenum, file_src: &str) -> Shader {
        let sh = load_file(file_src);
        let s = Shader::load_shader(ty, &sh);
        Shader {
            file_name: file_src.to_string(),
            shader_str: sh,
            ty: ty,
            s: s,
        }
    }

    /// Create a shader with type `ty` from the given string `shader_str`
    pub fn from_str(ty: GLenum, shader_str: &str) -> Shader {
        let s = Shader::load_shader(ty, &shader_str);
        Shader {
            file_name: "<none>".to_string(),
            shader_str: shader_str.to_string(),
            ty: ty,
            s: s,
        }
    }

    //TODO: write file load logic using http://static.rust-lang.org/doc/master/std/io/fs/struct.File.html
    /// Load a shader and return its handle.
    fn load_shader(ty: GLenum, src: &str) -> GLuint {
        unsafe {
            let shader = gl::CreateShader(ty);
            // attempt to compile the shader
            let c_str = CString::new(src).unwrap();
            gl::ShaderSource(shader, 1, &c_str.as_ptr(), ptr::null());
            gl::CompileShader(shader);

            // get the compile status
            let mut status = gl::FALSE as GLint;
            gl::GetShaderiv(shader, gl::COMPILE_STATUS, &mut status);

            // println!("loading shader: status: {}", status);

            // fail on error
            if status != (gl::TRUE as GLint) {
                let mut len = 0;
                gl::GetShaderiv(shader, gl::INFO_LOG_LENGTH, &mut len);
                // TODO: removed the subtract 1 on len, this may cause a crash
                let mut buf = Vec::with_capacity(len as usize);
                for _ in 0..len - 1 {
                    buf.push(0u8);
                }
                gl::GetShaderInfoLog(
                    shader,
                    len,
                    ptr::null_mut(),
                    buf.as_mut_ptr() as *mut GLchar,
                );
                panic!("{}", String::from_utf8(buf).unwrap());
            }
            shader
        }
    }
}

impl fmt::Display for Shader {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "file: {}, type: {}, s: {}",
            self.file_name, self.ty, self.s
        )
    }
}

pub struct Program(GLuint);

impl Program {
    pub fn new(shader_list: &Vec<Shader>) -> Program {
        unsafe {
            let prog = gl::CreateProgram();
            for s in shader_list.iter() {
                gl::AttachShader(prog, s.s);
            }
            gl::LinkProgram(prog);
            // get the link status
            let mut status = gl::FALSE as GLint;
            gl::GetProgramiv(prog, gl::LINK_STATUS, &mut status);

            // println!("loading program: status: {}", status);

            // fail on error
            if status != (gl::TRUE as GLint) {
                let mut len: GLint = 0;
                gl::GetProgramiv(prog, gl::INFO_LOG_LENGTH, &mut len);
                // TODO: removed the subtract 1 on len, this may cause a crash
                let mut buf = Vec::with_capacity(len as usize);
                for _ in 0..len - 1 {
                    buf.push(0u8);
                }
                gl::GetProgramInfoLog(prog, len, ptr::null_mut(), buf.as_mut_ptr() as *mut GLchar);
                panic!("{}", String::from_utf8(buf).unwrap());
            }
            Program(prog)
        }
    }

    #[inline(always)]
    pub fn get_unif(&self, name: &str) -> Uniform {
        let Program(p) = *self;
        let u = unsafe {
            let c_str = CString::new(name).unwrap();
            gl::GetUniformLocation(p, c_str.as_ptr())
        };
        Uniform(u)
    }

    #[inline(always)]
    pub fn get_attrib(&self, name: &str) -> GLint {
        let Program(p) = *self;
        unsafe {
            let c_str = CString::new(name).unwrap();
            gl::GetAttribLocation(p, c_str.as_ptr()) as GLint
        }
    }

    #[inline(always)]
    pub fn use_prog(&self) {
        let Program(p) = *self;
        unsafe {
            gl::UseProgram(p);
        }
    }

    #[inline(always)]
    pub fn delete(&mut self) {
        let Program(p) = *self;
        unsafe {
            gl::DeleteProgram(p);
        }
    }
}

impl fmt::Display for Program {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let Program(p) = *self;
        write!(f, "{}", p)
    }
}

pub struct Uniform(GLint);

impl Uniform {
    pub fn upload_m4f(&self, m: &Matrix4<f32>) {
        let Uniform(u) = *self;
        unsafe {
            gl::UniformMatrix4fv(u, 1, gl::FALSE, mem::transmute(m));
        }
    }

    pub fn upload_v4f(&self, v: &Vector4<f32>) {
        let Uniform(u) = *self;
        unsafe {
            gl::Uniform4fv(u, 4, mem::transmute(v));
        }
    }

    pub fn upload_3f(&self, a: f32, b: f32, c: f32) {
        let Uniform(u) = *self;
        unsafe {
            gl::Uniform3f(u, a, b, c);
        }
    }

    pub fn upload_1f(&self, a: f32) {
        let Uniform(u) = *self;
        unsafe {
            gl::Uniform1f(u, a);
        }
    }

    pub fn upload_1i(&self, a: i32) {
        let Uniform(u) = *self;
        unsafe {
            gl::Uniform1i(u, a);
        }
    }
}

impl fmt::Display for Uniform {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let Uniform(u) = *self;
        write!(f, "{}", u)
    }
}
