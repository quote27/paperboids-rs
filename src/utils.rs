extern crate time;

use nalgebra::na::{Vec2, Vec3};
use nalgebra::na;

pub struct Timer { s: u64, e: u64, }
impl Timer {
    pub fn new() -> Timer {
        Timer { s: 0, e: 0 }
    }

    #[inline(always)]
    pub fn start(&mut self) {
        self.s = time::precise_time_ns();
    }

    #[inline(always)]
    pub fn stop(&mut self) -> f64 {
        self.e = time::precise_time_ns();
        self.elapsedms()
    }

    #[inline(always)]
    pub fn elapsedms(&self) -> f64 {
        (self.e - self.s) as f64 / 1e6 //nanoseconds -> ms
    }
}

#[deriving(Clone)]
pub struct AABB {
    pub l: Vec3<f32>,
    pub h: Vec3<f32>,
}

impl AABB {
    pub fn new(low: Vec3<f32>, high: Vec3<f32>) -> AABB {
        AABB { l: low, h: high, }
    }

    pub fn empty() -> AABB {
        AABB { l: na::zero(), h: na::zero() }
    }

    #[inline(always)]
    pub fn xlen(&self) -> f32 { self.h.x - self.l.x }

    #[inline(always)]
    pub fn ylen(&self) -> f32 { self.h.y - self.l.y }

    #[inline(always)]
    pub fn zlen(&self) -> f32 { self.h.z - self.l.z }

    pub fn center(&self) -> Vec3<f32> {
        self.l + (self.h - self.l) / 2.0f32
    }

    // scales but pins to lower corner
    pub fn scale(&mut self, scale: f32) {
        self.h.x = self.l.x + self.xlen() * scale;
        self.h.y = self.l.y + self.ylen() * scale;
        self.h.z = self.l.z + self.zlen() * scale;
    }

    pub fn scale_center(&mut self, scale: f32) {
        let xl = self.xlen();
        let yl = self.ylen();
        let zl = self.zlen();

        let diffv = Vec3::new(xl - xl * scale, yl - yl * scale, zl - zl * scale) * 0.5f32;

        self.scale(scale);
        self.trans(&diffv);
    }

    #[inline(always)]
    pub fn trans(&mut self, trans: &Vec3<f32>) {
        self.h = self.h + *trans;
        self.l = self.l + *trans;
    }
}

#[inline(always)]
pub fn min(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

#[inline(always)]
pub fn max(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}


