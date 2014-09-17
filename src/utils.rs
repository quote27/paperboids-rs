extern crate time;

use nalgebra::na::{Vec3};
use std::collections::TreeMap;
use std::fmt::{Show, Formatter, FormatError};

/// A nanosecond resolution timer.
///
/// # Examples
///
/// Time one specific section
/// ```
/// let mut t = Timer::new();
/// t.start();
/// foo();
/// t.stop();
/// println!("time: {}", t.elapsedms());
/// ```
///
/// Time a series of sections with results relative to a starting point
/// ```
/// let mut t = Timer::new();
/// t.start();
/// foo();
/// println("start -> foo time: {}", t.stop());
///
/// bar();
/// println("start -> foo -> bar time: {}", t.stop());
/// ```
pub struct Timer {
    s: u64,
    e: u64,
}

impl Timer {
    /// Creates a new timer
    ///
    /// # Example
    ///
    /// ```
    /// let mut t = Timer::new();
    /// ```
    pub fn new() -> Timer {
        Timer { s: 0, e: 0 }
    }

    /// Starts the timer
    ///
    /// # Example
    ///
    /// ```
    /// let mut t = Timer::new();
    /// t.start();
    /// foo();
    /// ```
    #[inline(always)]
    pub fn start(&mut self) {
        self.s = time::precise_time_ns();
    }

    /// Stops the timer and returns the elapsed time in miliseconds
    ///
    /// # Exxample
    ///
    /// ```
    /// let mut t = Timer::new();
    /// t.start();
    /// foo();
    /// println!("time: {}", t.stop());
    /// ```
    #[inline(always)]
    pub fn stop(&mut self) -> f64 {
        self.e = time::precise_time_ns();
        self.elapsedms()
    }

    /// Prints out the elapsed time since the last stopped time
    ///
    /// # Example
    ///
    /// ```
    /// let mut t = Timer::new();
    /// t.start();
    /// foo();
    /// t.stop();
    /// println!("time: {}", t.elapsedms());
    /// ```
    #[inline(always)]
    pub fn elapsedms(&self) -> f64 {
        (self.e - self.s) as f64 * 1e-6 //nanoseconds -> ms
    }
}

pub struct TimeMap {
    tm: TreeMap<&'static str, f64>,
}

impl TimeMap {
    pub fn new() -> TimeMap {
        TimeMap {
            tm: TreeMap::new(),
        }
    }

    pub fn update(&mut self, s: &'static str, time: f64) {
        let t = match self.tm.find(&s) {
            None => time,
            Some(v) => time + *v,
        };

        self.tm.insert(s, t);
    }

    pub fn avg(&mut self, count: uint) {
        let count = count as f64;
        for (_, value) in self.tm.mut_iter() {
            *value /= count;
        }
    }

    pub fn clear(&mut self) {
        self.tm.clear();
    }
}

impl Show for TimeMap {
    fn fmt(&self, f: &mut Formatter) -> Result<(), FormatError> {
        f.write(format!("{}", self.tm).as_bytes())
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

    #[inline(always)]
    pub fn xlen(&self) -> f32 { self.h.x - self.l.x }

    #[inline(always)]
    pub fn ylen(&self) -> f32 { self.h.y - self.l.y }

    #[inline(always)]
    pub fn zlen(&self) -> f32 { self.h.z - self.l.z }

    pub fn center(&self) -> Vec3<f32> {
        self.l + (self.h - self.l) * 0.5f32
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

impl Show for AABB {
    fn fmt(&self, f: &mut Formatter) -> Result<(), FormatError> {
        f.write(format!("[l: {}, h: {}]", self.l, self.h).as_bytes())
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


