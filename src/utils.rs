extern crate time;

use nalgebra::na::{Vec3};
use std::collections::TreeMap;
use std::fmt::{Show, Formatter, FormatError};

/// A nanosecond resolution timer.  Results are returned in miliseconds.
/// This is basically a wrapper around `time::precise_time_ns()`.
///
/// # Examples
///
/// Time one specific section.
/// ```
/// let mut t = Timer::new();
/// t.start();
/// foo();
/// t.stop();
/// println!("foo: {}", t.elapsedms());
///
/// t.start(); // can re-use timers
/// bar();
/// println!("bar: {}", t.stop()); // `t.stop()` also returns the elapsed time
/// ```
///
/// Time a series of sections with results relative to a starting point.
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
    /// Creates a new timer.
    pub fn new() -> Timer {
        Timer { s: 0, e: 0 }
    }

    /// Starts the timer.
    #[inline(always)]
    pub fn start(&mut self) {
        self.s = time::precise_time_ns();
    }

    /// Stops the timer and returns the elapsed time in miliseconds.
    #[inline(always)]
    pub fn stop(&mut self) -> f64 {
        self.e = time::precise_time_ns();
        self.elapsedms()
    }

    /// Prints out the elapsed time in miliseconds since the last stopped time.
    #[inline(always)]
    pub fn elapsedms(&self) -> f64 {
        (self.e - self.s) as f64 * 1e-6 // nanoseconds -> ms
    }
}

/// A map to store a collection of timing results.  Wrapper around a
/// TreeMap<&'static str, f64> to store <string, time> values.
///
/// Used to aggregate times for named sections of code.  At the end
/// of a run, results can be averaged and printed out.
///
/// # Example
///
/// ```
/// let tm = TimeMap::new();
/// let t = Timer::new(); // Timer from util mod
/// let states = ["0.move", "1.sort", "2.draw"];
/// let iter = 1000;
///
/// let mut objects = gen_objects();
///
/// for i in range(0, iter) {
///     t.start();
///     objects.move();
///     tm.update(states[0], t.end());
///
///     t.start();
///     objects.sort();
///     tm.update(states[1], t.end());
///
///     t.start();
///     objects.draw();
///     tm.update(states[2], t.end());
/// }
///
/// tm.avg(iter);
/// println!("{}", tm);
/// ```
pub struct TimeMap {
    tm: TreeMap<&'static str, f64>,
}

impl TimeMap {
    /// Creates an empty TimeMap.
    pub fn new() -> TimeMap {
        TimeMap {
            tm: TreeMap::new(),
        }
    }

    /// Accumulates a time value in the map.  Inserts the entry if it
    /// doesn't already exist.
    ///
    /// # Example
    ///
    /// ```
    /// let mut tm = TimeMap::new();
    ///
    /// tm.update("a", 1.0); // {a: 1.0}
    /// tm.update("a", 2.0); // {a: 3.0}
    /// ```
    pub fn update(&mut self, s: &'static str, time: f64) {
        let t = match self.tm.find(&s) {
            None => time,
            Some(v) => time + *v,
        };

        self.tm.insert(s, t);
    }

    /// Average the results by dividing each entry by `count`.
    ///
    /// # Example
    ///
    /// ```
    /// let mut tm = TimeMap::new();
    ///
    /// tm.update("a", 20.0); // {a: 20.0}
    /// tm.update("b", 10.0); // {a: 20.0, b: 10.0}
    ///
    /// tm.avg(10); // {a: 2, b: 1}
    /// ```
    pub fn avg(&mut self, count: uint) {
        let count = count as f64;
        for (_, value) in self.tm.iter_mut() {
            *value /= count;
        }
    }

    /// Clear the map.
    ///
    /// # Example
    ///
    /// ```
    /// let mut tm = TimeMap::new();
    ///
    /// tm.update("a", 1.0); // {a: 1.0}
    /// tm.clear(); // {}
    /// ```
    pub fn clear(&mut self) {
        self.tm.clear();
    }
}

impl Show for TimeMap {
    fn fmt(&self, f: &mut Formatter) -> Result<(), FormatError> {
        f.write(format!("{}", self.tm).as_bytes())
    }
}

/// An axis aligned bounding box.  Uses nalgebra's Vec3<f32>.
///
/// ```
/// let b = AABB::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 100.0));
///
/// assert_eq!(b.xlen(), 100.0);
///
/// let c = b.center();
///
/// assert_eq!(c, Vec3::new(50.0, 50.0, 50.0));
///
/// let mut bhalf = b.clone();
/// bhalf.scale(0.5);
///
/// assert_eq!(bhalf.l, Vec3::new(0.0, 0.0, 0.0));
/// assert_eq!(bhalf.h, Vec3::new(50.0, 50.0, 50.0));
///
/// let mut bhalf2 = b.clone();
/// bhalf2.scale_center(0.5);
///
/// assert_eq!(bhalf2.l, Vec3::new(25.0, 25.0, 25.0));
/// assert_eq!(bhalf2.h, Vec3::new(75.0, 75.0, 75.0));
///
/// bhalf2.trans(&Vec3::new(-1.0, 5.0, 0.0));
///
/// assert_eq!(bhalf2.l, Vec3::new(24.0, 30.0, 25.0));
/// assert_eq!(bhalf2.h, Vec3::new(74.0, 80.0, 75.0));
/// ```
#[deriving(Clone)]
pub struct AABB {
    pub l: Vec3<f32>,
    pub h: Vec3<f32>,
}

impl AABB {
    /// Creates a new axis aligned bounding box.  Doesn't check to see if low < high.
    pub fn new(low: Vec3<f32>, high: Vec3<f32>) -> AABB {
        AABB { l: low, h: high, }
    }

    /// Calculates length of x-axis.
    #[inline(always)]
    pub fn xlen(&self) -> f32 { self.h.x - self.l.x }

    /// Calculates length of y-axis.
    #[inline(always)]
    pub fn ylen(&self) -> f32 { self.h.y - self.l.y }

    /// Calculates length of z-axis.
    #[inline(always)]
    pub fn zlen(&self) -> f32 { self.h.z - self.l.z }

    /// Returns a Vec3<f32> representing the center of the box.
    pub fn center(&self) -> Vec3<f32> {
        self.l + (self.h - self.l) * 0.5f32
    }

    /// Scales the aabb relative to the origin.
    pub fn scale(&mut self, scale: f32) {
        self.h.x = self.l.x + self.xlen() * scale;
        self.h.y = self.l.y + self.ylen() * scale;
        self.h.z = self.l.z + self.zlen() * scale;
    }

    /// Scales the aabb relative to its center.
    pub fn scale_center(&mut self, scale: f32) {
        let xl = self.xlen();
        let yl = self.ylen();
        let zl = self.zlen();

        let diffv = Vec3::new(xl - xl * scale, yl - yl * scale, zl - zl * scale) * 0.5f32;

        self.scale(scale);
        self.trans(&diffv);
    }

    /// Translate the box by the given Vec3<f32>.
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

/// Compare and return the minimum of 2 f32s
#[inline(always)]
pub fn min(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

/// Compare and return the maximum of 2 f32s
#[inline(always)]
pub fn max(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}


