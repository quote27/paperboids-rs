extern crate time;

use std::collections::BTreeMap;
use std::fmt;
use time::precise_time_ns;

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
        self.s = precise_time_ns();
    }

    /// Stops the timer and returns the elapsed time in miliseconds.
    #[inline(always)]
    pub fn stop(&mut self) -> f64 {
        self.e = precise_time_ns();
        self.elapsedms()
    }

    /// Prints out the elapsed time in miliseconds since the last stopped time.
    #[inline(always)]
    pub fn elapsedms(&self) -> f64 {
        (self.e - self.s) as f64 * 1e-6 // nanoseconds -> ms
    }
}

/// A map to store a collection of timing results.  Wrapper around a
/// BTreeMap<&'static str, f64> to store <string, time> values.
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
///     tm.update(states[0], t.stop());
///
///     t.start();
///     objects.sort();
///     tm.update(states[1], t.stop());
///
///     t.start();
///     objects.draw();
///     tm.update(states[2], t.stop());
/// }
///
/// tm.avg(iter);
/// println!("{}", tm);
/// ```
#[derive(Debug)]
pub struct TimeMap {
    tm: BTreeMap<&'static str, f64>,
}

impl TimeMap {
    /// Creates an empty TimeMap.
    pub fn new() -> TimeMap {
        TimeMap {
            tm: BTreeMap::new(),
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
        let t = match self.tm.get(&s) {
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
    pub fn avg(&mut self, count: usize) {
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

impl fmt::Display for TimeMap {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[");
        for (k, v) in self.tm.iter() {
            write!(f, " {}: {:.4},", k, v);
        }
        write!(f, "]")
    }
}
