extern crate cgmath;

use cgmath::{Vector, Vector3};
use std::fmt;

/// An axis aligned bounding box.  Uses cgmath's Vector3<f32>.
///
/// ```
/// let b = AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(100.0, 100.0, 100.0));
///
/// assert_eq!(b.xlen(), 100.0);
///
/// let c = b.center();
///
/// assert_eq!(c, Vector3::new(50.0, 50.0, 50.0));
///
/// let mut bhalf = b.clone();
/// bhalf.scale(0.5);
///
/// assert_eq!(bhalf.l, Vector3::new(0.0, 0.0, 0.0));
/// assert_eq!(bhalf.h, Vector3::new(50.0, 50.0, 50.0));
///
/// let mut bhalf2 = b.clone();
/// bhalf2.scale_center(0.5);
///
/// assert_eq!(bhalf2.l, Vector3::new(25.0, 25.0, 25.0));
/// assert_eq!(bhalf2.h, Vector3::new(75.0, 75.0, 75.0));
///
/// bhalf2.trans(&Vector3::new(-1.0, 5.0, 0.0));
///
/// assert_eq!(bhalf2.l, Vector3::new(24.0, 30.0, 25.0));
/// assert_eq!(bhalf2.h, Vector3::new(74.0, 80.0, 75.0));
/// ```
pub struct AABB {
    pub l: Vector3<f32>,
    pub h: Vector3<f32>,
}

impl AABB {
    /// Creates a new axis aligned bounding box.  Doesn't check to see if low < high.
    pub fn new(low: Vector3<f32>, high: Vector3<f32>) -> AABB {
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

    /// Returns a Vector3<f32> representing the center of the box.
    pub fn center(&self) -> Vector3<f32> {
        // have to do tricks to convert from points to vectors
        self.l + (self.h - self.l).mul_s(0.5)
    }

    /// Scales the aabb relative to the origin.
    pub fn scale(&mut self, scale: f32) {
        self.h.x = self.l.x + self.xlen() * scale;
        self.h.y = self.l.y + self.ylen() * scale;
        self.h.z = self.l.z + self.zlen() * scale;
    }

    /// Scales the aabb relative to its center.
    pub fn scale_center(&mut self, scale: f32) {
        self.scale(scale);

        let scale = 1.0 - scale; // (1 - scale) * xl = xl - xl*scale
        let xl = self.xlen();
        let yl = self.ylen();
        let zl = self.zlen();

        let diffv = Vector3::new(xl, yl, zl).mul_s(scale * 0.5);
        self.trans(&diffv);
    }

    /// Translate the box by the given Vector3<f32>.
    #[inline(always)]
    pub fn trans(&mut self, trans: &Vector3<f32>) {
        self.h = self.h + *trans;
        self.l = self.l + *trans;
    }
}

impl fmt::Display for AABB {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[l: ({}, {}, {}), h: ({}, {}, {})]", 
               self.l.x, self.l.y, self.l.z,
               self.h.x, self.h.y, self.h.z)
    }
}
