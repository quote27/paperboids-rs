extern crate cgmath;
extern crate rand;

use cgmath::*;
use aabb::AABB;

pub struct Boid {
    pub pos: Vector3<f32>,
    pub vel: Vector3<f32>,
    pub acc: Vector3<f32>,
}

impl Boid {
    pub fn random_new(bbox: &AABB) -> Boid {
        use rand::Rng;
        let mut rand = rand::weak_rng();

        let x = bbox.l.x + rand.next_f32() * bbox.xlen();
        let y = bbox.l.y + rand.next_f32() * bbox.ylen();
        let z = bbox.l.z + rand.next_f32() * bbox.zlen();

        let vx = 10.0 - rand.next_f32() * 20.0;
        let vy = 10.0 - rand.next_f32() * 20.0;
        let vz = 10.0 - rand.next_f32() * 20.0;

        Boid {
            pos: Vector3::new(x, y, z),
            vel: Vector3::new(vx, vy, vz),
            acc: Vector3::zero(),
        }
    }

    pub fn update(&mut self, dt: f32, world_scale: f32) {
        // TODO: figure out where to put these speed constants
        let max_speed = 2.0 * world_scale;
        let min_speed = 0.5 * world_scale;

        self.vel = self.vel + self.acc.mul_s(dt);

        let curr_speed = self.vel.length();
        if curr_speed > max_speed {
            self.vel = self.vel.mul_s(max_speed / curr_speed);
        } else if curr_speed < min_speed {
            self.vel = self.vel.mul_s(min_speed / curr_speed);
        }

        self.pos = self.pos + self.vel.mul_s(dt);
    }

    pub fn model(&self, mode: i32) -> Matrix4<f32> {
        // TODO: figure out 'up' vector to get bank rotation animation

        match mode {
            0 => {
                Matrix4::from_translation(&self.pos)
            }

            1 => {
                // this works if there is no y component
                // - dunno why i have to negate the 'x' direction
                let dir = Vector3::new(-self.vel.x, 0.0f32, self.vel.z);
                let up = Vector3::unit_y();
                Matrix4::from_translation(&self.pos) * Matrix4::from(Matrix3::look_at(&dir, &up))
            }

            2 => {
                // using Matrix4::look_at
                // this is buggy - position is off - seems to orbit the origin
                let eye = Point3::from_vec(&self.pos);
                let center = Point3::from_vec(&(self.pos + self.vel));
                let up = Vector3::unit_y();
                let mut m = Matrix4::look_at(&eye, &center, &up);
                m.w.x = 0.0;
                m.w.y = 0.0;
                m.w.z = 0.0;
                m.w.w = 1.0;
                m
            }

            3 => {
                // using Matrix3::look_at with Matrix4::from_translation
                // works when the y velocity component is 0
                let dir = Vector3::new(-self.vel.x, self.vel.y, self.vel.z);
                let dir = Vector3::new(-self.vel.x, 0.0, self.vel.z);
                let up = Vector3::unit_y();
                let mut m = Matrix4::from(Matrix3::look_at(&dir, &up));
                Matrix4::from_translation(&self.pos) * m
            }

            4 => {
               Matrix4::from_translation(&self.pos)
            }

            5 => {
                let eye = self.pos;
                let target = self.pos + self.vel;
                let up = Vector3::unit_y(); // TODO: this is probably not right...

                let za = (eye - target).normalize();
                let xa = up.cross(&za).normalize();
                let ya = za.cross(&xa);

                Matrix4::new( xa.x, ya.x, za.x, zero(),
                              xa.y, ya.y, za.y, zero(),
                              xa.z, ya.z, za.z, zero(),
                              -xa.dot(&eye), -ya.dot(&eye), -za.dot(&eye), one())
            }

            _ => {
                Matrix4::from_translation(&self.pos)
            }
        }
    }
}

