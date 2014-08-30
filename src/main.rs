extern crate native;
extern crate kiss3d;
extern crate nalgebra;
extern crate debug; //to print out type of variable at runtime

use std::rc::Rc;
use std::cell::RefCell;
use std::rand;
use nalgebra::na::Vec3;
use nalgebra::na;
use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use kiss3d::light;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

// reference: http://www.vergenet.net/~conrad/boids/pseudocode.html

//TODO: check out recorder demo to create mpg's of simulations

struct Plane {
    pos: Vec3<f32>,
    vel: Vec3<f32>,
    acc: Vec3<f32>,

    node: SceneNode,
}

impl Plane {
    fn gen_mesh() -> Rc<RefCell<Mesh>> {
        let vertices = vec!(
            Vec3::new(0.0, 0.0, 1.0), // front / nose
            Vec3::new(0.75, 0.0, -1.0), // left wing - 'port'
            Vec3::new(-0.75, 0.0, -1.0), // right wing - 'starboard'
            Vec3::new(0.0, 0.0, -1.0), // back midpoint between wings
            Vec3::new(0.0, -0.4, -1.0), // back bottom fin
        );

        let indices = vec!(
            Vec3::new(0u32, 1, 3),
            Vec3::new(0u32, 3, 2),
            Vec3::new(0u32, 4, 3),
        );

        Rc::new(RefCell::new(Mesh::new(vertices, indices, None, None, false)))
    }

    fn new(sn: SceneNode) -> Plane {
        let mut node = sn;
        node.set_color(1.0, 1.0, 1.0);
        node.enable_backface_culling(false);
        node.set_points_size(1.0); //wireframe mode for plane
        node.set_lines_width(1.0);
        node.set_surface_rendering_activation(false);

//        let mut cyl = node.add_cylinder(5.0, 0.0);
//        cyl.append_translation(&Vec3::new(0.0, -0.1, 0.0));
//        cyl.append_rotation(&Vec3::new(0.0, (90.0f32).to_radians(), 0.0));
//        cyl.set_color(0.5, 0.5, 0.5);
//        cyl.enable_backface_culling(true);
//        cyl.set_points_size(1.0); //wireframe mode for plane
//        cyl.set_lines_width(1.0);
//        cyl.set_surface_rendering_activation(false);

        let (x, z) = (rand::random::<f32>() * 50.0 - 25.0, rand::random::<f32>() * 50.0 - 25.0);
        let (vx, vz) = (rand::random::<f32>() * 40.0 - 20.0, rand::random::<f32>() * 40.0 - 20.0);
        let (y, vy) = (rand::random::<f32>() * 20.0 - 10.0 + 20.0, rand::random::<f32>() * 10.0 - 5.0);

        Plane {
            pos: Vec3::new(x, y, z),
            vel: Vec3::new(vx, vy, vz),
            acc: Vec3::new(0.0, 0.0, 0.0),
            node: node,
        }
    }

    fn update(&mut self, dt: f32) {
        self.vel = self.vel + self.acc * dt;

        let max_speed = 20.0;
        let min_speed = 2.0;
        let curr_speed = na::norm(&self.vel);
        if curr_speed > max_speed {
            self.vel = self.vel / curr_speed * max_speed;
        } else if curr_speed < min_speed {
            self.vel = self.vel / curr_speed * min_speed;
        }

        self.pos = self.pos + self.vel * dt;
        self.node.look_at_z(&self.pos, &(self.pos + self.vel), &Vec3::y());
    }
} 

fn main() {
    // have camera start from higher position
    let eye = Vec3::new(50.0, 50.0, 100.0);
    let at = na::one();
    let mut arc_ball = ArcBall::new(eye, at);

    let mut window = Window::new("Kiss3d: cube");
    window.set_framerate_limit(Some(60));
    window.set_light(light::StickToCamera);

    let mut ground = window.add_quad(100.0, 100.0, 1, 1);
    ground.set_local_rotation(Vec3::new((90.0f32).to_radians(), 0.0, 0.0));
    ground.set_color(0.1, 0.1, 0.1);

    let pmesh = Plane::gen_mesh();

    let num_planes = 500;

    let mut ps = Vec::new();
    for i in range(0i, num_planes) {
        //ps.push(Plane::new(window.add_mesh(pmesh.clone(), Vec3::new(0.2, 0.2, 0.2))));
        ps.push(Plane::new(window.add_mesh(pmesh.clone(), Vec3::new(1.0, 1.0, 1.0))));
    }

    let w1 = 0.5;
    let w2 = 50.0;
    let w3 = 0.5;

    let look_radius = 20.0;
    let collide_radius = 10.0; // TODO: figure out collide radius

    let mut last_time = window.context().get_time() as f32;
    let mut curr_time;
    while window.render_with_camera(&mut arc_ball) {
        curr_time = window.context().get_time() as f32;

        draw_axis(&mut window);

        //let flock_total_pos = ps.iter().fold(na::zero::<Vec3<f32>>(), |a, ref p| a + p.pos);
        //let flock_total_vel = ps.iter().fold(na::zero::<Vec3<f32>>(), |a, ref p| a + p.vel);

        //let flock_abs_center = flock_total_pos / (ps.len() as f32);
        //window.draw_point(&flock_abs_center, &Vec3::new(0.0, 1.0, 0.0));

        for i in range(0, ps.len()) {
            // rule 1 : calculate center minus the current bird
            let mut center_count = 0u;
            let center_pos = ps.iter()
                .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
                .fold(na::zero::<Vec3<f32>>(), |a, ref p| { center_count += 1; a + p.pos });
            center_count -= 1; // this is because the bird itself will be added

//            let (center_pos, neighbor_count) = ps.iter()
//                .fold((na::zero::<Vec3<f32>>(), 0), |(a, count), ref p| if na::norm(&(p.pos - ps[i].pos)) < look_radius { (a + p.pos, count+1) } else { (a,count) });
            //let center_pos = (flock_total_pos - ps[i].pos) / (ps.len() as f32 - 1.0);
            let r1_center_push =
                if center_count == 0 {
                    na::zero()
                } else {
                    ((center_pos - ps[i].pos) / center_count as f32) - ps[i].pos //un weighted rule
                };

            let mut r1_scaled = r1_center_push;
            r1_scaled.x *= w1;
            r1_scaled.y *= w1;
            r1_scaled.z *= w1;

            //window.draw_line(&ps[i].pos, &(ps[i].pos + r1_scaled), &Vec3::new(1.0, 0.0, 0.0));

            // rule 2 : steer away from nearby boids
            let mut r2_collide_push: Vec3<f32> = na::zero();
            let mut count_intersect  = 0u;
            for j in range(0, ps.len()) {
                if i != j {
                    let disp = ps[j].pos - ps[i].pos;
                    let dist = na::norm(&disp);
                    if dist < collide_radius {
                        let disp_norm_weight = disp / (dist * dist);
                        r2_collide_push = r2_collide_push - disp_norm_weight;
                        count_intersect += 1;
                    }
                }
            }
            if count_intersect > 0 {
                r2_collide_push = r2_collide_push / count_intersect as f32;
            }

            let mut r2_scaled = r2_collide_push;
            r2_scaled.x *= w2;
            r2_scaled.y *= w2;
            r2_scaled.z *= w2;

            //window.draw_line(&ps[i].pos, &(ps[i].pos + r2_scaled), &Vec3::new(0.0, 1.0, 0.0));

            // rule 3 : match velocity of nearby birds
            //let center_vel = (flock_total_vel - ps[i].vel) / (ps.len() as f32 - 1.0);
            //let r3_match_vel = na::normalize(&center_vel) - ps[i].vel;

            let r3_match_vel = ps.iter()
                .filter(|ref p| na::norm(&(p.pos - ps[i].pos)) < look_radius)
                .fold(na::zero::<Vec3<f32>>(), |a, ref p| a + p.vel);
            let mut r3_scaled =
                if center_count == 0 {
                    na::zero()
                } else {
                    r3_match_vel / center_count as f32
                };
            r3_scaled.x *= w3;
            r3_scaled.y *= w3;
            r3_scaled.z *= w3;

            //window.draw_line(&ps[i].pos, &(ps[i].pos + r3_scaled), &Vec3::new(0.0, 1.0, 1.0));

            ps.get_mut(i).acc = r1_scaled + r2_scaled + r3_scaled;
        }

        // step update
        for p in ps.mut_iter() {
            p.acc.x +=
                if p.pos.x > 50.0 {
                    -20.0
                } else if p.pos.x < -50.0 {
                    20.0
                } else {
                    p.acc.x
                };

            p.acc.z +=
                if p.pos.z > 50.0 {
                    -20.0
                } else if p.pos.z < -50.0 {
                    20.0
                } else {
                    p.acc.z
                };

            p.update(curr_time - last_time);
        }

        last_time = curr_time;
    }
}

fn draw_axis(w: &mut Window) {
    let o: Vec3<f32> = na::zero();
    let x = Vec3::x();
    let y = Vec3::y();
    let z = Vec3::z();

    w.draw_line(&o, &x, &x);
    w.draw_line(&o, &y, &y);
    w.draw_line(&o, &z, &z);
}
