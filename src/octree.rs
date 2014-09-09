extern crate nalgebra;

use utils::{Timer, AABB};
use nalgebra::na::{Vec2, Vec3};
use nalgebra::na;

use super::{Plane, PlaneId};

struct OctnodeId(int);

enum OctnodeState {
    Empty,
    Leaf,
    Node,
}

struct Octnode {
    parent: OctnodeId,
    child: [OctnodeId, ..8],
    plane_id: PlaneId, //TODO: convert to vector

    b: AABB,
    state: OctnodeState,

    c: Vec3<f32>, // flock center
    v: Vec3<f32>, // flock direction [average, but not normalized]
}

impl Octnode {
    fn new(parent: OctnodeId, plane_id: PlaneId, bbox: AABB) -> Octnode {
        let state = match plane_id {
            PlaneId(-1) => Empty,
            _ => Leaf,
        };
        Octnode {
            parent: parent,
            child: [OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1)],
            plane_id: plane_id,
            b: bbox,
            state: state,
            c: na::zero(), //TODO: set these to plane's data
            v: na::zero(),
        }
    }

    fn update(&mut self, pool: &Vec<Octnode>) {
        // TODO: write this
    }

    #[inline(always)]
    fn width(&self) -> f32{
        self.b.xlen()
    }

    #[inline(always)]
    fn center(&self) -> Vec3<f32> {
        self.b.center()
    }
}

// ---

pub struct Octree {
    root: OctnodeId,
    pool: Vec<Octnode>,
}

impl Octree {
    pub fn new(bbox: AABB) -> Octree {
        let mut p = Vec::with_capacity(1 << 8);
        p.push(Octnode::new(OctnodeId(-1), PlaneId(-1), bbox));

        Octree {
            root: OctnodeId(0),
            pool: p,
        }
    }

    pub fn reset(&mut self, bbox: AABB) {
        self.root = OctnodeId(0);
        self.pool.truncate(0); // 'empty' the vector, capacity stays the same
        self.pool.push(Octnode::new(OctnodeId(-1), PlaneId(-1), bbox));
    }

    pub fn insert(&mut self, ps: &Vec<Plane>) {
        let root = self.root;
        let rootb = self.pool[0].b;
        for i in range(0, ps.len() as int) {
            self.insert_recur(root, OctnodeId(-1), ps, PlaneId(i), &rootb);
        }
    }

    fn insert_recur(&mut self, curr: OctnodeId, parent: OctnodeId, ps: &Vec<Plane>, plane_id: PlaneId, bbox: &AABB) -> OctnodeId {
        let OctnodeId(cid) = curr;
        //println!("cid: {}", cid);
        if cid == -1 {
            //println!("null node, pulling from pool");
            self.pool.push(Octnode::new(parent, plane_id, *bbox));
            OctnodeId(self.pool.len() as int - 1)
        } else {
            let cid = cid as uint;

            match self.pool[cid].state {
                Empty => {
                    //println!("empty node");
                    *self.pool.get_mut(cid) = Octnode::new(parent, plane_id, *bbox);
                    OctnodeId(cid as int)
                }
                Leaf => {
                    //println!("leaf node");
                    // TODO: finish
                    let center = bbox.center();
                    let PlaneId(pid) = plane_id;

                    { // convert to internal node and move self to a child
                        let plane_id = self.pool[cid].plane_id;
                        let PlaneId(pid) = plane_id;
                        let oct = get_octant(&ps[pid as uint].pos, &center);
                        let new_bounds = gen_oct_bounds(oct, bbox, &center);

                        let child_id = self.pool[cid].child[oct];
                        let new_child_oid = self.insert_recur(child_id, curr, ps, plane_id, &new_bounds);

                        let on = self.pool.get_mut(cid);
                        on.child[oct] = new_child_oid;
                        on.plane_id = PlaneId(-1);
                        on.state = Node;
                    }

                    let oct = get_octant(&ps[pid as uint].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center);

                    let child_id = self.pool[cid].child[oct];
                    let new_child_oid = self.insert_recur(child_id, curr, ps, plane_id, &new_bounds);

                    self.pool.get_mut(cid).child[oct] = new_child_oid;
                    curr
                }
                Node => {
                    //println!("internal node");
                    let center = bbox.center();
                    let PlaneId(pid) = plane_id;
                    let oct = get_octant(&ps[pid as uint].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center);

                    let child_id = self.pool[cid].child[oct];
                    let new_child_oid = self.insert_recur(child_id, curr, ps, plane_id, &new_bounds);

                    self.pool.get_mut(cid).child[oct] = new_child_oid;
                    curr
                }
            }
        }
    }

}

// TODO: make this labelling follow the same direction as morton sort
fn get_octant(p: &Vec3<f32>, c: &Vec3<f32>) -> uint {
    let mut oct = 0;

    if p.x >= c.x {
        oct |= 0x01;
    }
    if p.y >= c.y {
        oct |= 0x02;
    }
    if p.z < c.z {
        oct |= 0x04;
    }
    oct
}

fn gen_oct_bounds(oct: uint, bbox: &AABB, center: &Vec3<f32>) -> AABB {
    let (lo, hi) = 
        match oct {
            0 => {
                (Vec3::new(bbox.l.x, bbox.l.y, center.z),
                 Vec3::new(center.x, center.y, bbox.h.z))
            }
            1 => {
                (Vec3::new(center.x, bbox.l.y, center.z),
                 Vec3::new(bbox.h.x, center.y, bbox.h.z))
            }
            2 => {
                (Vec3::new(bbox.l.x, center.y, center.z),
                 Vec3::new(center.x, bbox.h.y, bbox.h.z))
            }
            3 => {
                (*center,
                 bbox.h)
            }
            4 => {
                (bbox.l,
                 *center)
            }
            5 => {
                (Vec3::new(center.x, bbox.l.y, bbox.l.z),
                 Vec3::new(bbox.h.x, center.y, center.z))
            }
            6 => {
                (Vec3::new(bbox.l.x, center.y, bbox.l.z),
                 Vec3::new(center.x, bbox.h.y, center.z))
            }
            7 => {
                (Vec3::new(center.x, center.y, bbox.l.z),
                 Vec3::new(bbox.h.x, bbox.h.y, center.z))
            }
            _ => {
                (na::zero(), na::zero())
            }
        };

    AABB::new(lo, hi)
}
