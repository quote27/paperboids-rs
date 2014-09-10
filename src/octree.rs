extern crate nalgebra;

use utils::AABB;
use nalgebra::na::Vec3;
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
    pub plane_id: PlaneId, //TODO: convert to vector

    pub b: AABB,
    pub state: OctnodeState,

    c: Vec3<f32>, // flock center
    v: Vec3<f32>, // flock direction [average, but not normalized]
}

impl Octnode {
    fn new(parent: OctnodeId, plane_id: PlaneId, bbox: AABB, ps: &Vec<Plane>) -> Octnode {
        let PlaneId(pid) = plane_id; //TODO: verify: in theory this fn will not be called without a valid plane
        let pid = pid as uint;

        Octnode {
            parent: parent,
            child: [OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1)],
            plane_id: plane_id,
            b: bbox,
            state: Leaf,
            c: ps[pid].pos,
            v: ps[pid].vel,
        }
    }

    fn empty(bbox: AABB) -> Octnode {
        Octnode {
            parent: OctnodeId(-1),
            child: [OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1)],
            plane_id: PlaneId(-1),
            b: bbox,
            state: Empty,
            c: na::zero(),
            v: na::zero(),
        }
    }

    #[inline(always)]
    fn width(&self) -> f32{
        self.b.xlen()
    }
}

// ---

pub struct Octree {
    root: OctnodeId,
    pub pool: Vec<Octnode>,
}

impl Octree {
    pub fn new(bbox: AABB) -> Octree {
        let mut p = Vec::with_capacity(1 << 8);
        p.push(Octnode::empty(bbox));

        Octree {
            root: OctnodeId(0),
            pool: p,
        }
    }

    pub fn update(&mut self, ps: &Vec<Plane>) {
        let root = self.root;
        self.update_recur(root, ps);
    }
    fn update_recur(&mut self, curr: OctnodeId, ps: &Vec<Plane>) {
        let OctnodeId(cid) = curr;
        let cid = cid as uint;
        let state = self.pool[cid].state;

        match state {
            Empty => { }
            Leaf => {
                // TODO: verify: when leaf nodes are created, c and v are set. nodes are never converted to leaf state
                // let PlaneId(pid) = self.pool[cid].plane_id;
                // let pid = pid as uint;
                // let o = self.pool.get_mut(cid);
                // o.c = ps[pid].pos;
                // o.v = ps[pid].vel;
            }
            Node => {
                let mut c: Vec3<f32> = na::zero();
                let mut v: Vec3<f32> = na::zero();
                let mut active_children = 0u;

                for i in range(0, 8) {
                    let child_oid = self.pool[cid].child[i];
                    let OctnodeId(child_id) = child_oid;
                    if child_id >= 0 {
                        let child_id = child_id as uint;

                        match self.pool[child_id].state {
                            Node => self.update_recur(child_oid, ps),
                            _ => { }
                        }

                        let o = self.pool[child_id];
                        c = c + o.c;
                        v = v + o.v;
                        active_children += 1;
                    }
                }
                //TODO: verify: if state is node, there has to be at least one child, so can't have a divide by 0
                c = c / active_children as f32;
                v = v / active_children as f32;
                let o = self.pool.get_mut(cid);
                o.c = c;
                o.v = v;
            }
        }
    }

    pub fn reset(&mut self, bbox: AABB) {
        self.root = OctnodeId(0);
        self.pool.truncate(0); // 'empty' the vector, capacity stays the same
        self.pool.push(Octnode::empty(bbox));
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
            self.pool.push(Octnode::new(parent, plane_id, *bbox, ps));
            OctnodeId(self.pool.len() as int - 1)
        } else {
            let cid = cid as uint;

            match self.pool[cid].state {
                Empty => {
                    //println!("empty node");
                    *self.pool.get_mut(cid) = Octnode::new(parent, plane_id, *bbox, ps);
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

    pub fn stats(&self) {
        println!("elem used: {}, capacity: {}", self.pool.len(), self.pool.capacity());
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
