extern crate nalgebra;

use utils::AABB;
use nalgebra::Vec3;

use super::{Boid, BoidId};

/// Wrapper to contain boid id in main boid pool.
pub struct OctnodeId(uint);

impl OctnodeId {
    /// Check to see if enclosed uint is not -1.  Reserving -1 to represent a null value
    pub fn is_set(&self) -> bool {
        let OctnodeId(id) = *self;
        id != -1
    }
}

/// Enum representing the Octnode's state.
/// - Empty: node hasn't been initialized
/// - Left: node contains a boid
/// - Node: node is an internal node
pub enum OctnodeState {
    Empty, // the root node is the only one that's ever empty [this is before the tree gets built]
    Leaf,
    Node,
}

/// A node in the Octree
pub struct Octnode {
    pub parent: OctnodeId, // TODO: not used right now
    pub child: [OctnodeId, ..8],
    boid: BoidId, // TODO: convert to vector

    pub b: AABB,
    pub state: OctnodeState,

    pub c: Vec3<f32>, // flock center
    pub v: Vec3<f32>, // flock direction [average, but not normalized]
}

impl Octnode {
    /// Creates an Octnode leaf with the given boid assigned to it.
    fn new(parent: OctnodeId, boid: BoidId, bbox: AABB, ps: &Vec<Boid>) -> Octnode {
        let BoidId(bid) = boid; // TODO: verify: in theory this fn will not be called without a valid plane

        Octnode {
            parent: parent,
            child: [OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1)],
            boid: boid,
            b: bbox,
            state: Leaf,
            c: ps[bid].pos,
            v: ps[bid].vel,
        }
    }

    /// Creates an empty node with a bounding box - this is used to initialize the root of a blank tree.
    fn empty(bbox: AABB) -> Octnode {
        Octnode {
            parent: OctnodeId(-1),
            child: [OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1)],
            boid: BoidId(-1),
            b: bbox,
            state: Empty,
            c: nalgebra::zero(),
            v: nalgebra::zero(),
        }
    }

    /// Calculate the width of the cube.
    #[inline(always)]
    pub fn width(&self) -> f32 {
        self.b.xlen()
    }

    /// Returns true if the node is a leaf.
    #[inline(always)]
    pub fn is_leaf(&self) -> bool {
        match self.state {
            Leaf => true,
            _ => false,
        }
    }
}

/// Octree data structure.  Uses a Vec<Octnode> as a memory pool to contain node data.
pub struct Octree {
    pub root: OctnodeId,
    pub pool: Vec<Octnode>,
}

impl Octree {
    /// Creates a new empty Octree with the given bounding box.
    pub fn new(bbox: AABB) -> Octree {
        let mut p = Vec::with_capacity(1 << 8);
        p.push(Octnode::empty(bbox)); // TODO: assign world bb to octree struct, then can remove empty checks

        Octree {
            root: OctnodeId(0),
            pool: p,
        }
    }

    /// Barnes-Hut update function.
    pub fn update(&mut self, ps: &Vec<Boid>) {
        let root = self.root;
        self.update_recur(root, ps);
    }

    /// Recursive Barnes-Hut update function.
    fn update_recur(&mut self, curr_oid: OctnodeId, ps: &Vec<Boid>) {
        let OctnodeId(curr_id) = curr_oid;
        let state = self.pool[curr_id].state;

        match state {
            Empty => { }
            Leaf => { } // TODO: verify: when leaf nodes are created, c and v are set. nodes are never converted to leaf state
            Node => {
                let mut c: Vec3<f32> = nalgebra::zero();
                let mut v: Vec3<f32> = nalgebra::zero();
                let mut active_children = 0u;

                for i in range(0, 8) {
                    let child_oid = self.pool[curr_id].child[i];

                    if child_oid.is_set() {
                        let OctnodeId(child_id) = child_oid;

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
                let o = self.pool.get_mut(curr_id); // update the node's averages
                o.c = c;
                o.v = v;
            }
        }
    }

    /// Resets the Octree.  Truncates the internal vector and pushes an empty root.
    pub fn reset(&mut self, bbox: AABB) {
        self.root = OctnodeId(0);
        self.pool.truncate(0); // 'empty' the vector, capacity stays the same
        self.pool.push(Octnode::empty(bbox));
    }

    /// Insert the boids into the Octree.
    pub fn insert(&mut self, ps: &Vec<Boid>) {
        let root = self.root;
        let root_aabb = self.pool[0].b;
        for i in range(0, ps.len()) {
            self.insert_recur(root, OctnodeId(-1), ps, BoidId(i), &root_aabb);
        }
    }

    /// Recursively insert boid to Octree.
    fn insert_recur(&mut self, curr_oid: OctnodeId, parent_oid: OctnodeId, ps: &Vec<Boid>, boid_bid: BoidId, bbox: &AABB) -> OctnodeId {
        if !curr_oid.is_set() {
            //println!("null node, pulling from pool");
            self.pool.push(Octnode::new(parent_oid, boid_bid, *bbox, ps));
            OctnodeId(self.pool.len() - 1)
        } else {
            let OctnodeId(curr_id) = curr_oid;

            match self.pool[curr_id].state {
                Empty => { // this only happens for the first insert case (root node)
                    *self.pool.get_mut(curr_id) = Octnode::new(parent_oid, boid_bid, *bbox, ps);
                    curr_oid
                }
                Leaf => {
                    let center = bbox.center();
                    let BoidId(bid) = boid_bid;

                    { // convert current node to internal node, and push boid to the correct child
                        let oldboid_bid = self.pool[curr_id].boid;
                        let BoidId(oldboid_id) = oldboid_bid;
                        let new_oct = get_octant(&ps[oldboid_id].pos, &center);
                        let new_bounds = gen_oct_bounds(new_oct, bbox, &center);

                        let child_oid = self.pool[curr_id].child[new_oct];
                        let new_child_oid = self.insert_recur(child_oid, curr_oid, ps, oldboid_bid, &new_bounds);

                        let on = self.pool.get_mut(curr_id);
                        on.child[new_oct] = new_child_oid;
                        on.boid = BoidId(-1);
                        on.state = Node;
                    }

                    let oct = get_octant(&ps[bid].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center);

                    let child_oid = self.pool[curr_id].child[oct];
                    let new_child_oid = self.insert_recur(child_oid, curr_oid, ps, boid_bid, &new_bounds);

                    self.pool.get_mut(curr_id).child[oct] = new_child_oid;
                    curr_oid
                }
                Node => {
                    let center = bbox.center();
                    let BoidId(bid) = boid_bid;
                    let oct = get_octant(&ps[bid].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center); // TODO: if child exists, can skip this calc and reference child's aabb

                    let child_oid = self.pool[curr_id].child[oct];
                    let new_child_oid = self.insert_recur(child_oid, curr_oid, ps, boid_bid, &new_bounds);

                    self.pool.get_mut(curr_id).child[oct] = new_child_oid;
                    curr_oid
                }
            }
        }
    }

    /// Print out some statistics on the Octree.  Currently just prints `pool.len()` and `pool.capacity()`.
    pub fn stats(&self) {
        println!("elem used: {}, capacity: {}", self.pool.len(), self.pool.capacity());
    }

    /// Return a reference to the Octnode object at index `oid`.
    pub fn get_node(&self, oid: OctnodeId) -> &Octnode {
        let OctnodeId(id) = oid;
        &self.pool[id]
    }
}

/// Calculate which octant the point goes to relative to center `c`
fn get_octant(p: &Vec3<f32>, c: &Vec3<f32>) -> uint {
    // TODO: make this labelling follow the same direction as morton sort
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

/// Generate aabb for new octant.
fn gen_oct_bounds(oct: uint, bbox: &AABB, center: &Vec3<f32>) -> AABB {
    let (lo, hi) = 
        match oct {
            0 => { (Vec3::new(bbox.l.x, bbox.l.y, center.z), Vec3::new(center.x, center.y, bbox.h.z)) }
            1 => { (Vec3::new(center.x, bbox.l.y, center.z), Vec3::new(bbox.h.x, center.y, bbox.h.z)) }
            2 => { (Vec3::new(bbox.l.x, center.y, center.z), Vec3::new(center.x, bbox.h.y, bbox.h.z)) }
            3 => { (*center, bbox.h) }
            4 => { (bbox.l, *center) }
            5 => { (Vec3::new(center.x, bbox.l.y, bbox.l.z), Vec3::new(bbox.h.x, center.y, center.z)) }
            6 => { (Vec3::new(bbox.l.x, center.y, bbox.l.z), Vec3::new(center.x, bbox.h.y, center.z)) }
            7 => { (Vec3::new(center.x, center.y, bbox.l.z), Vec3::new(bbox.h.x, bbox.h.y, center.z)) }
            _ => { (nalgebra::zero(), nalgebra::zero()) } // TODO: maybe make this a fail! ?
        };
    AABB::new(lo, hi)
}
