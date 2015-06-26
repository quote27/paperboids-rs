extern crate cgmath;

use utils::AABB;
use self::cgmath::{Point, Point3, Vector, Vector3};
use std::fmt;

use super::{Boid, BoidId};

/// Wrapper to contain boid id in main boid pool.
pub struct OctnodeId(u32);

#[deriving(Show)]
impl OctnodeId {
    /// Check to see if enclosed u32 is not -1.  Reserving -1 to represent a null value
    pub fn is_set(&self) -> bool {
        let OctnodeId(id) = *self;
        id != -1
    }
}

impl fmt::String for OctnodeId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let OctnodeId(id) = *self;
        let id = if id != -1 { id as i32 } else { -1 };
        write!(f, "{}", id)
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
    pub child: [OctnodeId; 8],
    boid: BoidId, // TODO: convert to vector

    pub b: AABB,
    pub state: OctnodeState,

    pub c: Point3<f32>, // flock center
    pub v: Vector3<f32>, // flock direction [average, but not normalized]
}

impl Octnode {
    /// Creates an Octnode leaf with the given boid assigned to it.
    fn new(parent: OctnodeId, boid: BoidId, bbox: AABB, ps: &[Boid]) -> Octnode {
        let BoidId(bid) = boid; // TODO: verify: in theory this fn will not be called without a valid plane

        Octnode {
            parent: parent,
            child: [OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1), OctnodeId(-1)],
            boid: boid,
            b: bbox,
            state: OctnodeState::Leaf,
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
            state: OctnodeState::Empty,
            c: cgmath::zero(),
            v: cgmath::zero(),
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
            OctnodeState::Leaf => true,
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
            OctnodeState::Empty => { }
            OctnodeState::Leaf => { } // TODO: verify: when leaf nodes are created, c and v are set. nodes are never converted to leaf state
            OctnodeState::Node => {
                let mut c: Vector3<f32> = cgmath::zero();
                let mut v: Vector3<f32> = cgmath::zero();
                let mut active_children = 0u32;

                for i in range(0, 8) {
                    let child_oid = self.pool[curr_id].child[i];

                    if child_oid.is_set() {
                        let OctnodeId(child_id) = child_oid;

                        match self.pool[child_id].state {
                            OctnodeState::Node => self.update_recur(child_oid, ps),
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
                let o = &mut self.pool[curr_id]; // update the node's averages
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
            // println!("inserting {}: {}", i, ps[i]);
            self.insert_recur(root, OctnodeId(-1), ps, BoidId(i), &root_aabb, 0);
            // self.print();
        }
    }

    /// Recursively insert boid to Octree.
    fn insert_recur(&mut self, curr_oid: OctnodeId, parent_oid: OctnodeId, ps: &Vec<Boid>, boid_bid: BoidId, bbox: &AABB, recur: u32) -> OctnodeId {
        let mut space = String::new();
        space.grow(recur * 2, ' ');
        // println!("{}ir: boid: {}, curr: {}, parent: {}", space, boid_bid, curr_oid, parent_oid);

        if !curr_oid.is_set() {
            //println!("null node, pulling from pool");
            self.pool.push(Octnode::new(parent_oid, boid_bid, *bbox, ps));

            // println!("{}  : curr oid is null, adding new octnode: {}", space, OctnodeId(self.pool.len() - 1));

            OctnodeId(self.pool.len() - 1)
        } else {
            let OctnodeId(curr_id) = curr_oid;

            match self.pool[curr_id].state {
                OctnodeState::Empty => { // this only happens for the first insert case (root node)
                    self.pool[curr_id] = Octnode::new(parent_oid, boid_bid, *bbox, ps);

                    // println!("{}  : curr oid is empty, this is root", space);

                    curr_oid
                }
                OctnodeState::Leaf => {
                    let center = bbox.center();
                    let BoidId(bid) = boid_bid;

                    { // convert current node to internal node, and push boid to the correct child
                        let oldboid_bid = self.pool[curr_id].boid;
                        let BoidId(oldboid_id) = oldboid_bid;

                        let new_oct = get_octant(&ps[oldboid_id].pos, &center);
                        let new_bounds = gen_oct_bounds(new_oct, bbox, &center);

                        let child_oid = self.pool[curr_id].child[new_oct];

                        // println!("{}  : leaf a: old boid: {}, new oct: {}, traversal child: {}", space, oldboid_bid, new_oct, child_oid);

                        if bid == oldboid_id {
                            println!("{}  : error: current boid id and old boid id match: curr: {}, old: {}", space, bid, oldboid_id);
                            println!("\n\n");
                            self.print();
                            panic!("printing tree state and quitting");
                        }

                        let new_child_oid = self.insert_recur(child_oid, curr_oid, ps, oldboid_bid, &new_bounds, recur + 1);

                        // println!("{}  : leaf a: new child oid: {}", space, new_child_oid);

                        let on = &mut self.pool[curr_id];
                        on.child[new_oct] = new_child_oid;
                        on.boid = BoidId(-1);
                        on.state = OctnodeState::Node;
                    }

                    let oct = get_octant(&ps[bid].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center);

                    let child_oid = self.pool[curr_id].child[oct];

                    // println!("{}  : leaf b: oct: {}, traversal child: {}", space, oct, child_oid);

                    let new_child_oid = self.insert_recur(child_oid, curr_oid, ps, boid_bid, &new_bounds, recur +1);

                    // println!("{}  : leaf b: new child oid: {}", space, new_child_oid);

                    self.pool[curr_id].child[oct] = new_child_oid;
                    curr_oid
                }
                OctnodeState::Node => {
                    let center = bbox.center();
                    let BoidId(bid) = boid_bid;
                    let oct = get_octant(&ps[bid].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center); // TODO: if child exists, can skip this calc and reference child's aabb

                    let child_oid = self.pool[curr_id].child[oct];

                    // println!("{}  : node: oct: {}, traversal child: {}", space, oct, child_oid);

                    let new_child_oid = self.insert_recur(child_oid, curr_oid, ps, boid_bid, &new_bounds, recur + 1);

                    // println!("{}  : node: new child oid: {}", space, new_child_oid);

                    self.pool[curr_id].child[oct] = new_child_oid;
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

    pub fn print(&self) {
        println!("octree print");
        self.print_recur(self.root, 0, 0);
    }

    fn print_recur(&self, curr_oid: OctnodeId, oct: u32, recur: u32) {
        if !curr_oid.is_set() {
            return;
        }

        let mut space = String::new();
        space.grow(recur * 2, ' ');

        let OctnodeId(curr_id) = curr_oid;
        println!("{}{} {}: {} b: {}", space, oct, curr_oid,
                 match self.pool[curr_id].state {
                     OctnodeState::Empty => "e",
                     OctnodeState::Node => "n",
                     OctnodeState::Leaf => "l",
                 },
                 self.pool[curr_id].boid);

        for i in range(0, 8) {
            self.print_recur(self.pool[curr_id].child[i], i, recur + 1);
        }
    }
}

/// Calculate which octant the point goes to relative to center `c`
fn get_octant(p: &Vector3<f32>, c: &Vector3<f32>) -> u32 {
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
fn gen_oct_bounds(oct: u32, bbox: &AABB, center: &Vector3<f32>) -> AABB {
    let (lo, hi) = 
        match oct {
            0 => { (Vector3::new(bbox.l.x, bbox.l.y, center.z), Vector3::new(center.x, center.y, bbox.h.z)) }
            1 => { (Vector3::new(center.x, bbox.l.y, center.z), Vector3::new(bbox.h.x, center.y, bbox.h.z)) }
            2 => { (Vector3::new(bbox.l.x, center.y, center.z), Vector3::new(center.x, bbox.h.y, bbox.h.z)) }
            3 => { (*center, bbox.h) }
            4 => { (bbox.l, *center) }
            5 => { (Vector3::new(center.x, bbox.l.y, bbox.l.z), Vector3::new(bbox.h.x, center.y, center.z)) }
            6 => { (Vector3::new(bbox.l.x, center.y, bbox.l.z), Vector3::new(center.x, bbox.h.y, center.z)) }
            7 => { (Vector3::new(center.x, center.y, bbox.l.z), Vector3::new(bbox.h.x, bbox.h.y, center.z)) }
            _ => { (cgmath::zero(), cgmath::zero()) } // TODO: maybe make this a fail! ?
        };
    AABB::new(lo, hi)
}
