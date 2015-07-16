extern crate cgmath;

use std::fmt;
use cgmath::{Point, Point3, Vector, Vector3};
use aabb::AABB;
use boids::Boid;


/// Enum representing the Octnode's state.
/// - Empty: node hasn't been initialized
/// - Left: node contains a boid
/// - Node: node is an internal node
#[derive(Clone, Copy)]
pub enum OctnodeState {
    Empty, // the root node is the only one that's ever empty [this is before the tree gets built]
    Leaf,
    Node,
}

/// A node in the Octree
pub struct Octnode {
    pub parent: usize, // TODO: not used right now
    pub child: [usize; 8],
    pub boid: usize, // TODO: convert to vector

    pub bbox: AABB,
    pub state: OctnodeState,

    pub c: Vector3<f32>, // flock center
    pub v: Vector3<f32>, // flock direction [average, but not normalized]
}

impl Octnode {
    /// Creates an Octnode leaf with the given boid assigned to it.
    fn new(parent: usize, bbox: AABB, boid_id: usize, b: &Boid) -> Octnode {
        Octnode {
            parent: parent,
            child: [-1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize],
            boid: boid_id,
            bbox: bbox,
            state: OctnodeState::Leaf,
            c: b.pos,
            v: b.vel,
        }
    }

    /// Creates an empty node with a bounding box - this is used to initialize the root of a blank tree.
    fn empty(bbox: AABB) -> Octnode {
        Octnode {
            parent: -1 as usize,
            child: [-1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize, -1 as usize],
            boid: -1 as usize,
            bbox: bbox,
            state: OctnodeState::Empty,
            c: cgmath::zero(),
            v: cgmath::zero(),
        }
    }

    /// Calculate the width of the cube.
    #[inline(always)]
    pub fn width(&self) -> f32 {
        self.bbox.xlen()
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
    pub root: usize,
    pub pool: Vec<Octnode>,
}

impl Octree {
    /// Creates a new empty Octree with the given bounding box.
    pub fn new(bbox: AABB) -> Octree {
        let mut p = Vec::with_capacity(1 << 8);
        p.push(Octnode::empty(bbox)); // TODO: assign world bb to octree struct, then can remove empty checks

        Octree {
            root: 0,
            pool: p,
        }
    }

    /// Barnes-Hut update function.
    pub fn update(&mut self, bs: &Vec<Boid>) {
        let root = self.root; // TODO: why can't we pass self.root as an argument?
        self.update_recur(root, bs);
    }

    /// Recursive Barnes-Hut update function.
    fn update_recur(&mut self, curr_id: usize, bs: &Vec<Boid>) {
        let state = self.pool[curr_id].state;

        match state {
            OctnodeState::Empty => { }
            OctnodeState::Leaf => { } // TODO: verify: when leaf nodes are created, c and v are set. nodes are never converted to leaf state
            OctnodeState::Node => {
                let mut c: Vector3<f32> = cgmath::zero();
                let mut v: Vector3<f32> = cgmath::zero();
                let mut active_children = 0u8;

                for i in 0..8 {
                    let child_id = self.pool[curr_id].child[i];

                    if child_id != -1 as usize {
                        //if self.pool[child_id].state == OctnodeState::Node {
                        //    self.update_recur(child_id, bs);
                        //}

                        let child_state = self.pool[child_id].state;
                        match child_state {
                            OctnodeState::Node => self.update_recur(child_id, bs),
                            _ => { }
                        }

                        let o = &self.pool[child_id];
                        c = c + o.c;
                        v = v + o.v;
                        active_children += 1;
                    }
                }
                //TODO: verify: if state is node, there has to be at least one child, so can't have a divide by 0
                c = c.div_s(active_children as f32);
                v = v.div_s(active_children as f32);
                let o = &mut self.pool[curr_id]; // update the node's averages
                o.c = c;
                o.v = v;
            }
        }
    }

    /// Resets the Octree.  Truncates the internal vector and pushes an empty root.
    pub fn reset(&mut self, bbox: AABB) {
        self.pool.truncate(0); // 'empty' the vector, capacity stays the same
        self.pool.push(Octnode::empty(bbox));
        self.root = 0;
    }

    /// Insert the boids into the Octree.
    pub fn insert(&mut self, bs: &Vec<Boid>) {
        let root = self.root;
        let root_bbox = self.pool[0].bbox;
        for i in 0..bs.len() {
            // println!("inserting {}: {}", i, bs[i]);
            self.insert_recur(root, -1 as usize, bs, i, &root_bbox, 0);
        }
    }

    /// Recursively insert boid to Octree.
    fn insert_recur(&mut self, curr_id: usize, parent_id: usize, bs: &Vec<Boid>, boid_id: usize, bbox: &AABB, recur: usize) -> usize {
        let mut space = String::with_capacity(2 * recur);
        for _ in 0..recur { space.push_str("  "); }
        // println!("{}ir: boid: {}, curr: {}, parent: {}", space, boid_id, curr_id, parent_id);

        if curr_id == -1 as usize {
            //println!("null node, pulling from pool");
            self.pool.push(Octnode::new(parent_id, *bbox, boid_id, &bs[boid_id]));

            // println!("{}  : curr oid is null, adding new octnode: {}", space, i32(self.pool.len() - 1));
            self.pool.len() - 1

        } else {
            match self.pool[curr_id].state {
                OctnodeState::Empty => { // this only happens for the first insert case (root node)
                    self.pool[curr_id] = Octnode::new(parent_id, *bbox, boid_id, &bs[boid_id]);

                    // println!("{}  : curr id is empty, this is root", space);
                    curr_id
                }
                OctnodeState::Leaf => {
                    let center = bbox.center();

                    { // convert current node to internal node, and push boid to the correct child
                        let oldboid_id = self.pool[curr_id].boid;

                        let new_oct = get_octant(&bs[oldboid_id].pos, &center);
                        let new_bounds = gen_oct_bounds(new_oct, bbox, &center);

                        let child_id = self.pool[curr_id].child[new_oct];

                        // println!("{}  : leaf a: old boid: {}, new oct: {}, traversal child: {}", space, oldboid_id, new_oct, child_id);

                        if boid_id == oldboid_id {
                            println!("{}  : error: current boid id and old boid id match: curr: {}, old: {}", space, boid_id, oldboid_id);
                            println!("\n\n");
                            self.print();
                            panic!("printing tree state and quitting");
                        }

                        let new_child_id = self.insert_recur(child_id, curr_id, bs, oldboid_id, &new_bounds, recur + 1);

                        // println!("{}  : leaf a: new child id: {}", space, new_child_id);

                        let on = &mut self.pool[curr_id];
                        on.child[new_oct] = new_child_id;
                        on.boid = -1;
                        on.state = OctnodeState::Node;
                    }

                    let oct = get_octant(&bs[boid_id].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center);

                    let child_id = self.pool[curr_id].child[oct];

                    // println!("{}  : leaf b: oct: {}, traversal child: {}", space, oct, child_id);

                    let new_child_id = self.insert_recur(child_id, curr_id, bs, boid_id, &new_bounds, recur +1);

                    // println!("{}  : leaf b: new child id: {}", space, new_child_id);

                    self.pool[curr_id].child[oct] = new_child_id;
                    curr_id
                }
                OctnodeState::Node => {
                    let center = bbox.center();
                    let oct = get_octant(&bs[boid_id].pos, &center);
                    let new_bounds = gen_oct_bounds(oct, bbox, &center); // TODO: if child exists, can skip this calc and reference child's aabb

                    let child_id = self.pool[curr_id].child[oct];

                    // println!("{}  : node: oct: {}, traversal child: {}", space, oct, child_id);

                    let new_child_id = self.insert_recur(child_id, curr_id, bs, boid_id, &new_bounds, recur + 1);

                    // println!("{}  : node: new child id: {}", space, new_child_id);

                    self.pool[curr_id].child[oct] = new_child_id;
                    curr_id
                }
            }
        }
    }

    /// Print out some statistics on the Octree.  Currently just prints `pool.len()` and `pool.capacity()`.
    pub fn stats(&self) {
        println!("elem used: {}, capacity: {}", self.pool.len(), self.pool.capacity());
    }

    /// Return a reference to the Octnode object at index `id`.
    pub fn get_node(&self, id: usize) -> &Octnode {
        &self.pool[id]
    }

    pub fn print(&self) {
        println!("octree print");
        self.print_recur(self.root, 0, 0);
    }

    fn print_recur(&self, curr_id: usize, oct: usize, recur: usize) {
        if curr_id == -1 as usize {
            return;
        }

        let mut space = String::with_capacity(2 * recur);
        for _ in 0..recur { space.push_str("  "); }

        println!("{}{} {}: {} b: {}", space, oct, curr_id,
                 match self.pool[curr_id].state {
                     OctnodeState::Empty => "e",
                     OctnodeState::Node => "n",
                     OctnodeState::Leaf => "l",
                 },
                 self.pool[curr_id].boid);

        for i in 0..8 {
            self.print_recur(self.pool[curr_id].child[i], i, recur + 1);
        }
    }
}

/// Calculate which octant the point goes to relative to center `c`
fn get_octant(p: &Vector3<f32>, c: &Vector3<f32>) -> usize {
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
fn gen_oct_bounds(oct: usize, bbox: &AABB, center: &Vector3<f32>) -> AABB {
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
