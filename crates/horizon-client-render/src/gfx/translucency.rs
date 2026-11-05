//! Vanilla 26.3 MeshData.SortState / VertexSorting.byDistance.
//! Keep vertex order intact; only the quad indices change as the camera moves.
//!
//! A section whose quads cannot be seen through one another needs no sort at
//! all; one whose quads all face along a single axis sorts once by that axis,
//! each quad from the far end of the side it faces; and one whose quads only
//! ever see through each other one way sorts once by a topological order of
//! that relation. Remaining meshes can use a partition tree, which sorts the
//! separated groups by camera side and only distance-sorts unresolved leaves.
//! Classifying a mesh when it is uploaded saves resorting it every time the
//! camera crosses a block boundary. See [`classify`].

use crate::mesher::TerrainVertex;
use glam::Vec3;

/// MeshData.decodeQuadCentroids uses the opposite corners, not all four vertices.
pub fn quad_center(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| (a[i] + b[i]) / 2.0)
}

/// A face normal this close to an axis is that axis: the mesher emits model
/// UVs in exact sixteenths, but positions come from float arithmetic.
const AXIS_EPSILON: f32 = 1.0e-3;
/// Two planes this close are the same plane.
const PLANE_EPSILON: f32 = 1.0e-4;

/// A quad's supporting plane, in the mesh's own (section-relative) space.
struct Plane {
    verts: [[f32; 3]; 4],
    normal: Vec3,
    distance: f32,
    /// The axis the normal lies along and whether it points the positive way,
    /// or `None` for a sloped quad.
    facing: Option<(usize, bool)>,
}

/// How many quads [`classify`] will try a topological sort on: its predicate
/// tests every pair, so the cost grows with the square of the count.
const TOPO_QUADS: usize = 64;
/// A vertex this far past a plane counts as beyond it.
const HALF_SPACE_EPSILON: f32 = 1.0e-5;
/// Bound upload-time tree construction. Larger or poorly partitionable meshes
/// continue to use the per-view camera-distance sort.
const PARTITION_TREE_MAX_QUADS: usize = 4096;
const PARTITION_TREE_MAX_DEPTH: usize = 32;

#[derive(Clone, Copy)]
struct QuadBounds {
    min: [f32; 3],
    max: [f32; 3],
}

impl QuadBounds {
    fn of(quad: &[TerrainVertex]) -> Self {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for vertex in quad {
            for axis in 0..3 {
                min[axis] = min[axis].min(vertex.pos[axis]);
                max[axis] = max[axis].max(vertex.pos[axis]);
            }
        }
        Self { min, max }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Cut {
    axis: usize,
    coordinate: f32,
}

// Cuts are built only from finite mesh coordinates, so float equality is
// reflexive for every value admitted to this private type.
impl Eq for Cut {}

#[derive(Clone, Debug, PartialEq, Eq)]
enum PartitionNode {
    Leaf(Vec<u32>),
    Cut {
        cut: usize,
        plane: Vec<u32>,
        lower: Box<Self>,
        upper: Box<Self>,
    },
}

/// Geometry-based translucent ordering. Safe cuts are kept in preorder so a
/// sort state can cheaply detect when the camera crosses any ordering plane.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PartitionTree {
    root: PartitionNode,
    cuts: Vec<Cut>,
}

#[derive(Clone, Copy)]
struct Split {
    cut: Cut,
    plane: bool,
    balance: usize,
}

#[derive(Clone, Copy)]
struct Interval {
    min: f32,
    max: f32,
    quad: u32,
}

struct Component {
    start: usize,
    end: usize,
    max: f32,
    plane: bool,
    coordinate: f32,
}

impl PartitionTree {
    fn build(bounds: &[QuadBounds]) -> Option<Self> {
        if bounds.len() < 2 || bounds.len() > PARTITION_TREE_MAX_QUADS {
            return None;
        }

        let quads = (0..bounds.len() as u32).collect();
        let mut cuts = Vec::new();
        let root = Self::build_node(quads, bounds, &mut cuts, 0);
        (!cuts.is_empty()).then_some(Self { root, cuts })
    }

    fn build_node(
        quads: Vec<u32>,
        bounds: &[QuadBounds],
        cuts: &mut Vec<Cut>,
        depth: usize,
    ) -> PartitionNode {
        if depth >= PARTITION_TREE_MAX_DEPTH {
            return PartitionNode::Leaf(quads);
        }
        let Some(split) = best_split(&quads, bounds) else {
            return PartitionNode::Leaf(quads);
        };

        let axis = split.cut.axis;
        let coordinate = split.cut.coordinate;
        let mut lower = Vec::new();
        let mut plane = Vec::new();
        let mut upper = Vec::new();
        for quad in quads.iter().copied() {
            let bound = bounds[quad as usize];
            if split.plane && bound.min[axis] == coordinate && bound.max[axis] == coordinate {
                plane.push(quad);
            } else if bound.max[axis] < coordinate - PLANE_EPSILON {
                lower.push(quad);
            } else if bound.min[axis] > coordinate + PLANE_EPSILON {
                upper.push(quad);
            } else {
                // Candidate selection only admits cuts with a clear gap or
                // an exactly coplanar component. Keep this guard conservative
                // if floating-point bounds ever disagree with that analysis.
                return PartitionNode::Leaf(quads);
            }
        }
        if lower.is_empty() || upper.is_empty() {
            return PartitionNode::Leaf(quads);
        }

        let cut_index = cuts.len();
        cuts.push(split.cut);
        let lower = Box::new(Self::build_node(lower, bounds, cuts, depth + 1));
        let upper = Box::new(Self::build_node(upper, bounds, cuts, depth + 1));
        PartitionNode::Cut {
            cut: cut_index,
            plane,
            lower,
            upper,
        }
    }

    /// Updates the camera-side signature for every cut. A change means at
    /// least one subtree pair changed relative ordering.
    fn update_sides(&self, camera: [f64; 3], origin: [i32; 3], sides: &mut Vec<i8>) -> bool {
        let mut changed = sides.len() != self.cuts.len();
        sides.resize(self.cuts.len(), 0);
        for (index, cut) in self.cuts.iter().enumerate() {
            let relative = (camera[cut.axis] - f64::from(origin[cut.axis])) as f32;
            let side = if relative < cut.coordinate {
                -1
            } else if relative > cut.coordinate {
                1
            } else {
                0
            };
            if sides[index] != side {
                sides[index] = side;
                changed = true;
            }
        }
        changed
    }

    fn emit(
        &self,
        centers: &[[f32; 3]],
        camera: [f32; 3],
        order: &mut Vec<(u32, f32)>,
        indices: &mut Vec<u32>,
    ) {
        self.root.emit(&self.cuts, centers, camera, order, indices);
    }
}

fn best_split(quads: &[u32], bounds: &[QuadBounds]) -> Option<Split> {
    let mut best = None;
    for axis in 0..3 {
        let mut intervals: Vec<_> = quads
            .iter()
            .map(|&quad| {
                let bound = bounds[quad as usize];
                Interval {
                    min: bound.min[axis],
                    max: bound.max[axis],
                    quad,
                }
            })
            .collect();
        intervals.sort_by(|a, b| {
            a.min
                .total_cmp(&b.min)
                .then_with(|| a.max.total_cmp(&b.max))
                .then_with(|| a.quad.cmp(&b.quad))
        });

        let mut components = Vec::new();
        let mut start = 0;
        let mut component_max = intervals[0].max;
        for index in 1..=intervals.len() {
            if index < intervals.len()
                && intervals[index].min - component_max <= 2.0 * PLANE_EPSILON
            {
                component_max = component_max.max(intervals[index].max);
                continue;
            }

            let coordinate = intervals[start].min;
            let plane = intervals[start..index]
                .iter()
                .all(|interval| interval.min == coordinate && interval.max == coordinate);
            components.push(Component {
                start,
                end: index,
                max: component_max,
                plane,
                coordinate,
            });
            if index < intervals.len() {
                start = index;
                component_max = intervals[index].max;
            }
        }

        let mut lower_count = 0;
        for (index, component) in components.iter().enumerate() {
            if component.plane && index > 0 && index + 1 < components.len() {
                consider_split(
                    &mut best,
                    Split {
                        cut: Cut {
                            axis,
                            coordinate: component.coordinate,
                        },
                        plane: true,
                        balance: lower_count.min(quads.len() - component.end),
                    },
                );
            }

            lower_count = component.end;
            if index + 1 < components.len() {
                let next = &components[index + 1];
                let lower_count = component.end;
                let upper_count = quads.len() - lower_count;
                let coordinate = component.max + (intervals[next.start].min - component.max) * 0.5;
                consider_split(
                    &mut best,
                    Split {
                        cut: Cut { axis, coordinate },
                        plane: false,
                        balance: lower_count.min(upper_count),
                    },
                );
            }
        }
    }
    best
}

fn consider_split(best: &mut Option<Split>, candidate: Split) {
    if candidate.balance == 0 {
        return;
    }
    if best.is_none_or(|known| candidate.balance > known.balance) {
        *best = Some(candidate);
    }
}

impl PartitionNode {
    fn emit(
        &self,
        cuts: &[Cut],
        centers: &[[f32; 3]],
        camera: [f32; 3],
        order: &mut Vec<(u32, f32)>,
        indices: &mut Vec<u32>,
    ) {
        match self {
            Self::Leaf(quads) => sort_group(quads, centers, camera, order, indices),
            Self::Cut {
                cut,
                plane,
                lower,
                upper,
            } => {
                let cut = cuts[*cut];
                match camera[cut.axis].total_cmp(&cut.coordinate) {
                    std::cmp::Ordering::Less => {
                        upper.emit(cuts, centers, camera, order, indices);
                        sort_group(plane, centers, camera, order, indices);
                        lower.emit(cuts, centers, camera, order, indices);
                    }
                    std::cmp::Ordering::Greater => {
                        lower.emit(cuts, centers, camera, order, indices);
                        sort_group(plane, centers, camera, order, indices);
                        upper.emit(cuts, centers, camera, order, indices);
                    }
                    std::cmp::Ordering::Equal => {
                        upper.emit(cuts, centers, camera, order, indices);
                        lower.emit(cuts, centers, camera, order, indices);
                        sort_group(plane, centers, camera, order, indices);
                    }
                }
            }
        }
    }
}

fn sort_group(
    quads: &[u32],
    centers: &[[f32; 3]],
    camera: [f32; 3],
    order: &mut Vec<(u32, f32)>,
    indices: &mut Vec<u32>,
) {
    order.clear();
    order.extend(quads.iter().map(|&quad| {
        let center = centers[quad as usize];
        let [x, y, z] = std::array::from_fn(|axis| camera[axis] - center[axis]);
        (quad, x * x + y * y + z * z)
    }));
    order.sort_by(|a, b| b.1.total_cmp(&a.1));
    for &(quad, _) in order.iter() {
        push_quad(indices, quad);
    }
}

/// How often a section's translucent quads must be re-sorted as the camera
/// moves.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Order {
    /// No two quads can be seen through one another, so every order blends
    /// the same. The mesh's own order is written once and kept.
    AnyOrder,
    /// Every quad faces along `axis`. `signs` holds one entry per quad, `+1`
    /// when it faces the positive side of the axis and `-1` the other: the
    /// fixed order is by `signs[q] * coord[axis]`, the far end of the side
    /// each quad faces. No camera changes it, so one sort is enough.
    Static { axis: usize, signs: Vec<i8> },
    /// `order` is a back-to-front order of the quads found as a topological
    /// order of the see-through relation, so it holds for every camera and is
    /// written once. It covers what `Static` cannot: quads on more than one
    /// axis, and sloped quads.
    StaticTopo { order: Vec<u32> },
    /// A camera-dependent partition tree. Separated groups are emitted in
    /// back-to-front order; unresolved leaves keep vanilla's distance sort.
    PartitionTree(PartitionTree),
    /// The order depends on the camera: vanilla's per-viewpoint sort.
    Dynamic,
}

/// Classifies a translucent mesh from its vertices (four per quad, in the
/// mesher's winding).
///
/// The result is conservative: `AnyOrder`, `Static` and `StaticTopo` are
/// returned only when the geometry proves them; remaining partitionable
/// geometry uses `PartitionTree`, and everything else stays `Dynamic`. A
/// `Static` order is by the axis the quads face, never by the
/// camera: vanilla's camera-distance order for two parallel panes flips as
/// the eye crosses them, which is the mistake a camera-relative one-time sort
/// would make.
pub fn classify(vertices: &[TerrainVertex]) -> Order {
    let quads = vertices.as_chunks::<4>().0;

    if !vertices.len().is_multiple_of(4)
        || vertices
            .iter()
            .any(|vertex| vertex.pos.iter().any(|coordinate| !coordinate.is_finite()))
    {
        return Order::Dynamic;
    }
    if quads.len() <= 1 {
        return Order::AnyOrder;
    }

    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for vertex in vertices {
        for axis in 0..3 {
            min[axis] = min[axis].min(vertex.pos[axis]);
            max[axis] = max[axis].max(vertex.pos[axis]);
        }
    }

    let mut planes = Vec::with_capacity(quads.len());
    let mut bounds = Vec::with_capacity(quads.len());
    let mut axis_seen: Option<usize> = None;
    let mut same_axis = true;
    let mut all_on_hull = true;
    let mut signs = Vec::with_capacity(quads.len());

    for quad in quads {
        let Some(plane) = plane_of(quad) else {
            return Order::Dynamic;
        };
        bounds.push(QuadBounds::of(quad));

        match plane.facing {
            Some((axis, positive)) => {
                match axis_seen {
                    None => axis_seen = Some(axis),
                    Some(known) if known == axis => {}
                    Some(_) => same_axis = false,
                }
                signs.push(if positive { 1 } else { -1 });

                // The box's extreme along the axis is its farthest face.
                let extreme = if positive { max[axis] } else { -min[axis] };
                if (plane.distance - extreme).abs() > PLANE_EPSILON {
                    all_on_hull = false;
                }
            }
            // A sloped quad is on no axis-aligned hull face, and it makes the
            // mesh too mixed for the axis-ordered class.
            None => {
                all_on_hull = false;
                same_axis = false;
            }
        }

        planes.push(plane);
    }

    if all_on_hull {
        // Every quad lies on the surface of the box of all vertices: the
        // surface of a convex body. Its faces cannot be seen through one
        // another, and neither can coplanar quads.
        return Order::AnyOrder;
    }

    if same_axis {
        // Every quad faces along one axis, either way. A quad facing the
        // positive side is seen only with the eye beyond it, so its order
        // along the axis is fixed; a quad facing the other way is seen from
        // the other side, so its order is that one reversed. The two sets are
        // never both in front of the eye, so how they interleave does not
        // matter.
        let Some(axis) = axis_seen else {
            return Order::Dynamic;
        };
        return Order::Static { axis, signs };
    }

    if planes.len() <= TOPO_QUADS
        && let Some(order) = topological_order(&planes)
    {
        return Order::StaticTopo { order };
    }

    PartitionTree::build(&bounds).map_or(Order::Dynamic, Order::PartitionTree)
}

/// The supporting plane of a quad, or `None` when the quad is degenerate.
fn plane_of(quad: &[TerrainVertex]) -> Option<Plane> {
    let a = Vec3::from_array(quad[0].pos);
    let b = Vec3::from_array(quad[1].pos);
    let c = Vec3::from_array(quad[2].pos);

    let normal = (b - a).cross(c - a);
    let length = normal.length();
    if !length.is_finite() || length <= 1.0e-6 {
        return None;
    }
    let normal = normal / length;

    Some(Plane {
        verts: [quad[0].pos, quad[1].pos, quad[2].pos, quad[3].pos],
        normal,
        distance: normal.dot(a),
        facing: axis_of(normal),
    })
}

/// The axis a unit normal lies along and whether it points the positive way,
/// or `None` when it is sloped.
fn axis_of(normal: Vec3) -> Option<(usize, bool)> {
    let mut direction = None;
    for axis in 0..3 {
        let component = normal[axis];
        if component.abs() >= 1.0 - AXIS_EPSILON {
            if direction.is_some() {
                return None;
            }
            direction = Some((axis, component > 0.0));
        } else if component.abs() > AXIS_EPSILON {
            return None;
        }
    }
    direction
}

/// Whether `b` could be seen through `a`: `a` before `b` with both facing the
/// eye, and `b` beyond `a`'s plane.
///
/// This is a superset of the real relation, which is what keeps a topological
/// order of it sound: linking a pair that cannot really see through each other
/// only constrains the order further, and a cycle then falls back to
/// `Dynamic`. It must never miss a pair, so its tests are the necessary
/// conditions only.
fn sees_through(a: &Plane, b: &Plane) -> bool {
    if let (Some((axis, positive)), Some((other_axis, other_positive))) = (a.facing, b.facing)
        && axis == other_axis
        && positive != other_positive
    {
        // Opposite faces of one axis look away from each other.
        return false;
    }

    // Some of `b` must lie behind `a`'s plane, or an eye in front of `a`
    // cannot reach `b` through it.
    if !b
        .verts
        .iter()
        .any(|v| a.normal.dot(Vec3::from_array(*v)) < a.distance - HALF_SPACE_EPSILON)
    {
        return false;
    }

    // Some of `a` must lie in front of `b`'s plane, or `b` is the one in
    // front and the order is the other way round.
    a.verts
        .iter()
        .any(|v| b.normal.dot(Vec3::from_array(*v)) > b.distance + HALF_SPACE_EPSILON)
}

/// A back-to-front order of the quads from the see-through relation, or
/// `None` when the relation has a cycle: then no fixed order is correct.
///
/// Depth-first, writing each quad's index as it is finished, so a quad is
/// written before everything that can be seen through it.
fn topological_order(quads: &[Plane]) -> Option<Vec<u32>> {
    let count = quads.len();
    let mut unvisited = vec![true; count];
    let mut on_stack = vec![false; count];
    let mut next_edge = vec![0usize; count];
    let mut stack: Vec<usize> = Vec::with_capacity(count);
    let mut order = Vec::with_capacity(count);
    let mut visited = 0usize;

    while visited < count {
        let Some(root) = (0..count).find(|&quad| unvisited[quad]) else {
            break;
        };
        stack.push(root);
        on_stack[root] = true;
        next_edge[stack.len() - 1] = 0;

        while let Some(&current) = stack.last() {
            let mut next = None;
            for edge in next_edge[stack.len() - 1]..count {
                if edge != current && unvisited[edge] && sees_through(&quads[current], &quads[edge])
                {
                    next = Some(edge);
                    next_edge[stack.len() - 1] = edge + 1;
                    break;
                }
            }

            match next {
                Some(edge) => {
                    if on_stack[edge] {
                        // A cycle: `edge` is in front of `current` and
                        // behind it at once.
                        return None;
                    }
                    stack.push(edge);
                    on_stack[edge] = true;
                    next_edge[stack.len() - 1] = 0;
                }
                None => {
                    // No edges left: nothing is in front of it, so it comes
                    // first of those left, and the order is built back to
                    // front.
                    on_stack[current] = false;
                    unvisited[current] = false;
                    visited += 1;
                    order.push(current as u32);
                    stack.pop();
                }
            }
        }
    }

    Some(order)
}

#[derive(Default)]
pub struct Sorter {
    order: Vec<(u32, f32)>,
    indices: Vec<u32>,
}

impl Sorter {
    pub fn sort(&mut self, centers: &[[f32; 3]], camera: [f32; 3]) -> &[u32] {
        self.order.clear();
        self.order.extend(centers.iter().enumerate().map(|(i, c)| {
            let [x, y, z] = std::array::from_fn(|j| camera[j] - c[j]);
            (i as u32, x * x + y * y + z * z)
        }));
        // VertexSorting uses a stable merge sort, preserving mesh order for ties.
        self.order.sort_by(|a, b| b.1.total_cmp(&a.1));
        self.indices.clear();
        for &(quad, _) in &self.order {
            push_quad(&mut self.indices, quad);
        }
        &self.indices
    }

    /// Traverses safe partition planes from far to near, sorting only the
    /// unresolved groups by the same center distances as the dynamic path.
    pub fn sort_partitioned(
        &mut self,
        centers: &[[f32; 3]],
        camera: [f32; 3],
        tree: &PartitionTree,
    ) -> &[u32] {
        self.indices.clear();
        tree.emit(centers, camera, &mut self.order, &mut self.indices);
        &self.indices
    }

    /// The fixed order for quads that all face along `axis`: by
    /// `signs[q] * coord[axis]`, so each quad comes from the far end of the
    /// side it faces. Stable, so ties keep the mesh's order.
    pub fn sort_static(&mut self, centers: &[[f32; 3]], axis: usize, signs: &[i8]) -> &[u32] {
        self.order.clear();
        self.order.extend(
            centers
                .iter()
                .enumerate()
                .map(|(i, c)| (i as u32, f32::from(signs[i]) * c[axis])),
        );
        self.order.sort_by(|a, b| a.1.total_cmp(&b.1));
        self.indices.clear();
        for &(quad, _) in &self.order {
            push_quad(&mut self.indices, quad);
        }
        &self.indices
    }

    /// The order `StaticTopo` found, as an index list: written once, then kept.
    pub fn permutation(&mut self, order: &[u32]) -> &[u32] {
        self.indices.clear();
        self.indices.reserve(order.len() * 6);
        for &quad in order {
            push_quad(&mut self.indices, quad);
        }
        &self.indices
    }

    /// The mesh's own quad order, for a section that needs no sorting.
    pub fn identity(&mut self, quads: usize) -> &[u32] {
        self.indices.clear();
        self.indices.reserve(quads * 6);
        for quad in 0..quads as u32 {
            push_quad(&mut self.indices, quad);
        }
        &self.indices
    }
}

fn push_quad(indices: &mut Vec<u32>, quad: u32) {
    let v = quad * 4;
    indices.extend([v, v + 1, v + 2, v + 2, v + 3, v]);
}

pub struct SortState {
    centers: Vec<[f32; 3]>,
    order: Order,
    last: Option<([i32; 3], [i32; 3])>,
    tree_sides: Vec<i8>,
}

impl SortState {
    pub fn new(centers: Vec<[f32; 3]>, order: Order) -> Self {
        Self {
            centers,
            order,
            last: None,
            tree_sides: Vec::new(),
        }
    }

    pub fn is_sorted(&self) -> bool {
        self.last.is_some()
    }

    /// Large tiles cannot use the section viewpoint shortcut: the camera may
    /// cross a quad's plane while remaining on the same side of the first 16^3.
    pub fn update_lod<'a>(
        &mut self,
        camera: [f64; 3],
        origin: [i32; 3],
        sorter: &'a mut Sorter,
    ) -> Option<&'a [u32]> {
        match self.order {
            Order::AnyOrder => self.write_identity(sorter),
            Order::Static { axis, ref signs } => {
                if self.last.is_some() {
                    return None;
                }
                self.last = Some((block(camera), [0; 3]));
                Some(sorter.sort_static(&self.centers, axis, signs))
            }
            Order::StaticTopo { ref order } => {
                if self.last.is_some() {
                    return None;
                }
                self.last = Some(([0; 3], [0; 3]));
                Some(sorter.permutation(order))
            }
            Order::PartitionTree(ref tree) => {
                let block = block(camera);
                let cut_changed = tree.update_sides(camera, origin, &mut self.tree_sides);
                let camera_changed = self.last.is_none_or(|(last_block, _)| last_block != block);
                if !camera_changed && !cut_changed {
                    return None;
                }
                self.last = Some((block, [0; 3]));
                Some(sorter.sort_partitioned(&self.centers, relative(camera, origin), tree))
            }
            Order::Dynamic => self.update_dynamic_lod(camera, origin, sorter),
        }
    }

    /// TranslucencyPointOfView and ResortTransparencyTask: diagonal views keep
    /// their ordering until the point of view changes; aligned views also
    /// resort on crossing a block boundary. Sub-block movement needs no upload.
    pub fn update<'a>(
        &mut self,
        camera: [f64; 3],
        origin: [i32; 3],
        sorter: &'a mut Sorter,
    ) -> Option<&'a [u32]> {
        match self.order {
            Order::AnyOrder => self.write_identity(sorter),
            Order::Static { axis, ref signs } => {
                if self.last.is_some() {
                    return None;
                }
                self.last = Some((block(camera), [0; 3]));
                Some(sorter.sort_static(&self.centers, axis, signs))
            }
            Order::StaticTopo { ref order } => {
                if self.last.is_some() {
                    return None;
                }
                self.last = Some(([0; 3], [0; 3]));
                Some(sorter.permutation(order))
            }
            Order::PartitionTree(ref tree) => {
                let block = block(camera);
                let pov =
                    std::array::from_fn(|i| ((block[i] >> 4) - (origin[i] >> 4)).clamp(-1, 1));
                let cut_changed = tree.update_sides(camera, origin, &mut self.tree_sides);
                let camera_changed = self.last.is_none_or(|(last_block, last_pov)| {
                    last_pov != pov || (last_block != block && pov.contains(&0))
                });
                if !camera_changed && !cut_changed {
                    return None;
                }
                self.last = Some((block, pov));
                Some(sorter.sort_partitioned(&self.centers, relative(camera, origin), tree))
            }
            Order::Dynamic => self.update_dynamic(camera, origin, sorter),
        }
    }

    fn write_identity<'a>(&mut self, sorter: &'a mut Sorter) -> Option<&'a [u32]> {
        if self.last.is_some() {
            return None;
        }
        self.last = Some(([0; 3], [0; 3]));
        Some(sorter.identity(self.centers.len()))
    }

    fn update_dynamic_lod<'a>(
        &mut self,
        camera: [f64; 3],
        origin: [i32; 3],
        sorter: &'a mut Sorter,
    ) -> Option<&'a [u32]> {
        let block = block(camera);
        if self.last.is_some_and(|(last, _)| last == block) {
            return None;
        }
        self.last = Some((block, [0; 3]));
        Some(sorter.sort(&self.centers, relative(camera, origin)))
    }

    fn update_dynamic<'a>(
        &mut self,
        camera: [f64; 3],
        origin: [i32; 3],
        sorter: &'a mut Sorter,
    ) -> Option<&'a [u32]> {
        let block = block(camera);
        let pov = std::array::from_fn(|i| ((block[i] >> 4) - (origin[i] >> 4)).clamp(-1, 1));
        if let Some((last_block, last_pov)) = self.last
            && last_pov == pov
            && (last_block == block || !pov.contains(&0))
        {
            return None;
        }
        self.last = Some((block, pov));
        Some(sorter.sort(&self.centers, relative(camera, origin)))
    }
}

fn block(camera: [f64; 3]) -> [i32; 3] {
    camera.map(|v| v.floor() as i32)
}

fn relative(camera: [f64; 3], origin: [i32; 3]) -> [f32; 3] {
    std::array::from_fn(|i| (camera[i] - f64::from(origin[i])) as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quad(pos: [[f32; 3]; 4]) -> [TerrainVertex; 4] {
        pos.map(|pos| TerrainVertex {
            pos,
            uv: [0; 2],
            layer: 0,
            light: 0,
            color: u32::MAX,
        })
    }

    /// A quad in the plane `z = z`, facing +Z (counter-clockwise seen from +Z).
    fn facing_pos_z(z: f32) -> [TerrainVertex; 4] {
        quad([[0.0, 0.0, z], [1.0, 0.0, z], [1.0, 1.0, z], [0.0, 1.0, z]])
    }

    /// The same plane, wound the other way: facing -Z.
    fn facing_neg_z(z: f32) -> [TerrainVertex; 4] {
        quad([[0.0, 0.0, z], [0.0, 1.0, z], [1.0, 1.0, z], [1.0, 0.0, z]])
    }

    /// A quad in the plane `x = x`, facing +X.
    fn facing_pos_x(x: f32) -> [TerrainVertex; 4] {
        quad([
            [x, -0.5, -0.5],
            [x, 0.5, -0.5],
            [x, 0.5, 0.5],
            [x, -0.5, 0.5],
        ])
    }

    /// A quad in the plane `y = 0`, facing -Y, with its own separated x slab.
    fn facing_neg_y_slab(x: f32) -> [TerrainVertex; 4] {
        quad([
            [x, 0.0, -0.5],
            [x + 0.5, 0.0, -0.5],
            [x + 0.5, 0.0, 0.5],
            [x, 0.0, 0.5],
        ])
    }

    fn partitionable_mixed_quads() -> Vec<[TerrainVertex; 4]> {
        (0usize..65)
            .map(|index| {
                let x = index as f32 * 2.0;
                if index.is_multiple_of(2) {
                    facing_pos_x(x)
                } else {
                    facing_neg_y_slab(x)
                }
            })
            .collect()
    }

    fn mesh(quads: &[[TerrainVertex; 4]]) -> Vec<TerrainVertex> {
        quads.iter().flatten().copied().collect()
    }

    #[test]
    fn equal_distances_keep_mesh_order_and_triangle_winding() {
        let mut sorter = Sorter::default();
        assert_eq!(
            sorter.sort(
                &[[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [4.0, 0.0, 0.0]],
                [0.0; 3]
            ),
            &[8, 9, 10, 10, 11, 8, 0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4]
        );
        assert!(sorter.sort(&[], [0.0; 3]).is_empty());
    }

    #[test]
    fn identity_is_the_mesh_order() {
        let mut sorter = Sorter::default();
        assert_eq!(
            sorter.identity(3),
            &[0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4, 8, 9, 10, 10, 11, 8]
        );
        assert!(sorter.identity(0).is_empty());
    }

    #[test]
    fn movement_reverses_faces_without_moving_vertices() {
        let centers = [[0.5, 0.5, 0.0], [0.5, 0.5, 1.0]];
        let mut sorter = Sorter::default();
        assert_eq!(sorter.sort(&centers, [0.5, 0.5, -2.0])[0], 4);
        assert_eq!(sorter.sort(&centers, [0.5, 0.5, 3.0])[0], 0);
    }

    #[test]
    fn cached_viewpoint_handles_negative_and_large_coordinates() {
        let origin = [-30_000_000, -64, 30_000_000];
        let mut state = SortState::new(vec![[0.5; 3]], Order::Dynamic);
        let mut sorter = Sorter::default();
        let camera = [-29_999_999.5, -63.5, 30_000_000.5];
        assert!(state.update(camera, origin, &mut sorter).is_some());
        assert!(state.update(camera, origin, &mut sorter).is_none());
        assert!(
            state
                .update([-29_999_999.1, -63.1, 30_000_000.9], origin, &mut sorter)
                .is_none()
        );
        assert!(
            state
                .update([-29_999_998.9, -63.1, 30_000_000.9], origin, &mut sorter)
                .is_some()
        );
    }

    #[test]
    fn large_tiles_resort_inside_the_same_section_viewpoint() {
        let mut state = SortState::new(vec![[32.0, 0.0, 0.0], [96.0, 0.0, 0.0]], Order::Dynamic);
        let mut sorter = Sorter::default();
        assert_eq!(
            state
                .update_lod([48.0, 20.0, 20.0], [0; 3], &mut sorter)
                .unwrap()[0],
            4
        );
        assert!(
            state
                .update_lod([48.5, 20.0, 20.0], [0; 3], &mut sorter)
                .is_none()
        );
        // Both views have positive x/y/z signs relative to the first section,
        // so the section-sized shortcut would incorrectly keep the old order.
        assert_eq!(
            state
                .update_lod([80.0, 20.0, 20.0], [0; 3], &mut sorter)
                .unwrap()[0],
            0
        );
    }

    #[test]
    fn a_single_quad_needs_no_order() {
        assert_eq!(classify(&mesh(&[facing_pos_z(4.0)])), Order::AnyOrder);
        assert_eq!(classify(&[]), Order::AnyOrder);
    }

    #[test]
    fn coplanar_quads_need_no_order() {
        let a = facing_pos_z(4.0);
        let mut b = facing_pos_z(4.0);
        b[0].pos = [4.0, 0.0, 4.0];
        assert_eq!(classify(&mesh(&[a, b])), Order::AnyOrder);
    }

    #[test]
    fn a_double_sided_plane_needs_no_order() {
        // The same plane, both ways: a zero-thickness pane.
        assert_eq!(
            classify(&mesh(&[facing_pos_z(4.0), facing_neg_z(4.0)])),
            Order::AnyOrder
        );
    }

    #[test]
    fn parallel_quads_facing_one_way_sort_once_by_the_axis() {
        assert_eq!(
            classify(&mesh(&[facing_pos_z(2.0), facing_pos_z(6.0)])),
            Order::Static {
                axis: 2,
                signs: vec![1, 1]
            }
        );
        assert_eq!(
            classify(&mesh(&[facing_neg_z(2.0), facing_neg_z(6.0)])),
            Order::Static {
                axis: 2,
                signs: vec![-1, -1]
            }
        );
    }

    #[test]
    fn panes_facing_both_ways_on_one_axis_share_the_static_order() {
        // A flat wall: a +Z face at 2 and a -Z face at 6. The eye can pass
        // between them, but each side's order along z is still fixed.
        assert_eq!(
            classify(&mesh(&[facing_pos_z(2.0), facing_neg_z(6.0)])),
            Order::Static {
                axis: 2,
                signs: vec![1, -1]
            }
        );
    }

    #[test]
    fn faces_on_two_axes_sort_once_when_none_sees_through() {
        // Two +Z faces, one behind the other, and a +X face beside them: on
        // more than one axis, so `Static` refuses, but no pair can be seen
        // through another, so one fixed order serves every camera. The inner
        // +Z face is the far one and comes first.
        let inner = facing_pos_z(2.0);
        let outer = facing_pos_z(6.0);
        let side = quad([
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 1.0, 1.0],
            [1.0, 0.0, 1.0],
        ]);
        assert_eq!(
            classify(&mesh(&[inner, outer, side])),
            Order::StaticTopo {
                order: vec![0, 1, 2]
            }
        );
    }

    #[test]
    fn a_convex_box_surface_needs_no_order() {
        // +Z, -Z, +X, -X, +Y, -Y faces of the box [0,1]^3, wound outwards.
        let quads = [
            quad([
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ]),
            quad([
                [0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
            ]),
            quad([
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 1.0, 1.0],
                [1.0, 0.0, 1.0],
            ]),
            quad([
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 1.0],
                [0.0, 1.0, 0.0],
            ]),
            quad([
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 1.0],
                [1.0, 1.0, 1.0],
                [1.0, 1.0, 0.0],
            ]),
            quad([
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
            ]),
        ];
        assert_eq!(classify(&mesh(&quads)), Order::AnyOrder);
    }

    #[test]
    fn a_quad_inside_a_box_sorts_once_by_topology() {
        let quads = [
            quad([
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 1.0],
                [1.0, 1.0, 1.0],
                [0.0, 1.0, 1.0],
            ]),
            quad([
                [0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
            ]),
            quad([
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 1.0, 1.0],
                [1.0, 0.0, 1.0],
            ]),
            quad([
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 1.0],
                [0.0, 1.0, 0.0],
            ]),
            quad([
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 1.0],
                [1.0, 1.0, 1.0],
                [1.0, 1.0, 0.0],
            ]),
            quad([
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 0.0, 1.0],
                [0.0, 0.0, 1.0],
            ]),
            // Through the middle, facing +X.
            quad([
                [0.5, 0.2, 0.2],
                [0.5, 0.8, 0.2],
                [0.5, 0.8, 0.8],
                [0.5, 0.2, 0.8],
            ]),
        ];
        // The box's faces are all on its hull and the one inside is behind
        // the near faces from every eye that sees it, so one fixed order
        // holds: the inner face first, then the hull.
        let Order::StaticTopo { order } = classify(&mesh(&quads)) else {
            panic!("expected a topological order");
        };
        assert_eq!(order.len(), quads.len());
        assert_eq!(order[0], 6, "the inner face comes first: {order:?}");
    }

    #[test]
    fn a_sloped_ramp_sorts_once_by_topology() {
        let sloped = quad([
            [0.0, 0.0, 4.0],
            [1.0, 0.0, 4.5],
            [1.0, 1.0, 4.5],
            [0.0, 1.0, 4.0],
        ]);
        // One quad alone is trivially any order.
        assert_eq!(classify(&mesh(&[sloped])), Order::AnyOrder);

        // Two parallel sloped quads: sloped is not axis aligned, so `Static`
        // cannot order them, but one fixed order does hold — the far one
        // first, from whichever side the eye is on.
        let mut other = sloped;
        for vertex in &mut other {
            vertex.pos[2] += 2.0;
        }
        assert_eq!(
            classify(&mesh(&[sloped, other])),
            Order::StaticTopo { order: vec![0, 1] }
        );
    }

    #[test]
    fn crossing_quads_need_sorting() {
        // Two quads that pass through each other: each can be seen through
        // the other, so the relation has a cycle and no fixed order is right.
        let z_plane = quad([
            [0.0, 0.0, 4.0],
            [8.0, 0.0, 4.0],
            [8.0, 8.0, 4.0],
            [0.0, 8.0, 4.0],
        ]);
        let x_plane = quad([
            [4.0, 0.0, 0.0],
            [4.0, 8.0, 0.0],
            [4.0, 8.0, 8.0],
            [4.0, 0.0, 8.0],
        ]);
        assert_eq!(classify(&mesh(&[z_plane, x_plane])), Order::Dynamic);
    }

    #[test]
    fn partition_tree_sorts_separated_mixed_axis_slabs_far_to_near() {
        let quads = partitionable_mixed_quads();
        let vertices = mesh(&quads);
        let Order::PartitionTree(tree) = classify(&vertices) else {
            panic!("expected mixed-axis slabs to build a partition tree");
        };
        assert_eq!(tree.cuts[0].axis, 0);

        let centers: Vec<_> = quads
            .iter()
            .map(|quad| quad_center(quad[0].pos, quad[2].pos))
            .collect();
        let quads_of = |indices: &[u32]| -> Vec<u32> {
            indices.iter().step_by(6).map(|index| index / 4).collect()
        };
        let mut sorter = Sorter::default();
        let left = quads_of(sorter.sort_partitioned(&centers, [-10.0, 0.0, 0.0], &tree));
        let right = quads_of(sorter.sort_partitioned(&centers, [200.0, 0.0, 0.0], &tree));
        assert_eq!(left, (0..65).rev().collect::<Vec<_>>());
        assert_eq!(right, (0..65).collect::<Vec<_>>());
    }

    #[test]
    fn partition_tree_crossing_a_cut_invalidates_a_diagonal_view() {
        let quads = partitionable_mixed_quads();
        let Order::PartitionTree(tree) = classify(&mesh(&quads)) else {
            panic!("expected mixed-axis slabs to build a partition tree");
        };
        let cut = tree.cuts[0];
        let centers: Vec<_> = quads
            .iter()
            .map(|quad| quad_center(quad[0].pos, quad[2].pos))
            .collect();
        let mut state = SortState::new(centers, Order::PartitionTree(tree));
        let mut sorter = Sorter::default();
        let camera = |x: f64| [x, 20.0, 20.0];

        assert!(
            state
                .update(
                    camera(f64::from(cut.coordinate) - 0.01),
                    [0; 3],
                    &mut sorter
                )
                .is_some()
        );
        assert!(
            state
                .update(
                    camera(f64::from(cut.coordinate) - 0.005),
                    [0; 3],
                    &mut sorter
                )
                .is_none()
        );
        assert!(
            state
                .update(
                    camera(f64::from(cut.coordinate) + 0.01),
                    [0; 3],
                    &mut sorter
                )
                .is_some()
        );
    }

    #[test]
    fn partition_tree_distance_sorts_inside_unresolved_leaves() {
        let tree = PartitionTree {
            root: PartitionNode::Cut {
                cut: 0,
                plane: Vec::new(),
                lower: Box::new(PartitionNode::Leaf(vec![0, 1])),
                upper: Box::new(PartitionNode::Leaf(vec![2, 3])),
            },
            cuts: vec![Cut {
                axis: 0,
                coordinate: 0.0,
            }],
        };
        let centers = [
            [-2.0, 0.0, 0.0],
            [-8.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [8.0, 0.0, 0.0],
        ];
        let mut sorter = Sorter::default();
        let quads: Vec<_> = sorter
            .sort_partitioned(&centers, [10.0, 0.0, 0.0], &tree)
            .iter()
            .step_by(6)
            .map(|index| index / 4)
            .collect();
        assert_eq!(quads, [1, 0, 2, 3]);
    }

    #[test]
    fn quads_on_two_axes_sort_once_by_topology() {
        // Two -Y faces at different heights, with a +Z face beside them: more
        // than one axis, and neither -Y face on the box's hull, so `Static`
        // and `AnyOrder` both refuse, but no pair sees through another.
        let lower = quad([
            [0.0, 4.0, 0.0],
            [1.0, 4.0, 0.0],
            [1.0, 4.0, 1.0],
            [0.0, 4.0, 1.0],
        ]);
        let upper = quad([
            [0.0, 8.0, 0.0],
            [1.0, 8.0, 0.0],
            [1.0, 8.0, 1.0],
            [0.0, 8.0, 1.0],
        ]);
        let side = quad([
            [0.0, 0.0, 4.0],
            [1.0, 0.0, 4.0],
            [1.0, 1.0, 4.0],
            [0.0, 1.0, 4.0],
        ]);
        let Order::StaticTopo { order } = classify(&mesh(&[lower, upper, side])) else {
            panic!("expected a topological order");
        };
        assert_eq!(order.len(), 3);
        // Seen from below, the higher face is the farther one, so it comes
        // first of the two.
        let position = |quad: u32| order.iter().position(|&q| q == quad).unwrap();
        assert!(position(1) < position(0), "order {order:?}");
    }

    #[test]
    fn any_order_writes_identity_once() {
        let mut state = SortState::new(vec![[0.0; 3]; 2], Order::AnyOrder);
        let mut sorter = Sorter::default();
        assert_eq!(
            state.update([0.0; 3], [0; 3], &mut sorter).unwrap(),
            &[0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4]
        );
        assert!(state.is_sorted());
        assert!(state.update([64.0; 3], [0; 3], &mut sorter).is_none());
    }

    #[test]
    fn static_order_is_the_far_end_first_from_every_camera() {
        // Quads at z = 6 (index 0) and z = 2 (index 1), both facing +Z.
        let centers = vec![[0.0, 0.0, 6.0], [0.0, 0.0, 2.0]];
        let order = Order::Static {
            axis: 2,
            signs: vec![1, 1],
        };
        let mut state = SortState::new(centers.clone(), order.clone());
        let mut sorter = Sorter::default();
        let written = state
            .update([0.0, 0.0, 10.0], [0; 3], &mut sorter)
            .unwrap()
            .to_vec();
        // z = 2 is the far one, so its quad (index 1) comes first, and the
        // order is written once and kept.
        assert_eq!(written, [4, 5, 6, 6, 7, 4, 0, 1, 2, 2, 3, 0]);
        assert!(
            state
                .update([0.0, 0.0, -10.0], [0; 3], &mut sorter)
                .is_none()
        );

        // A camera on the far side sorts the same, so nothing recreates the
        // vanilla mistake of flipping parallel panes as the eye crosses them.
        let mut other = SortState::new(centers, order);
        let mut other_sorter = Sorter::default();
        assert_eq!(
            other
                .update([0.0, 0.0, -10.0], [0; 3], &mut other_sorter)
                .unwrap()
                .to_vec(),
            written
        );
    }

    #[test]
    fn static_order_agrees_with_vanilla_for_the_quads_the_eye_sees() {
        // Four +Z-facing quads at z = 0, 4, 8, 12. Vanilla orders them by
        // camera distance; a +Z face is seen only with the eye beyond it, so
        // the static order must give the same sequence for the visible ones.
        let zs = [0.0f32, 4.0, 8.0, 12.0];
        let centers: Vec<[f32; 3]> = zs.iter().map(|&z| [0.0, 0.0, z]).collect();
        let quads_of =
            |indices: &[u32]| -> Vec<u32> { indices.iter().step_by(6).map(|v| v / 4).collect() };
        let mut by_distance = Sorter::default();
        for camera_z in [-8.0f32, -1.0, 2.0, 6.0, 10.0, 20.0] {
            let visible = |q: &u32| camera_z > zs[*q as usize];
            let mut state = SortState::new(
                centers.clone(),
                Order::Static {
                    axis: 2,
                    signs: vec![1; 4],
                },
            );
            let mut sorter = Sorter::default();
            let static_order = state
                .update([0.0, 0.0, f64::from(camera_z)], [0; 3], &mut sorter)
                .unwrap();
            let static_seen: Vec<u32> =
                quads_of(static_order).into_iter().filter(visible).collect();
            let vanilla_seen: Vec<u32> = quads_of(by_distance.sort(&centers, [0.0, 0.0, camera_z]))
                .into_iter()
                .filter(visible)
                .collect();
            assert_eq!(static_seen, vanilla_seen, "camera z = {camera_z}");
        }
    }

    #[test]
    fn static_order_agrees_with_vanilla_for_both_sides_of_one_axis() {
        // A flat wall: a +Z face at z = 0, 4 and a -Z face at z = 8, 12. The
        // two sides are never both in front of the eye, so only each side's
        // visible order has to match vanilla's.
        let quads = [
            facing_pos_z(0.0),
            facing_pos_z(4.0),
            facing_neg_z(8.0),
            facing_neg_z(12.0),
        ];
        let signs = [1i8, 1, -1, -1];
        assert_eq!(
            classify(&mesh(&quads)),
            Order::Static {
                axis: 2,
                signs: signs.to_vec()
            }
        );

        let centers: Vec<[f32; 3]> = quads
            .iter()
            .map(|q| quad_center(q[0].pos, q[2].pos))
            .collect();
        let zs = [0.0f32, 4.0, 8.0, 12.0];
        let quads_of =
            |indices: &[u32]| -> Vec<u32> { indices.iter().step_by(6).map(|v| v / 4).collect() };
        let mut by_distance = Sorter::default();
        for camera_z in [-2.0f32, 2.0, 6.0, 10.0, 14.0] {
            let mut state = SortState::new(
                centers.clone(),
                Order::Static {
                    axis: 2,
                    signs: signs.to_vec(),
                },
            );
            let mut sorter = Sorter::default();
            let written = quads_of(
                state
                    .update([0.0, 0.0, f64::from(camera_z)], [0; 3], &mut sorter)
                    .unwrap(),
            );
            let vanilla = quads_of(by_distance.sort(&centers, [0.0, 0.0, camera_z]));
            for side in [1i8, -1] {
                let seen = |q: &u32| {
                    signs[*q as usize] == side
                        && if side > 0 {
                            camera_z > zs[*q as usize]
                        } else {
                            camera_z < zs[*q as usize]
                        }
                };
                let ours: Vec<u32> = written.iter().copied().filter(seen).collect();
                let theirs: Vec<u32> = vanilla.iter().copied().filter(seen).collect();
                assert_eq!(ours, theirs, "camera z = {camera_z}, side {side}");
            }
        }
    }

    #[test]
    fn topology_agrees_with_vanilla_for_the_quads_the_eye_sees() {
        // Two parallel sloped quads, a ramp apart: the far one first, from
        // whichever side the eye is on. Only the quads facing the eye are
        // drawn, so vanilla's order over those must be the static one.
        let far = quad([
            [0.0, 0.0, 4.0],
            [1.0, 0.0, 4.5],
            [1.0, 1.0, 4.5],
            [0.0, 1.0, 4.0],
        ]);
        let near = quad([
            [0.0, 0.0, 6.0],
            [1.0, 0.0, 6.5],
            [1.0, 1.0, 6.5],
            [0.0, 1.0, 6.0],
        ]);
        let vertices = mesh(&[near, far]);
        let Order::StaticTopo { order } = classify(&vertices) else {
            panic!("expected a topological order");
        };
        assert_eq!(order, vec![1, 0]);

        let centers: Vec<[f32; 3]> = [near, far]
            .iter()
            .map(|q| quad_center(q[0].pos, q[2].pos))
            .collect();
        let quads_of =
            |indices: &[u32]| -> Vec<u32> { indices.iter().step_by(6).map(|v| v / 4).collect() };
        let mut by_distance = Sorter::default();
        for camera in [[-10.0f32, 0.0, 20.0], [-1.0, 2.0, 10.0], [0.5, 0.0, 20.0]] {
            let faces = |q: u32| {
                let at = q as usize * 4;
                let plane = plane_of(&vertices[at..at + 4]).unwrap();
                plane.normal.dot(Vec3::from_array(camera)) > plane.distance
            };
            let ours: Vec<u32> = order.iter().copied().filter(|q| faces(*q)).collect();
            let theirs: Vec<u32> = quads_of(by_distance.sort(&centers, camera))
                .into_iter()
                .filter(|q| faces(*q))
                .collect();
            assert_eq!(ours, theirs, "camera {camera:?}");
        }
    }

    #[test]
    fn topology_is_written_once() {
        let mut state = SortState::new(
            vec![[0.0; 3]; 3],
            Order::StaticTopo {
                order: vec![2, 0, 1],
            },
        );
        let mut sorter = Sorter::default();
        assert_eq!(
            state.update([0.0; 3], [0; 3], &mut sorter).unwrap(),
            &[8, 9, 10, 10, 11, 8, 0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4]
        );
        assert!(state.is_sorted());
        assert!(state.update([64.0; 3], [0; 3], &mut sorter).is_none());
    }
}
