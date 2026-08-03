// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/link_model.hpp
//   moveit_core/robot_model/src/link_model.cpp

use moveit_geometry::{Isometry3, Shape, Vector3};
use nalgebra::Matrix3;

use crate::aabb::Aabb;

/// One piece of a link's collision geometry: a shape, and the constant
/// offset applied to it before the link's own transform.
///
/// Upstream keeps `shapes_` and `collision_origin_transform_` as two
/// parallel `std::vector`s that every mutator must keep the same length —
/// `LinkModel::set_geometry` takes one `Vec` of this pair instead, so
/// "one shape, no matching transform" is unrepresentable rather than a bug
/// class to avoid.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkShape {
    /// The collision shape itself.
    pub shape: Shape,
    /// The constant transform applied to this shape, relative to the link's
    /// own origin. Upstream `collision_origin_transform_[i]`.
    pub origin_transform: Isometry3,
}

impl LinkShape {
    /// Whether [`LinkShape::origin_transform`] is (numerically) the identity.
    ///
    /// Upstream caches this as `collision_origin_transform_is_identity_`,
    /// recomputed by `setGeometry` whenever the shape list changes. Nothing
    /// here needs that value on a hot path, so this port recomputes it from
    /// the transform on every call instead of storing a second value that
    /// could drift from the first.
    pub fn origin_transform_is_identity(&self) -> bool {
        is_identity(&self.origin_transform)
    }
}

/// Upstream `LinkModel::setJointOriginTransform`/`setGeometry`'s identity
/// check: `linear().isIdentity() && translation().norm() < epsilon`. Eigen's
/// `isIdentity()` compares against its type's `dummy_precision()` (`1e-12`
/// for `double`); `translation().norm()` is compared against
/// `std::numeric_limits<double>::epsilon()` (`~2.22e-16`) directly.
fn is_identity(transform: &Isometry3) -> bool {
    const ROTATION_PRECISION: f64 = 1e-12;
    let identity_error =
        transform.rotation.to_rotation_matrix().matrix() - nalgebra::Matrix3::identity();
    identity_error.iter().all(|e| e.abs() < ROTATION_PRECISION)
        && transform.translation.vector.norm() < f64::EPSILON
}

/// A link from the robot: its place in the kinematic tree, the constant
/// offset applied before any joint transform, and its collision/visual
/// geometry.
///
/// Upstream `moveit::core::LinkModel`. Built and owned by
/// [`crate::robot_model::RobotModel`]; there is no public constructor.
///
/// # Deviations from upstream
///
/// 1. **Cross-references are indices, not pointers.** Upstream stores
///    `parent_joint_model_`/`parent_link_model_`/`child_joint_models_` as raw
///    `const JointModel*`/`const LinkModel*`. This port has no raw pointers
///    into a sibling `Vec` — every reference here is an index into
///    [`RobotModel::link_models`](crate::robot_model::RobotModel::link_models)
///    or
///    [`RobotModel::joint_models`](crate::robot_model::RobotModel::joint_models),
///    resolved through the owning `RobotModel`'s accessors.
/// 2. **`shapes_`/`collision_origin_transform_` are one `Vec`, not two.**
///    See [`LinkShape`]. `collision_origin_transform_is_identity_` is
///    likewise not stored — [`LinkShape::origin_transform_is_identity`]
///    recomputes it.
/// 3. **No `shape_extents_`, `associated_fixed_transforms_` or
///    `first_collision_body_transform_index_`.** These serve the collision
///    backend (`moveit-collision`, a later phase) rather than anything this
///    phase's done-criteria read; only `centered_bounding_box_offset_` is
///    carried, because `JointModelGroup`/`RobotModel` construction is not
///    where any of the other three are consumed.
/// 4. **`<mesh>` collision geometry loads STL only, and only through an
///    explicit search-path list.** Upstream calls
///    `shapes::createMeshFromResource`, which resolves a `package://` URI
///    against the live ROS ament index and parses any Assimp-supported mesh
///    format (STL, DAE, OBJ, ...) off disk. This port has no ROS
///    environment to query and no DAE/OBJ parser — only
///    [`moveit_geometry::stl`]'s STL loader — so
///    [`crate::robot_model::RobotModel::from_urdf_and_srdf`] takes an
///    explicit [`crate::MeshSearchPaths`] instead of an ament index, and a
///    `<mesh>` element resolving to anything other than a well-formed STL
///    file is skipped with a
///    [`crate::diagnostic::Diagnostic::UnsupportedLinkGeometry`] naming why
///    (unresolved `package://` URI, unsupported extension, or a malformed
///    STL) — the same way upstream itself skips a shape `constructShape`
///    fails to build (`if (s) shapes.push_back(s);`), just for a broader,
///    named set of reasons. A caller not exercising collision geometry
///    passes [`crate::MeshSearchPaths::none`], under which every `<mesh>`
///    element is skipped this way, matching this port's behaviour before
///    mesh loading existed at all. `<capsule>` (a urdf-rs extension
///    upstream's own URDF parser doesn't itself recognise) is always
///    skipped the same way, with no search path able to change that.
/// 5. **Visual-mesh geometry is never loaded — permanently, by design, not
///    an interim gap.** Only a `<visual><mesh>`'s filename, origin and scale
///    are kept (see [`LinkModel::visual_mesh_filename`]/
///    [`LinkModel::visual_mesh_origin`]/[`LinkModel::visual_mesh_scale`]).
///    Upstream loads visual geometry because `moveit::core::LinkModel` is
///    shared with RViz's renderer; this port's D1 scope is a ROS-independent
///    motion-planning library with no renderer of its own, so a link's
///    rendered appearance has no consumer here to begin with — unlike
///    deviation 4's collision-mesh gap, there is no downstream phase whose
///    done-criteria this blocks, since none of collision checking,
///    kinematics or dynamics reads a visual shape. Loading it would cost a
///    DAE/OBJ parser (Assimp's formats, well beyond [`moveit_geometry::stl`])
///    for data nothing in this crate's dependency graph ever reads. Revisit
///    only if a future phase adds a renderer or visualization export that
///    needs real visual geometry rather than just its filename.
/// 6. **Carries mass and rotational inertia — upstream's own `LinkModel`
///    does not.** `moveit::core::LinkModel` has no such field at all;
///    `dynamics_solver::DynamicsSolver` gets this data by bypassing
///    `RobotModel` entirely and re-parsing the raw URDF a second time via
///    `kdl_parser`. This port's dynamics solver (`moveit-state`) instead
///    reads it from here, so it needs only a `RobotModel` handle — the same
///    reasoning `dynamics_solver.rs`'s doc comment applies to threading
///    URDF `<limit effort="...">` onto [`crate::joint::JointModel`]. [`mass`](LinkModel::mass)
///    is `0.0` and [`inertia`](LinkModel::inertia)'s tensor is all-zero for a
///    link with no `<inertial>` element (`urdf_rs::Inertial::default()`),
///    matching a `<inertial>`-less URDF link being physically massless.
#[derive(Debug, Clone, PartialEq)]
pub struct LinkModel {
    name: String,
    link_index: usize,
    parent_joint_index: usize,
    parent_link_index: Option<usize>,
    child_joint_indices: Vec<usize>,
    joint_origin_transform: Isometry3,
    shapes: Vec<LinkShape>,
    centered_bounding_box_offset: Vector3,
    visual_mesh_filename: Option<String>,
    visual_mesh_origin: Isometry3,
    visual_mesh_scale: Vector3,
    mass: f64,
    center_of_mass: Vector3,
    inertia: Matrix3<f64>,
}

impl LinkModel {
    pub(crate) fn new(
        name: impl Into<String>,
        link_index: usize,
        parent_joint_index: usize,
        parent_link_index: Option<usize>,
        joint_origin_transform: Isometry3,
    ) -> Self {
        Self {
            name: name.into(),
            link_index,
            parent_joint_index,
            parent_link_index,
            child_joint_indices: Vec::new(),
            joint_origin_transform,
            shapes: Vec::new(),
            centered_bounding_box_offset: Vector3::zeros(),
            visual_mesh_filename: None,
            visual_mesh_origin: Isometry3::identity(),
            visual_mesh_scale: Vector3::new(1.0, 1.0, 1.0),
            mass: 0.0,
            center_of_mass: Vector3::zeros(),
            inertia: Matrix3::zeros(),
        }
    }

    pub(crate) fn add_child_joint_index(&mut self, joint_index: usize) {
        self.child_joint_indices.push(joint_index);
    }

    /// `setGeometry`: replace this link's collision shapes and recompute
    /// [`LinkModel::centered_bounding_box_offset`] from them.
    ///
    /// A [`Shape::Mesh`] is measured by its actual vertices (transformed by
    /// its `origin_transform`) rather than [`Shape::extents`], matching
    /// upstream's own special case in `LinkModel::setGeometry` — a mesh's
    /// extents (from `computeShapeExtents`) do not account for the mesh
    /// being off-center within its own local frame, and this AABB needs the
    /// true bound, not the origin-centered one.
    pub(crate) fn set_geometry(&mut self, shapes: Vec<LinkShape>) {
        let mut aabb = Aabb::default();
        for shape in &shapes {
            match &shape.shape {
                Shape::Mesh(mesh) => {
                    for vertex in &mesh.vertices {
                        aabb.extend(shape.origin_transform * vertex);
                    }
                }
                other => aabb.extend_with_transformed_box(&shape.origin_transform, other.extents()),
            }
        }
        self.centered_bounding_box_offset = aabb.center();
        self.shapes = shapes;
    }

    /// Not an upstream method — see [`LinkModel`]'s doc comment, deviation
    /// 5. Sets this link's mass, the center of mass (relative to this
    /// link's own origin, in this link's own frame), and the rotational
    /// inertia tensor about the center of mass, also in this link's own
    /// frame axes (i.e. already rotated out of whatever frame a URDF
    /// `<inertial><origin rpy="...">` expressed it in — the caller's job,
    /// since only it has the raw URDF `Inertial` element to rotate from).
    pub(crate) fn set_inertial(
        &mut self,
        mass: f64,
        center_of_mass: Vector3,
        inertia: Matrix3<f64>,
    ) {
        self.mass = mass;
        self.center_of_mass = center_of_mass;
        self.inertia = inertia;
    }

    /// `setVisualMesh`.
    pub(crate) fn set_visual_mesh(
        &mut self,
        filename: impl Into<String>,
        origin: Isometry3,
        scale: Vector3,
    ) {
        self.visual_mesh_filename = Some(filename.into());
        self.visual_mesh_origin = origin;
        self.visual_mesh_scale = scale;
    }

    /// `getName`
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `getLinkIndex`: this link's position in
    /// [`RobotModel::link_models`](crate::robot_model::RobotModel::link_models),
    /// which is also the order links are visited when traversing the
    /// kinematic tree depth-first.
    pub fn link_index(&self) -> usize {
        self.link_index
    }

    /// `getParentJointModel`, as an index. There is always a parent joint —
    /// even the root link's parent is a joint (the SRDF virtual joint, or an
    /// assumed fixed joint if the SRDF names none).
    pub fn parent_joint_index(&self) -> usize {
        self.parent_joint_index
    }

    /// `getParentLinkModel`, as an index. [`None`] for the root link.
    pub fn parent_link_index(&self) -> Option<usize> {
        self.parent_link_index
    }

    /// `getChildJointModels`, as indices.
    pub fn child_joint_indices(&self) -> &[usize] {
        &self.child_joint_indices
    }

    /// `getJointOriginTransform`: the constant offset pre-applied before the
    /// parent joint's own transform.
    pub fn joint_origin_transform(&self) -> &Isometry3 {
        &self.joint_origin_transform
    }

    /// `getShapes`: this link's collision geometry, each with its own origin
    /// transform. See [`LinkModel`]'s doc comment, deviation 4, for which
    /// `<collision>` elements never appear here.
    pub fn shapes(&self) -> &[LinkShape] {
        &self.shapes
    }

    /// `getCenteredBoundingBoxOffset`: the center of the axis-aligned
    /// bounding box of [`LinkModel::shapes`], with the link positioned at
    /// its own origin. Exactly zero in every component if `shapes` is empty
    /// — see `Aabb`'s doc comment.
    pub fn centered_bounding_box_offset(&self) -> Vector3 {
        self.centered_bounding_box_offset
    }

    /// `getVisualMeshFilename`. [`None`] if this link has no `<mesh>`
    /// visual geometry (upstream: `visual_mesh_filename_.empty()`).
    pub fn visual_mesh_filename(&self) -> Option<&str> {
        self.visual_mesh_filename.as_deref()
    }

    /// `getVisualMeshOrigin`. Meaningless if [`LinkModel::visual_mesh_filename`]
    /// is [`None`].
    pub fn visual_mesh_origin(&self) -> &Isometry3 {
        &self.visual_mesh_origin
    }

    /// `getVisualMeshScale`. Meaningless if [`LinkModel::visual_mesh_filename`]
    /// is [`None`].
    pub fn visual_mesh_scale(&self) -> Vector3 {
        self.visual_mesh_scale
    }

    /// This link's mass, from its URDF `<inertial><mass>`. `0.0` if the
    /// link has no `<inertial>` element. See [`LinkModel`]'s doc comment,
    /// deviation 6, for why upstream's own `LinkModel` has no equivalent.
    pub fn mass(&self) -> f64 {
        self.mass
    }

    /// The center of mass, relative to this link's own origin and expressed
    /// in this link's own frame axes (URDF `<inertial><origin xyz="...">`,
    /// unrotated — a translation only needs no frame conversion). Zero if
    /// the link has no `<inertial>` element.
    pub fn center_of_mass(&self) -> Vector3 {
        self.center_of_mass
    }

    /// The rotational inertia tensor about [`LinkModel::center_of_mass`],
    /// expressed in this link's own frame axes — already rotated out of the
    /// `<inertial><origin rpy="...">` frame URDF itself expresses
    /// `<inertia ixx="..." .../>` in. All-zero if the link has no
    /// `<inertial>` element.
    pub fn inertia(&self) -> &Matrix3<f64> {
        &self.inertia
    }
}
