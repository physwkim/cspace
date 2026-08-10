// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `shape_msgs/{Mesh,Plane}` <-> [`cspace_core::geometry::Shape`]'s `Mesh`/`Plane`
//! variants. Split out from `collision_object.rs` because
//! `moveit_msgs::msg::CollisionObject.{meshes,planes}` and
//! `moveit_msgs::msg::PositionConstraint`'s `BoundingVolume` (see
//! `crate::constraints::position`, which rejects meshes outright this round)
//! are two different consumers of the same wire shapes -- §11 needs the
//! conversion `constraints::position` chose not to build.

use cspace_core::error::Error;
use cspace_core::geometry::{Mesh, Plane, Shape, Vector3 as CoreVector3};
use r2r::shape_msgs::msg as shape_msgs;

use crate::geometry::Point;

/// Wraps `shape_msgs::msg::Mesh` (see `crate::lib` doc: orphan-rule wrapper).
pub struct MeshMsg(pub shape_msgs::Mesh);

/// Plain local wrapper, for the core->msg direction.
pub struct MeshMsgOut(pub shape_msgs::Mesh);

impl TryFrom<MeshMsg> for Shape {
    type Error = Error;

    /// `MeshTriangle.vertex_indices` is `Vec<u32>`, not `[u32; 3]` -- the
    /// wire format allows any length even though a triangle only ever has
    /// three vertices; a length other than 3 is rejected here rather than
    /// silently truncated or panicking on `Mesh::new`'s array conversion.
    /// [`Mesh::new`] itself then rejects any vertex index out of range for
    /// `vertices` (cspace_core::geometry's own D6-shaped construction check), so
    /// this conversion does not have to re-check that.
    fn try_from(msg: MeshMsg) -> Result<Self, Self::Error> {
        let shape_msgs::Mesh {
            triangles,
            vertices,
        } = msg.0;

        let mut core_vertices = Vec::with_capacity(vertices.len());
        for v in vertices {
            core_vertices.push(CoreVector3::try_from(Point(v))?);
        }

        let mut core_triangles = Vec::with_capacity(triangles.len());
        for (i, tri) in triangles.into_iter().enumerate() {
            let idx = tri.vertex_indices;
            let [a, b, c]: [u32; 3] = idx.clone().try_into().map_err(|_| {
                Error::construct(format!(
                    "MeshTriangle[{i}].vertex_indices has length {}, expected exactly 3",
                    idx.len()
                ))
            })?;
            core_triangles.push([a, b, c]);
        }

        Ok(Shape::Mesh(Mesh::new(core_vertices, core_triangles)?))
    }
}

impl TryFrom<Mesh> for MeshMsgOut {
    type Error = Error;

    fn try_from(mesh: Mesh) -> Result<Self, Self::Error> {
        let mut vertices = Vec::with_capacity(mesh.vertices.len());
        for v in mesh.vertices {
            vertices.push(Point::try_from(v)?.0);
        }
        let triangles = mesh
            .triangles
            .into_iter()
            .map(|[a, b, c]| shape_msgs::MeshTriangle {
                vertex_indices: vec![a, b, c],
            })
            .collect();
        Ok(MeshMsgOut(shape_msgs::Mesh {
            triangles,
            vertices,
        }))
    }
}

/// Wraps `shape_msgs::msg::Plane` (see `crate::lib` doc: orphan-rule wrapper).
pub struct PlaneMsg(pub shape_msgs::Plane);

/// Plain local wrapper, for the core->msg direction.
pub struct PlaneMsgOut(pub shape_msgs::Plane);

impl TryFrom<PlaneMsg> for Plane {
    type Error = Error;

    /// `shape_msgs/Plane.coef` is `Vec<f64>` on the wire (`float64[4]` in the
    /// `.msg` source, but r2r generates a plain `Vec`, not a fixed array --
    /// confirmed against the generated bindings, not assumed) -- a length
    /// other than 4 is rejected rather than indexed out of bounds.
    fn try_from(msg: PlaneMsg) -> Result<Self, Self::Error> {
        let coef = msg.0.coef;
        let [a, b, c, d]: [f64; 4] = coef.clone().try_into().map_err(|_| {
            Error::construct(format!(
                "Plane.coef has length {}, expected exactly 4",
                coef.len()
            ))
        })?;
        Ok(Plane::new(a, b, c, d))
    }
}

impl From<Plane> for PlaneMsgOut {
    fn from(plane: Plane) -> Self {
        PlaneMsgOut(shape_msgs::Plane {
            coef: vec![plane.a, plane.b, plane.c, plane.d],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: f64, y: f64, z: f64) -> r2r::geometry_msgs::msg::Point {
        r2r::geometry_msgs::msg::Point { x, y, z }
    }

    #[test]
    fn mesh_round_trips_through_msg() {
        let msg = shape_msgs::Mesh {
            vertices: vec![
                point(0.0, 0.0, 0.0),
                point(1.0, 0.0, 0.0),
                point(0.0, 1.0, 0.0),
            ],
            triangles: vec![shape_msgs::MeshTriangle {
                vertex_indices: vec![0, 1, 2],
            }],
        };
        let shape = Shape::try_from(MeshMsg(msg)).unwrap();
        let Shape::Mesh(mesh) = shape else {
            panic!("expected Mesh");
        };
        assert_eq!(mesh.vertices.len(), 3);
        assert_eq!(mesh.triangles, vec![[0, 1, 2]]);

        let back = MeshMsgOut::try_from(mesh).unwrap().0;
        assert_eq!(back.vertices.len(), 3);
        assert_eq!(back.triangles[0].vertex_indices, vec![0, 1, 2]);
    }

    #[test]
    fn mesh_triangle_with_wrong_vertex_count_is_rejected() {
        let msg = shape_msgs::Mesh {
            vertices: vec![point(0.0, 0.0, 0.0)],
            triangles: vec![shape_msgs::MeshTriangle {
                vertex_indices: vec![0, 0],
            }],
        };
        let err = Shape::try_from(MeshMsg(msg)).unwrap_err();
        // Not just the variant: `Mesh::new` (cspace_core::geometry) has a sibling
        // `Error::Construct` site (out-of-range vertex index, hit by
        // `mesh_out_of_range_vertex_index_is_rejected` below) that a bare
        // `matches!` cannot tell apart from this length check.
        assert!(
            err.to_string().contains("expected exactly 3"),
            "got: {err:?}"
        );
    }

    #[test]
    fn mesh_out_of_range_vertex_index_is_rejected() {
        let msg = shape_msgs::Mesh {
            vertices: vec![point(0.0, 0.0, 0.0)],
            triangles: vec![shape_msgs::MeshTriangle {
                vertex_indices: vec![0, 1, 2],
            }],
        };
        let err = Shape::try_from(MeshMsg(msg)).unwrap_err();
        // Sibling of `mesh_triangle_with_wrong_vertex_count_is_rejected`
        // above -- this one must name `Mesh::new`'s own check, not this
        // file's length check.
        assert!(
            err.to_string().contains("only 1 vertices exist"),
            "got: {err:?}"
        );
    }

    #[test]
    fn plane_round_trips_through_msg() {
        let msg = shape_msgs::Plane {
            coef: vec![1.0, 2.0, 3.0, 4.0],
        };
        let plane = Plane::try_from(PlaneMsg(msg)).unwrap();
        assert_eq!(plane, Plane::new(1.0, 2.0, 3.0, 4.0));

        let back = PlaneMsgOut::from(plane).0;
        assert_eq!(back.coef, vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn plane_wrong_coef_length_is_rejected() {
        let msg = shape_msgs::Plane {
            coef: vec![1.0, 2.0, 3.0],
        };
        let err = Plane::try_from(PlaneMsg(msg)).unwrap_err();
        assert!(matches!(err, Error::Construct(_)), "got: {err:?}");
    }
}
