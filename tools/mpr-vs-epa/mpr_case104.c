/* Drives the real, unmodified libccd `ccdMPRPenetration` (the same
 * algorithm upstream FCL's self-collision `distanceCallback` resolves
 * through, via `GJKCollide`) on a triangle-vs-cylinder pair, to compare
 * against this port's own `parry3d_f64::query::contact` (EPA) result for
 * the identical pair.
 *
 * This does not reconstruct the triangle or cylinder itself -- both come
 * on stdin, already computed by
 * `crates/cspace-collision/examples/case104_mpr_input.rs`, which does the
 * one reconstruction (forward kinematics + `VisibilityConstraint::cone_mesh`'s
 * formula + parry's own deepest-triangle search) that both sides compare
 * against. Re-deriving that reconstruction here in C, from scratch, a
 * second time, is exactly the failure mode `PORTING-PLAN.md` §188 names
 * for an oracle: a second implementation of a value that should come from
 * one place. This program only ever runs the *comparison* algorithm
 * (MPR), never the reconstruction.
 *
 * stdin format: 11 whitespace-separated doubles, in order --
 *   p0.x p0.y p0.z   (triangle vertex 0, cylinder-local frame, Z-native)
 *   p1.x p1.y p1.z   (triangle vertex 1)
 *   p2.x p2.y p2.z   (triangle vertex 2)
 *   radius           (cylinder radius)
 *   length           (cylinder full length -- NOT half-length; libccd's
 *                      own `ccd_cyl_t.height` is halved internally, see
 *                      `testsuites/support.c`'s `cylSupport`, so this is
 *                      the same convention `cspace_geometry::Cylinder`'s
 *                      own `length` field already uses)
 *
 * stdout: one line, `mpr_depth=<value>` (or `collision=0` if MPR finds no
 * overlap at all -- should not happen for a case constructed to
 * interpenetrate, and is itself a finding if it ever does).
 *
 * # The axis_fix trap (do not re-add it here)
 *
 * `parry3d_f64::shape::Cylinder`'s own canonical axis is Y, so this port's
 * own code (`convert_shape` in `crates/cspace-collision/src/parry.rs`)
 * applies a fixed +90-degree-about-X rotation (`axis_fix`) when building a
 * `parry3d_f64::shape::Cylinder` from this crate's Z-axis
 * `cspace_geometry::Shape::Cylinder`. libccd's own `ccd_cyl_t` support
 * function (`testsuites/support.c`'s `cylSupport`, `ccdVec3Z(&dir)`
 * directly) is already Z-native -- the *same* convention this program's
 * own stdin frame uses, and the same convention URDF/FCL use. Applying
 * `axis_fix`'s rotation to the `ccd_cyl_t` built below would be a category
 * error (a parry-internal representational quirk, not an upstream
 * concept) -- this was round 21's own first-pass mistake on this exact
 * pair, caught by reading `cylSupport` directly before trusting a number
 * from it. The identity quaternion below is correct; do not rotate it.
 */
#include <stdio.h>
#include <stdlib.h>
#include <ccd/ccd.h>
#include "testsuites/support.h"

typedef struct {
    int type;
    ccd_vec3_t pos;
    ccd_quat_t quat;
    ccd_vec3_t p[3];
    ccd_vec3_t c;
} ccd_triangle_t;

#define CCD_OBJ_TRIANGLE 100

static void triSupport(const void *_obj, const ccd_vec3_t *dir_, ccd_vec3_t *v) {
    const ccd_triangle_t *tri = (const ccd_triangle_t *)_obj;
    ccd_vec3_t dir, p;
    ccd_quat_t qinv;
    ccd_real_t maxdot, dot;
    int i;

    ccdVec3Copy(&dir, dir_);
    ccdQuatInvert2(&qinv, &tri->quat);
    ccdQuatRotVec(&dir, &qinv);

    maxdot = -CCD_REAL_MAX;
    ccdVec3Set(v, 0, 0, 0);
    for (i = 0; i < 3; i++) {
        ccdVec3Set(&p, tri->p[i].v[0] - tri->c.v[0], tri->p[i].v[1] - tri->c.v[1],
                   tri->p[i].v[2] - tri->c.v[2]);
        dot = ccdVec3Dot(&dir, &p);
        if (dot > maxdot) {
            maxdot = dot;
            ccdVec3Copy(v, &tri->p[i]);
        }
    }
    ccdQuatRotVec(v, &tri->quat);
    ccdVec3Add(v, &tri->pos);
}

static void triCenter(const void *_obj, ccd_vec3_t *c) {
    const ccd_triangle_t *tri = (const ccd_triangle_t *)_obj;
    ccdVec3Copy(c, &tri->c);
    ccdQuatRotVec(c, &tri->quat);
    ccdVec3Add(c, &tri->pos);
}

int main(void) {
    ccd_triangle_t tri;
    ccd_cyl_t cyl;
    double radius, length;
    int n;

    tri.type = CCD_OBJ_TRIANGLE;
    tri.pos.v[0] = 0;
    tri.pos.v[1] = 0;
    tri.pos.v[2] = 0;
    tri.quat.q[0] = 0;
    tri.quat.q[1] = 0;
    tri.quat.q[2] = 0;
    tri.quat.q[3] = 1;

    n = scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
              &tri.p[0].v[0], &tri.p[0].v[1], &tri.p[0].v[2],
              &tri.p[1].v[0], &tri.p[1].v[1], &tri.p[1].v[2],
              &tri.p[2].v[0], &tri.p[2].v[1], &tri.p[2].v[2], &radius, &length);
    if (n != 11) {
        fprintf(stderr, "expected 11 doubles on stdin (see this file's own header comment for the "
                         "order), got %d\n",
                n);
        return 2;
    }

    tri.c.v[0] = (tri.p[0].v[0] + tri.p[1].v[0] + tri.p[2].v[0]) / 3.0;
    tri.c.v[1] = (tri.p[0].v[1] + tri.p[1].v[1] + tri.p[2].v[1]) / 3.0;
    tri.c.v[2] = (tri.p[0].v[2] + tri.p[1].v[2] + tri.p[2].v[2]) / 3.0;

    cyl.type = CCD_OBJ_CYL;
    cyl.radius = radius;
    cyl.height = length;
    cyl.pos.v[0] = 0;
    cyl.pos.v[1] = 0;
    cyl.pos.v[2] = 0;
    /* Identity: see this file's own header comment, "The axis_fix trap". */
    cyl.quat.q[0] = 0.0;
    cyl.quat.q[1] = 0.0;
    cyl.quat.q[2] = 0.0;
    cyl.quat.q[3] = 1.0;

    ccd_t ccd;
    CCD_INIT(&ccd);
    ccd.support1 = triSupport;
    ccd.support2 = ccdSupport;
    ccd.center1 = triCenter;
    ccd.center2 = ccdObjCenter;
    ccd.max_iterations = 500;
    ccd.mpr_tolerance = 1e-10;

    double depth = 0;
    ccd_vec3_t pdir, ppos;
    int res = ccdMPRPenetration(&tri, &cyl, &ccd, &depth, &pdir, &ppos);
    if (res == 0) {
        printf("mpr_depth=%.17e\n", depth);
    } else {
        printf("collision=0\n");
    }
    return 0;
}
