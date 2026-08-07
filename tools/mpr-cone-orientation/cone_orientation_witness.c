/* Witness for: is `SPECIALISED[cone][*]=false`
 * (`crates/moveit-collision/src/fcl_tangency_table.rs`) an
 * orientation-independent claim, or does it hold only at the one pose
 * `tools/fcl-tangency-probe/probe.cpp` happened to construct?
 *
 * `SPECIALISED`'s `false` cells mean "no closed-form fcl routine is
 * registered for this pair; it falls through to fcl's generic libccd MPR
 * path" -- see that file's own module doc. The pinned per-pair boolean
 * pinned in `tools/ci/verify-fcl-tangency-dispatch.sh`'s `EXPECTED` table
 * (box x cone = F, box x cylinder = F, etc.) is a single MEASUREMENT, at
 * probe.cpp's one specific configuration: both shapes at IDENTITY
 * rotation, stacked along z (probe.cpp:57-61). This program asks whether
 * that measurement generalizes to every relative ORIENTATION of an
 * otherwise-equally-exact tangency, by driving the real, unmodified
 * libccd (this directory's `build.sh` pins the same checkout/tag
 * `tools/mpr-vs-epa/build.sh` does: `/home/stevek/work/libccd` @ `v2.1`)
 * through the exact same entry points fcl's own dispatcher calls.
 *
 * # Why this ports fcl's own support functions instead of reusing
 * libccd's `testsuites/support.c` (as `tools/mpr-vs-epa/mpr_case104.c`
 * does)
 *
 * `mpr_case104.c` compares this port's own EPA against a reference MPR
 * oracle and is free to use libccd's own generic support functions for
 * that. This program instead asks what FCL SPECIFICALLY reports, so it
 * needs FCL's own support/center functions, not a substitute -- `fcl`'s
 * `ShapeIntersectLibccdImpl` default template
 * (`include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h:110-164`)
 * always calls `GJKCollide` with
 * `GJKInitializer<S,Shape>::getSupportFunction()` /
 * `getCenterFunction()`, and `GJKCollide`
 * (`include/fcl/narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h:2670-2709`)
 * itself just forwards straight to `ccdMPRIntersect` /
 * `ccdMPRPenetration` with those exact function pointers -- no
 * intervening logic. `supportCone`/`supportBox`/`supportCyl`/
 * `centerShape` below are byte-for-byte ports (renamed struct/field names
 * only) of `gjk_libccd-inl.h:2435-2455,2485-2543,2645-2649`, so the
 * geometric decision this program makes is provably fcl's own.
 *
 * # Construction
 *
 * Two families, both built at the SAME shape parameters
 * `probe.cpp:45-48` uses (box: side 1.0, half-extent 0.5; cone/cylinder:
 * radius 0.5, lz 1.0, ccd height 0.5) -- an earlier revision of this
 * program used an arbitrarily larger box (100x100 half-extent) and found
 * every orientation reads INTERSECT, including the axis-aligned case that
 * should agree with the pin; a probe-literal reproduction at probe.cpp's
 * own proportions correctly reads separated at theta=0, so discoverPortal's
 * outcome depends on the actual center-to-center metric geometry, not
 * just directions -- the box must match the touching shape's own scale to
 * mean anything.
 *
 * `runProbeLikeConeCase` / `runProbeLikeCylinderCase`: the literal
 * probe.cpp configuration (identity rotations, delta=0) -- the control
 * that MUST reproduce the pin, or nothing below is trustworthy.
 *
 * `runConeTilt`: box's flat top face at world z=0 (fixed, never rotated);
 * a cone's apex is placed at world (0,0,0) by construction -- solved as
 * `-R*(0,0,height)` using the library's own `ccdQuatRotVec`, so the apex
 * lands at the origin to floating-point exactness regardless of
 * quaternion sign convention, not assumed -- with its axis tilted by
 * `theta` degrees off vertical (about world Y) from the probe's own
 * axis-aligned pose. Elementary solid-cone geometry: for
 * `theta + alpha < 90` degrees (alpha = the cone's own half-angle), the
 * entire cone body other than the apex point stays strictly on the z>0
 * side, so this is an exact single-point tangency for every theta in that
 * range -- confirmed here by direct numeric sampling of the base rim's
 * minimum world z (`groundTruth`), not assumed from the algebra alone.
 *
 * `runCylTilt`: the cylinder analogue. A cylinder has no single fixed
 * apex, so the touching point is found numerically: sample both rim
 * circles (top and bottom face) under the candidate rotation and take the
 * global argmin of world z (a flat convex disk's z-extreme under any
 * rotation always lies on its own boundary circle, so sampling both rims
 * is exhaustive for the whole solid), then place the cylinder so that
 * point sits at the origin. An earlier revision of this function searched
 * only the bottom rim; at theta=0 (a full 180-degree flip under this
 * rotation convention) the true minimum is on the OTHER face, uniformly,
 * and solving position from the wrong face drove the cylinder up to 0.5
 * deep into the box -- caught by this same ground-truth check, not by
 * inspection.
 *
 * Every case's true minimum world z (offset-adjusted) is printed
 * alongside the algorithm's verdict, so a reader can confirm "exact
 * tangency" independently of trusting this program's own construction.
 */
#include <stdio.h>
#include <math.h>
#include <ccd/ccd.h>
#include <ccd/vec3.h>
#include <ccd/quat.h>

typedef struct { ccd_vec3_t pos; ccd_quat_t rot, rot_inv; } obj_t;
typedef struct { obj_t o; ccd_real_t radius, height; } cone_t; /* height = half-height, coneToGJK: cone->height = s.lz/2 */
typedef struct { obj_t o; ccd_real_t dim[3]; } box_t;          /* dim = half-extents, boxToGJK: box->dim[i] = s.side[i]/2 */
typedef struct { obj_t o; ccd_real_t radius, height; } cyl_t;  /* height = half-height, cylToGJK: cyl->height = s.lz/2 */

static const double COLLISION_TOLERANCE = 2.0097183471152322e-14; /* fcl's gjk_default_tolerance() = eps()^(7/8), eps=DBL_EPSILON; math/constants.h:145-148,164-166 */
static const unsigned MAX_ITERATIONS = 500;                       /* fcl's default max_collision_iterations; gjk_solver_libccd-inl.h:973 */

static inline ccd_real_t signOf(ccd_real_t v) { return v < 0 ? -CCD_ONE : CCD_ONE; }

/* verbatim port of gjk_libccd-inl.h:2513-2543 */
static void supportCone(const void *obj, const ccd_vec3_t *dir_, ccd_vec3_t *v)
{
    const cone_t *cone = (const cone_t *)obj;
    ccd_vec3_t dir;
    ccdVec3Copy(&dir, dir_);
    ccdQuatRotVec(&dir, &cone->o.rot_inv);

    double zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
    double len = zdist + dir.v[2] * dir.v[2];
    zdist = sqrt(zdist);
    len = sqrt(len);
    double sin_a = cone->radius / sqrt(cone->radius * cone->radius + 4 * cone->height * cone->height);

    if (dir.v[2] > len * sin_a)
        ccdVec3Set(v, 0., 0., cone->height);
    else if (zdist > 0) {
        double rad = cone->radius / zdist;
        ccdVec3Set(v, rad * ccdVec3X(&dir), rad * ccdVec3Y(&dir), -cone->height);
    } else
        ccdVec3Set(v, 0, 0, -cone->height);

    ccdQuatRotVec(v, &cone->o.rot);
    ccdVec3Add(v, &cone->o.pos);
}

/* verbatim port of gjk_libccd-inl.h:2435-2455 */
static void supportBox(const void *obj, const ccd_vec3_t *dir_, ccd_vec3_t *v)
{
    const box_t *o = (const box_t *)obj;
    ccd_vec3_t dir;
    ccdVec3Copy(&dir, dir_);
    ccdQuatRotVec(&dir, &o->o.rot_inv);
    ccdVec3Set(v, signOf(ccdVec3X(&dir)) * o->dim[0],
               signOf(ccdVec3Y(&dir)) * o->dim[1],
               signOf(ccdVec3Z(&dir)) * o->dim[2]);
    ccdQuatRotVec(v, &o->o.rot);
    ccdVec3Add(v, &o->o.pos);
}

/* verbatim port of gjk_libccd-inl.h:2485-2511 */
static void supportCyl(const void *obj, const ccd_vec3_t *dir_, ccd_vec3_t *v)
{
    const cyl_t *cyl = (const cyl_t *)obj;
    ccd_vec3_t dir;
    ccdVec3Copy(&dir, dir_);
    ccdQuatRotVec(&dir, &cyl->o.rot_inv);
    double zdist = sqrt(dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1]);
    if (ccdIsZero(zdist))
        ccdVec3Set(v, 0., 0., signOf(ccdVec3Z(&dir)) * cyl->height);
    else {
        double rad = cyl->radius / zdist;
        ccdVec3Set(v, rad * ccdVec3X(&dir), rad * ccdVec3Y(&dir), signOf(ccdVec3Z(&dir)) * cyl->height);
    }
    ccdQuatRotVec(v, &cyl->o.rot);
    ccdVec3Add(v, &cyl->o.pos);
}

/* verbatim port of gjk_libccd-inl.h:2645-2649 */
static void centerShape(const void *obj, ccd_vec3_t *c)
{
    ccdVec3Copy(c, &((const obj_t *)obj)->pos);
}

static void setPose(obj_t *o, ccd_real_t px, ccd_real_t py, ccd_real_t pz, const ccd_quat_t *q)
{
    ccdVec3Set(&o->pos, px, py, pz);
    ccdQuatCopy(&o->rot, q);
    ccdQuatInvert2(&o->rot_inv, &o->rot);
}

static void initCcd(ccd_t *ccd)
{
    CCD_INIT(ccd);
    ccd->max_iterations = MAX_ITERATIONS;
    ccd->mpr_tolerance = COLLISION_TOLERANCE;
}

static void report(const char *label, const void *o1, ccd_support_fn s1, const void *o2, ccd_support_fn s2, double true_min_gap)
{
    ccd_t ccd;
    initCcd(&ccd);
    ccd.support1 = s1; ccd.support2 = s2;
    ccd.center1 = centerShape; ccd.center2 = centerShape;
    int intersect = ccdMPRIntersect(o1, o2, &ccd);
    ccd_real_t depth = -1; ccd_vec3_t dir, pos;
    int pen = ccdMPRPenetration(o1, o2, &ccd, &depth, &dir, &pos);
    printf("%-70s true_min_gap=%+.3e  ccdMPRIntersect=%d (%-9s)  ccdMPRPenetration=%d (%-9s) depth=%.4g\n",
           label, true_min_gap,
           intersect, intersect ? "INTERSECT" : "separated",
           pen, pen == 0 ? "INTERSECT" : "separated", pen == 0 ? (double)depth : 0.0);
}

/* probe.cpp's own placement (probe.cpp:57-61): box "upper" at
 * (0,0,+half_z), the other shape "lower" at (0,0,-half_z), both identity
 * rotation, delta=0. The control every finding below is anchored to. */
static void runProbeLikeConeCase(void)
{
    ccd_quat_t idq; ccdQuatSet(&idq, 0, 0, 0, 1);
    box_t box; box.dim[0] = box.dim[1] = box.dim[2] = 0.5;
    setPose(&box.o, 0, 0, 0.5, &idq);
    cone_t cone; cone.radius = 0.5; cone.height = 0.5;
    setPose(&cone.o, 0, 0, -0.5, &idq);
    report("PROBE-LITERAL box(upper) x cone(lower), identity, delta=0 [pin: F]", &box, supportBox, &cone, supportCone, 0.0);
}

static void runProbeLikeCylinderCase(void)
{
    ccd_quat_t idq; ccdQuatSet(&idq, 0, 0, 0, 1);
    box_t box; box.dim[0] = box.dim[1] = box.dim[2] = 0.5;
    setPose(&box.o, 0, 0, 0.5, &idq);
    cyl_t cyl; cyl.radius = 0.5; cyl.height = 0.5;
    setPose(&cyl.o, 0, 0, -0.5, &idq);
    report("PROBE-LITERAL box(upper) x cylinder(lower), identity, delta=0 [pin: F, non-cone control]", &box, supportBox, &cyl, supportCyl, 0.0);
}

/* box: fixed, top face at world z=0. cone: apex placed at world
 * (0,0,apex_z_offset), axis tilted theta degrees off vertical (about
 * world Y) from the probe's own pose. apex_z_offset=0 is the exact-tangency
 * family under test; nonzero values are sanity controls. */
static void runConeTilt(double theta_deg, double apex_z_offset)
{
    double radius = 0.5, lz = 1.0, h = lz / 2.0; /* matches probe.cpp's Cone<S>(0.5,1.0) exactly */
    double alpha_deg = asin(radius / sqrt(radius * radius + lz * lz)) * 180.0 / M_PI;
    double theta = theta_deg * M_PI / 180.0;
    double phi = M_PI - theta; /* rotation about world +Y; theta=0 reproduces probe.cpp's own axis-aligned apex-down pose */

    ccd_quat_t q; ccd_vec3_t yaxis;
    ccdVec3Set(&yaxis, 0, 1, 0);
    ccdQuatSetAngleAxis(&q, phi, &yaxis);

    ccd_vec3_t apex_dir;
    ccdVec3Set(&apex_dir, 0, 0, h);
    ccdQuatRotVec(&apex_dir, &q); /* apex_dir = R*(0,0,h), using the library's own rotation, not hand algebra */
    ccd_vec3_t cone_pos;
    ccdVec3Copy(&cone_pos, &apex_dir);
    ccdVec3Scale(&cone_pos, -1.0);
    cone_pos.v[2] += apex_z_offset; /* apex now lands at exactly (0,0,apex_z_offset) */

    cone_t cone; cone.radius = radius; cone.height = h;
    setPose(&cone.o, ccdVec3X(&cone_pos), ccdVec3Y(&cone_pos), ccdVec3Z(&cone_pos), &q);

    ccd_quat_t idq; ccdQuatSet(&idq, 0, 0, 0, 1);
    box_t box; box.dim[0] = box.dim[1] = box.dim[2] = 0.5;
    setPose(&box.o, 0, 0, -0.5, &idq);

    /* ground truth: sample the base rim (a straight cone's z-extreme away
     * from the apex always lies on its own base circle) to confirm the
     * whole body stays at z>=apex_z_offset -- independent of trusting the
     * theta+alpha<90 algebra */
    double min_rim_z = 1e300;
    int N = 3600;
    for (int i = 0; i < N; i++) {
        double psi = 2.0 * M_PI * i / N;
        ccd_vec3_t p;
        ccdVec3Set(&p, radius * cos(psi), radius * sin(psi), -h);
        ccdQuatRotVec(&p, &q);
        ccdVec3Add(&p, &cone_pos);
        if (ccdVec3Z(&p) < min_rim_z) min_rim_z = ccdVec3Z(&p);
    }

    char label[128];
    snprintf(label, sizeof label, "CONE-TILT theta=%6.2f deg (alpha=%.3f, theta+alpha=%.3f) apex_z=%+.3f",
              theta_deg, alpha_deg, theta_deg + alpha_deg, apex_z_offset);
    report(label, &cone, supportCone, &box, supportBox, min_rim_z - apex_z_offset);
}

/* cylinder analogue of runConeTilt -- see file header for the argmin
 * construction and the bug an earlier revision had. */
static void runCylTilt(double theta_deg, double apex_z_offset)
{
    double radius = 0.5, hc = 0.5; /* matches probe.cpp's Cylinder<S>(0.5,1.0) exactly */
    double theta = theta_deg * M_PI / 180.0;
    double phi = M_PI - theta;
    ccd_quat_t q; ccd_vec3_t yaxis;
    ccdVec3Set(&yaxis, 0, 1, 0);
    ccdQuatSetAngleAxis(&q, phi, &yaxis);

    int N = 3600;
    double best_z = 1e300;
    ccd_vec3_t best_local = {{0, 0, 0}};
    for (int which = 0; which < 2; which++) {
        double z0 = which == 0 ? -hc : hc;
        for (int i = 0; i < N; i++) {
            double psi = 2.0 * M_PI * i / N;
            ccd_vec3_t p;
            ccdVec3Set(&p, radius * cos(psi), radius * sin(psi), z0);
            ccdQuatRotVec(&p, &q);
            if (ccdVec3Z(&p) < best_z) { best_z = ccdVec3Z(&p); best_local = p; }
        }
    }
    ccd_vec3_t cyl_pos;
    ccdVec3Copy(&cyl_pos, &best_local);
    ccdVec3Scale(&cyl_pos, -1.0);
    cyl_pos.v[2] += apex_z_offset;

    cyl_t cyl; cyl.radius = radius; cyl.height = hc;
    setPose(&cyl.o, ccdVec3X(&cyl_pos), ccdVec3Y(&cyl_pos), ccdVec3Z(&cyl_pos), &q);

    ccd_quat_t idq; ccdQuatSet(&idq, 0, 0, 0, 1);
    box_t box; box.dim[0] = box.dim[1] = box.dim[2] = 0.5;
    setPose(&box.o, 0, 0, -0.5, &idq);

    double min_z = 1e300;
    for (int which = 0; which < 2; which++) {
        double z0 = which == 0 ? -hc : hc;
        for (int i = 0; i < N; i++) {
            double psi = 2.0 * M_PI * i / N;
            ccd_vec3_t p;
            ccdVec3Set(&p, radius * cos(psi), radius * sin(psi), z0);
            ccdQuatRotVec(&p, &cyl.o.rot);
            ccdVec3Add(&p, &cyl.o.pos);
            if (ccdVec3Z(&p) < min_z) min_z = ccdVec3Z(&p);
        }
    }

    char label[128];
    snprintf(label, sizeof label, "CYL-TILT  theta=%6.2f deg                                    apex_z=%+.3f", theta_deg, apex_z_offset);
    report(label, &cyl, supportCyl, &box, supportBox, min_z - apex_z_offset);
}

int main(void)
{
    printf("== controls: must reproduce the pin, or nothing below is trustworthy ==\n");
    runProbeLikeConeCase();
    runProbeLikeCylinderCase();

    printf("\n== cone: exact single-point apex tangency across theta in [0,63.4) deg (alpha=26.565) ==\n");
    printf("-- sanity: apex_z=+0.05 must all read separated --\n");
    double sanity_thetas[] = {0.0, 30.0, 62.0};
    for (size_t i = 0; i < sizeof(sanity_thetas)/sizeof(sanity_thetas[0]); i++)
        runConeTilt(sanity_thetas[i], 0.05);
    printf("-- sanity: apex_z=-0.05 must all read INTERSECT with depth~0.05 --\n");
    for (size_t i = 0; i < sizeof(sanity_thetas)/sizeof(sanity_thetas[0]); i++)
        runConeTilt(sanity_thetas[i], -0.05);
    printf("-- the witness family: apex_z=0, exact tangency at every theta --\n");
    double thetas[] = {0.0, 5.0, 10.0, 15.0, 20.0, 26.565, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 62.0, 63.0, 63.4};
    for (size_t i = 0; i < sizeof(thetas)/sizeof(thetas[0]); i++)
        runConeTilt(thetas[i], 0.0);

    printf("\n== cylinder control: same tilt-to-single-point-contact family, non-cone shape ==\n");
    double cyl_thetas[] = {0.0, 5.0, 10.0, 20.0, 30.0, 45.0, 60.0, 80.0};
    printf("-- sanity: apex_z=+-0.05 --\n");
    for (size_t i = 0; i < sizeof(cyl_thetas)/sizeof(cyl_thetas[0]); i++)
        runCylTilt(cyl_thetas[i], 0.05);
    for (size_t i = 0; i < sizeof(cyl_thetas)/sizeof(cyl_thetas[0]); i++)
        runCylTilt(cyl_thetas[i], -0.05);
    printf("-- the witness family: apex_z=0, exact tangency at every theta --\n");
    for (size_t i = 0; i < sizeof(cyl_thetas)/sizeof(cyl_thetas[0]); i++)
        runCylTilt(cyl_thetas[i], 0.0);
    return 0;
}
