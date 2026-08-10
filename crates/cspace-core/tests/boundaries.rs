// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! One case per invariant boundary of the SRDF parser.
//!
//! Each expected value was confirmed against `libsrdfdom.so.2.0.8` — the
//! library `tools/moveit-oracle` links — by feeding the same document to a
//! probe built on it, except where a case is marked as a deliberate deviation
//! from upstream. The upstream behaviour is stated at each of those.
//!
//! The boundaries covered, and why each is a boundary rather than a scenario:
//!
//! - attribute absent / present-but-empty — upstream tests a null `const char*`,
//!   so `attr=""` is a present attribute
//! - element a direct child of `<robot>` / nested one level deeper — upstream
//!   walks siblings and never descends
//! - subgroup resolvable / forward-referenced / dangling / cyclic / transitively
//!   dangling — the resolution runs to a fixpoint, not in document order
//! - group referenced by a state or effector: defined / undefined / dropped
//! - joint value: one number / several / none / partly numeric / repeated joint
//! - sphere radius: above `f64::EPSILON` / exactly it / zero / negative, and
//!   whether a positive sphere has been seen yet
//! - number text: leading whitespace / trailing whitespace / trailing garbage
//! - root element: `robot` / anything else / not XML at all

use cspace_core::srdf::{Diagnostic, Sphere, SrdfModel, VirtualJointType};

fn parse(xml: &str) -> SrdfModel {
    SrdfModel::parse_str(xml).expect("document should parse")
}

fn group_names(model: &SrdfModel) -> Vec<&str> {
    model.groups().iter().map(|g| g.name.as_str()).collect()
}

// ------------------------------------------------------- fatal vs. not ----

// Assertion-discrimination sweep (round 2): `parse()` has two
// `Error::Parse` sites (`parse.rs:27` for malformed XML, `parse.rs:37` for
// a non-`robot` root), both carrying the same `source_kind: SRDF` constant
// -- checking `source_kind` alone cannot tell them apart, so this is the
// same shape as an `Error::Code` shared across sibling guards. Fixed to
// check `message` instead, which does differ (roxmltree's own text vs
// this crate's "expected a `robot` root element"); see
// `a_root_element_other_than_robot_is_an_error` below for that sibling.
#[test]
fn malformed_xml_is_an_error() {
    let err = SrdfModel::parse_str("<robot name=\"a\">").unwrap_err();
    assert!(err.to_string().contains("opened but never closed"), "{err}");
}

#[test]
fn a_root_element_other_than_robot_is_an_error() {
    let err = SrdfModel::parse_str("<humanoid name=\"a\"/>").unwrap_err();
    assert!(err.to_string().contains("robot"), "{err}");
}

#[test]
fn an_empty_robot_element_is_a_valid_empty_model() {
    let model = parse("<robot name=\"a\"/>");
    assert_eq!(model.name(), Some("a"));
    assert_eq!(model.groups(), &[]);
    assert_eq!(model.diagnostics(), &[]);
}

// ------------------------------------------- attribute absent vs. empty ----

#[test]
fn absent_robot_name_is_none_and_diagnosed() {
    let model = parse("<robot/>");
    assert_eq!(model.name(), None);
    assert_eq!(model.diagnostics(), &[Diagnostic::MissingRobotName]);
}

/// Upstream reads the attribute as a `const char*` and only checks it against
/// null, so `name=""` is a name that happens to be empty and raises nothing.
#[test]
fn empty_robot_name_is_some_and_not_diagnosed() {
    let model = parse("<robot name=\"\"/>");
    assert_eq!(model.name(), Some(""));
    assert_eq!(model.diagnostics(), &[]);
}

#[test]
fn a_virtual_joint_missing_any_required_attribute_is_dropped() {
    for (attribute, xml) in [
        (
            "name",
            r#"<robot name="r"><virtual_joint type="fixed" parent_frame="w" child_link="l"/></robot>"#,
        ),
        (
            "child_link",
            r#"<robot name="r"><virtual_joint name="v" type="fixed" parent_frame="w"/></robot>"#,
        ),
        (
            "parent_frame",
            r#"<robot name="r"><virtual_joint name="v" type="fixed" child_link="l"/></robot>"#,
        ),
        (
            "type",
            r#"<robot name="r"><virtual_joint name="v" parent_frame="w" child_link="l"/></robot>"#,
        ),
    ] {
        let model = parse(xml);
        assert_eq!(model.virtual_joints(), &[], "missing {attribute}");
        assert!(
            matches!(
                model.diagnostics(),
                [Diagnostic::MissingAttribute { element, attribute: a, .. }]
                    if *element == "virtual_joint" && *a == attribute
            ),
            "missing {attribute}: {:?}",
            model.diagnostics()
        );
    }
}

#[test]
fn a_collision_pair_needs_both_links_and_defaults_its_reason() {
    let model = parse(
        r#"<robot name="r">
             <disable_collisions link1="a" link2="b"/>
             <disable_collisions link1="a"/>
             <disable_collisions link2="b"/>
           </robot>"#,
    );
    assert_eq!(model.disabled_collision_pairs().len(), 1);
    assert_eq!(model.disabled_collision_pairs()[0].reason, "");
    assert_eq!(model.diagnostics().len(), 2);
}

#[test]
fn an_absent_end_effector_parent_group_is_none() {
    let model = parse(
        r#"<robot name="r">
             <group name="g"><joint name="j"/></group>
             <end_effector name="e1" group="g" parent_link="l"/>
             <end_effector name="e2" group="g" parent_link="l" parent_group=""/>
           </robot>"#,
    );
    assert_eq!(model.end_effectors()[0].parent_group, None);
    assert_eq!(
        model.end_effectors()[1].parent_group,
        Some(String::new()),
        "upstream stores both of these as the empty string"
    );
}

// -------------------------------------------- direct child vs. nested ----

/// Upstream reaches every top-level element with `FirstChildElement` /
/// `NextSiblingElement`, which never descends. `panda.srdf` relies on this: it
/// writes a `<passive_joint>` inside a group, and upstream reports no passive
/// joints at all.
#[test]
fn top_level_elements_nested_inside_a_group_are_invisible() {
    let model = parse(
        r#"<robot name="r">
             <group name="g">
               <joint name="j2"/>
               <passive_joint name="j1"/>
               <disable_collisions link1="a" link2="b" reason="Nested"/>
               <virtual_joint name="v" type="fixed" parent_frame="w" child_link="l"/>
             </group>
             <passive_joint name="j3"/>
           </robot>"#,
    );
    assert_eq!(model.passive_joints(), &["j3".to_owned()]);
    assert_eq!(model.disabled_collision_pairs(), &[]);
    assert_eq!(model.virtual_joints(), &[]);
    // ... and the nested elements are not group members either.
    assert_eq!(model.groups()[0].joints, ["j2".to_owned()]);
}

/// A `<group>` inside a `<group>` is a reference, never a definition.
#[test]
fn a_nested_group_element_defines_nothing() {
    let model = parse(
        r#"<robot name="r">
             <group name="outer"><group name="inner"/></group>
             <group name="inner"><joint name="j"/></group>
           </robot>"#,
    );
    assert_eq!(group_names(&model), ["outer", "inner"]);
    assert_eq!(model.groups()[0].subgroups, ["inner".to_owned()]);
}

// -------------------------------------------------- subgroup fixpoint ----

#[test]
fn a_subgroup_may_be_defined_later_in_the_document() {
    let model = parse(
        r#"<robot name="r">
             <group name="wrapper"><group name="inner"/></group>
             <group name="inner"><joint name="j"/></group>
           </robot>"#,
    );
    assert_eq!(group_names(&model), ["wrapper", "inner"]);
    assert_eq!(model.diagnostics(), &[]);
}

#[test]
fn a_group_with_a_dangling_subgroup_is_dropped_transitively() {
    let model = parse(
        r#"<robot name="r">
             <group name="dangling"><group name="nope"/></group>
             <group name="transitive"><group name="dangling"/></group>
             <group name="kept"><joint name="j"/></group>
           </robot>"#,
    );
    assert_eq!(group_names(&model), ["kept"]);
    assert_eq!(
        model.diagnostics(),
        &[
            Diagnostic::UnsatisfiedSubgroups {
                group: "dangling".to_owned()
            },
            Diagnostic::UnsatisfiedSubgroups {
                group: "transitive".to_owned()
            },
        ]
    );
}

/// Resolvability is tracked by *name*, in a `std::set<std::string>` upstream
/// and a `BTreeSet<String>` here (`model.cpp:267,274`, `parse.rs:221,226`),
/// not by which `<group>` element earned it. Two groups sharing one name
/// therefore share one fate: the moment either instance resolves, the name
/// is in the resolved set, so a second element with the same name is skipped
/// by the `already resolvable` guard before its own subgroups are ever
/// looked at — even a dangling one. Upstream's final filter
/// (`groups_.swap(correct)`, `model.cpp:296-306`) and this port's `retain`
/// (`parse.rs:236-246`) both key on the same name, so both instances survive
/// and no `UnsatisfiedSubgroups` is raised for the second.
#[test]
fn a_second_group_with_a_duplicate_name_inherits_the_firsts_resolvability() {
    let model = parse(
        r#"<robot name="r">
             <group name="dup"><group name="nope"/></group>
             <group name="dup"><joint name="j"/></group>
           </robot>"#,
    );
    assert_eq!(group_names(&model), ["dup", "dup"]);
    assert_eq!(model.diagnostics(), &[]);
}

#[test]
fn a_subgroup_cycle_drops_every_group_in_it() {
    let model = parse(
        r#"<robot name="r">
             <group name="selfref"><group name="selfref"/></group>
             <group name="a"><group name="b"/></group>
             <group name="b"><group name="a"/></group>
           </robot>"#,
    );
    assert_eq!(group_names(&model), [] as [&str; 0]);
    assert_eq!(model.diagnostics().len(), 3);
}

// ------------------------------------------------------ group emptiness ----

#[test]
fn an_empty_group_is_kept_but_diagnosed() {
    let model = parse(r#"<robot name="r"><group name="g"/></robot>"#);
    assert_eq!(group_names(&model), ["g"]);
    assert_eq!(
        model.diagnostics(),
        &[Diagnostic::EmptyGroup {
            group: "g".to_owned()
        }]
    );
}

#[test]
fn a_group_holding_only_subgroups_is_not_empty() {
    let model = parse(
        r#"<robot name="r">
             <group name="inner"><joint name="j"/></group>
             <group name="outer"><group name="inner"/></group>
           </robot>"#,
    );
    assert_eq!(model.diagnostics(), &[]);
}

// ----------------------------------------------- group reference checks ----

#[test]
fn a_state_or_effector_may_precede_the_group_it_names() {
    let model = parse(
        r#"<robot name="r">
             <group_state name="s" group="g"><joint name="j" value="0"/></group_state>
             <end_effector name="e" group="g" parent_link="l"/>
             <group name="g"><joint name="j"/></group>
           </robot>"#,
    );
    assert_eq!(model.group_states().len(), 1);
    assert_eq!(model.end_effectors().len(), 1);
    assert_eq!(model.diagnostics(), &[]);
}

#[test]
fn a_state_or_effector_naming_an_undefined_group_is_dropped() {
    let model = parse(
        r#"<robot name="r">
             <group_state name="s" group="nope"><joint name="j" value="0"/></group_state>
             <end_effector name="e" group="nope" parent_link="l"/>
           </robot>"#,
    );
    assert_eq!(model.group_states(), &[]);
    assert_eq!(model.end_effectors(), &[]);
    assert_eq!(
        model.diagnostics(),
        &[
            Diagnostic::UnknownGroup {
                element: "group_state",
                name: "s".to_owned(),
                group: "nope".to_owned(),
            },
            Diagnostic::UnknownGroup {
                element: "end_effector",
                name: "e".to_owned(),
                group: "nope".to_owned(),
            },
        ]
    );
}

/// Groups are fully resolved, including the drop pass, before states and
/// effectors are read, so a state naming a *dropped* group is dropped too.
#[test]
fn a_state_naming_a_group_that_was_dropped_is_dropped() {
    let model = parse(
        r#"<robot name="r">
             <group name="g"><group name="nope"/></group>
             <group_state name="s" group="g"><joint name="j" value="0"/></group_state>
           </robot>"#,
    );
    assert_eq!(model.group_states(), &[]);
    assert!(model.diagnostics().contains(&Diagnostic::UnknownGroup {
        element: "group_state",
        name: "s".to_owned(),
        group: "g".to_owned(),
    }));
}

// ----------------------------------------------- virtual joint typing ----

#[test]
fn virtual_joint_type_is_trimmed_and_case_folded() {
    for (raw, expected) in [
        ("fixed", VirtualJointType::Fixed),
        ("planar", VirtualJointType::Planar),
        ("floating", VirtualJointType::Floating),
        (" FlOaTiNg ", VirtualJointType::Floating),
        ("PLANAR", VirtualJointType::Planar),
    ] {
        let model = parse(&format!(
            r#"<robot name="r"><virtual_joint name="v" type="{raw}" parent_frame="w" child_link="l"/></robot>"#
        ));
        assert_eq!(model.virtual_joints()[0].joint_type, expected, "{raw:?}");
        assert_eq!(model.diagnostics(), &[], "{raw:?}");
    }
}

/// Upstream keeps a joint of unknown type and calls it fixed rather than
/// dropping it; a dropped virtual joint would detach the robot from its world
/// frame, a larger failure than assuming it is bolted down.
#[test]
fn an_unknown_virtual_joint_type_becomes_fixed_and_is_diagnosed() {
    let model = parse(
        r#"<robot name="r"><virtual_joint name="v" type="SPHERICAL" parent_frame="w" child_link="l"/></robot>"#,
    );
    assert_eq!(
        model.virtual_joints()[0].joint_type,
        VirtualJointType::Fixed
    );
    assert_eq!(
        model.diagnostics(),
        &[Diagnostic::UnknownVirtualJointType {
            joint: "v".to_owned(),
            raw: "SPHERICAL".to_owned(),
        }]
    );
}

// ------------------------------------------------ group state values ----

#[test]
fn a_joint_value_holds_as_many_numbers_as_it_names() {
    let model = parse(
        r#"<robot name="r">
             <group name="g"><joint name="j"/></group>
             <group_state name="s" group="g">
               <joint name="one" value="0"/>
               <joint name="three" value="1 2 3"/>
               <joint name="padded" value="  1.5&#10;2.5  "/>
             </group_state>
           </robot>"#,
    );
    let values = &model.group_states()[0].joint_values;
    assert_eq!(values["one"], [0.0]);
    assert_eq!(values["three"], [1.0, 2.0, 3.0]);
    assert_eq!(values["padded"], [1.5, 2.5]);
    assert_eq!(model.diagnostics(), &[]);
}

/// Upstream `push_back`s into the joint's vector, so a repeated joint appends
/// rather than replaces.
#[test]
fn repeating_a_joint_in_one_state_concatenates_its_values() {
    let model = parse(
        r#"<robot name="r">
             <group name="g"><joint name="j"/></group>
             <group_state name="s" group="g">
               <joint name="j" value="1 2 3"/>
               <joint name="j" value="4"/>
             </group_state>
           </robot>"#,
    );
    assert_eq!(
        model.group_states()[0].joint_values["j"],
        [1.0, 2.0, 3.0, 4.0]
    );
}

/// Deliberate deviation. Upstream extracts with an `istringstream` whose
/// failure leaves the `double` at `0` and stores that `0` regardless, so
/// `value="oops"` and `value="0"` build the same state. `0.0` being a legal
/// joint position makes that indistinguishable downstream, so the joint is
/// dropped and reported here instead.
#[test]
fn an_unparsable_joint_value_drops_the_joint_instead_of_storing_zero() {
    for raw in ["", "   ", "oops", "1 oops", "1,2"] {
        let model = parse(&format!(
            r#"<robot name="r">
                 <group name="g"><joint name="j"/></group>
                 <group_state name="s" group="g"><joint name="j" value="{raw}"/></group_state>
               </robot>"#
        ));
        assert!(
            model.group_states()[0].joint_values.is_empty(),
            "{raw:?} should not produce a value"
        );
        assert!(
            matches!(
                model.diagnostics(),
                [Diagnostic::MalformedValue { attribute, .. }] if *attribute == "value"
            ),
            "{raw:?}: {:?}",
            model.diagnostics()
        );
    }
}

#[test]
fn a_joint_in_a_state_needs_both_a_name_and_a_value() {
    let model = parse(
        r#"<robot name="r">
             <group name="g"><joint name="j"/></group>
             <group_state name="s" group="g">
               <joint value="0"/>
               <joint name="j"/>
             </group_state>
           </robot>"#,
    );
    assert!(model.group_states()[0].joint_values.is_empty());
    assert_eq!(model.diagnostics().len(), 2);
}

// ------------------------------------------------------ sphere radius ----

fn spheres(body: &str) -> Vec<Sphere> {
    let model = parse(&format!(
        r#"<robot name="r"><link_sphere_approximation link="l">{body}</link_sphere_approximation></robot>"#
    ));
    model
        .link_sphere_approximations()
        .first()
        .map(|l| l.spheres.clone())
        .unwrap_or_default()
}

/// A link whose spheres are all radius zero means "never collision-check this
/// link", and is normalised to one sphere at the origin — the centre written
/// in the document is discarded.
#[test]
fn radius_zero_spheres_collapse_to_one_sphere_at_the_origin() {
    assert_eq!(
        spheres(r#"<sphere center="1 2 3" radius="0"/><sphere center="4 5 6" radius="0"/>"#),
        [Sphere::default()]
    );
}

/// The threshold is strict: `f64::EPSILON` itself is not a positive radius,
/// matching upstream's `radius > std::numeric_limits<double>::epsilon()`.
#[test]
fn the_positive_radius_threshold_is_strictly_above_epsilon() {
    assert_eq!(
        spheres(&format!(
            r#"<sphere center="9 9 9" radius="{:e}"/>"#,
            f64::EPSILON
        )),
        [Sphere::default()]
    );
    let above = spheres(&format!(
        r#"<sphere center="9 9 9" radius="{:e}"/>"#,
        f64::EPSILON * 2.0
    ));
    assert_eq!(above.len(), 1);
    assert_eq!(above[0].center, [9.0, 9.0, 9.0]);
}

#[test]
fn a_negative_radius_counts_as_zero() {
    assert_eq!(
        spheres(r#"<sphere center="1 2 3" radius="-1"/>"#),
        [Sphere::default()]
    );
}

/// The first positive sphere clears whatever the radius-zero placeholder left
/// behind, and later radius-zero spheres are then discarded outright.
#[test]
fn a_positive_radius_evicts_zero_spheres_on_either_side_of_it() {
    assert_eq!(
        spheres(
            r#"<sphere center="1 2 3" radius="0"/>
               <sphere center="4 5 6" radius="0.5"/>
               <sphere center="7 8 9" radius="0"/>
               <sphere center="1 1 1" radius="0.25"/>"#
        ),
        [
            Sphere {
                center: [4.0, 5.0, 6.0],
                radius: 0.5
            },
            Sphere {
                center: [1.0, 1.0, 1.0],
                radius: 0.25
            },
        ]
    );
}

/// A link with nothing left is absent from the model, not present and empty —
/// that absence is what tells `collision_distance_field` to generate a bounding
/// sphere for the link instead.
#[test]
fn a_link_with_no_surviving_sphere_is_left_out_entirely() {
    let model = parse(
        r#"<robot name="r">
             <link_sphere_approximation link="none"/>
             <link_sphere_approximation link="bad">
               <sphere center="1 2" radius="0.5"/>
             </link_sphere_approximation>
           </robot>"#,
    );
    assert_eq!(model.link_sphere_approximations(), &[]);
}

// --------------------------------------------------- number text rules ----

#[test]
fn a_sphere_centre_takes_three_numbers_and_ignores_a_fourth() {
    let ok = spheres(r#"<sphere center=" 1 2 3 4 " radius="0.5"/>"#);
    assert_eq!(ok[0].center, [1.0, 2.0, 3.0]);
    assert_eq!(spheres(r#"<sphere center="1 2" radius="0.5"/>"#), []);
    assert_eq!(spheres(r#"<sphere center="1 2 x" radius="0.5"/>"#), []);
}

/// Upstream's `toDouble` rejects anything its stream extraction did not
/// consume, so a radius may have leading whitespace but not trailing.
#[test]
fn a_radius_allows_leading_whitespace_but_not_trailing() {
    assert_eq!(
        spheres(r#"<sphere center="0 0 0" radius=" 0.5"/>"#).len(),
        1
    );
    assert_eq!(spheres(r#"<sphere center="0 0 0" radius="0.5 "/>"#), []);
    assert_eq!(spheres(r#"<sphere center="0 0 0" radius="0.5x"/>"#), []);
}

// --------------------------------------------- trimmed vs. verbatim ----

#[test]
fn names_are_trimmed() {
    let model = parse(
        r#"<robot name="  r  ">
             <virtual_joint name=" v " type=" fixed " parent_frame=" w " child_link=" l "/>
             <group name=" g ">
               <link name=" a "/><joint name=" b "/>
               <chain base_link=" c " tip_link=" d "/>
             </group>
             <group name="wrap"><group name=" g "/></group>
             <group_state name=" s " group=" g "><joint name=" b " value="0"/></group_state>
             <end_effector name=" e " group=" g " parent_link=" p " parent_group=" q "/>
             <disable_default_collisions link=" x "/>
             <disable_collisions link1=" y " link2=" z "/>
             <passive_joint name=" pj "/>
             <joint_property joint_name=" jp " property_name=" pn " value="v"/>
           </robot>"#,
    );
    assert_eq!(model.name(), Some("r"));
    assert_eq!(model.virtual_joints()[0].name, "v");
    assert_eq!(model.virtual_joints()[0].parent_frame, "w");
    assert_eq!(model.virtual_joints()[0].child_link, "l");
    let g = &model.groups()[0];
    assert_eq!(g.name, "g");
    assert_eq!(g.links, ["a".to_owned()]);
    assert_eq!(g.joints, ["b".to_owned()]);
    assert_eq!(g.chains[0].base_link, "c");
    assert_eq!(g.chains[0].tip_link, "d");
    assert_eq!(model.groups()[1].subgroups, ["g".to_owned()]);
    assert_eq!(model.group_states()[0].name, "s");
    assert!(model.group_states()[0].joint_values.contains_key("b"));
    let e = &model.end_effectors()[0];
    assert_eq!((e.name.as_str(), e.parent_link.as_str()), ("e", "p"));
    assert_eq!(e.parent_group.as_deref(), Some("q"));
    assert_eq!(model.no_default_collision_links(), &["x".to_owned()]);
    assert_eq!(model.disabled_collision_pairs()[0].link1, "y");
    assert_eq!(model.passive_joints(), &["pj".to_owned()]);
    assert_eq!(model.joint_properties_for("jp")[0].property_name, "pn");
}

/// The two fields upstream stores without trimming are the two that are free
/// text rather than a name to be matched.
#[test]
fn a_collision_reason_and_a_property_value_are_verbatim() {
    let model = parse(
        r#"<robot name="r">
             <disable_collisions link1="a" link2="b" reason="  spaced  "/>
             <joint_property joint_name="j" property_name="p" value="  raw value  "/>
           </robot>"#,
    );
    assert_eq!(model.disabled_collision_pairs()[0].reason, "  spaced  ");
    assert_eq!(model.joint_properties_for("j")[0].value, "  raw value  ");
}

// ------------------------------------------------------- routing rules ----

#[test]
fn enable_and_disable_collisions_land_in_separate_lists() {
    let model = parse(
        r#"<robot name="r">
             <enable_collisions link1="a" link2="b" reason="E"/>
             <disable_collisions link1="c" link2="d" reason="D"/>
           </robot>"#,
    );
    assert_eq!(model.enabled_collision_pairs()[0].reason, "E");
    assert_eq!(model.disabled_collision_pairs()[0].reason, "D");
    assert_eq!(model.enabled_collision_pairs().len(), 1);
    assert_eq!(model.disabled_collision_pairs().len(), 1);
}

#[test]
fn joint_properties_accumulate_per_joint() {
    let model = parse(
        r#"<robot name="r">
             <joint_property joint_name="j" property_name="p1" value="1"/>
             <joint_property joint_name="j" property_name="p2" value="2"/>
             <joint_property joint_name="k" property_name="p3" value="3"/>
           </robot>"#,
    );
    assert_eq!(model.joint_properties().len(), 2);
    assert_eq!(model.joint_properties_for("j").len(), 2);
    assert_eq!(model.joint_properties_for("j")[1].property_name, "p2");
    assert_eq!(model.joint_properties_for("absent"), &[]);
}
