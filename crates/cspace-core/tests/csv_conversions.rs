// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Invariant-boundary tests for `cspace_core::state::conversions` — the CSV half
//! of upstream `robot_state/conversions` (`robotStateToStream` x2,
//! `streamToRobotState`).
//!
//! One case per boundary the three functions actually have, not one per
//! usage story: header present vs absent, separator comma vs not, trailing
//! separator present (grouped) vs absent (un-grouped), cell count short vs
//! exact vs long, cell numeric vs not, group known vs not, group order
//! given vs model order, and transforms dirty vs stale after a read. The
//! last three of those are where this port and upstream differ, so each is
//! pinned rather than assumed.

use std::fs;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::{
    RobotState, csv_to_robot_state, robot_state_to_csv, robot_state_to_csv_by_groups,
};

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/state/{}"),
        file_name
    )
}

fn panda() -> RobotModel {
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// The first variable's position, spelled out so
/// `a_round_trip_returns_every_position_bit_for_bit` can assert on the text
/// the writer emitted. Seventeen significant digits, and deliberately not
/// near any `f64::consts` value clippy would rather see named.
const FIRST_POSITION: f64 = 0.5123456789012345;

/// Distinct, non-default, full-precision positions: a value that survives a
/// six-significant-digit round trip would make
/// `a_round_trip_returns_every_position_bit_for_bit` pass vacuously.
fn distinctive_positions(model: &RobotModel) -> Vec<f64> {
    (0..model.variable_count())
        .map(|i| FIRST_POSITION - (i as f64) * 0.1111111111111111)
        .collect()
}

// ---- include_header: the boundary between two lines and one ---------------

/// `include_header` adds the name line and changes nothing else, so the two
/// values cannot disagree about the value line they produce.
#[test]
fn the_header_line_is_the_only_difference_include_header_makes() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_variable_positions(&distinctive_positions(&model));

    let with = robot_state_to_csv(&state, true, ',');
    let without = robot_state_to_csv(&state, false, ',');

    let (header, values) = with
        .split_once('\n')
        .expect("a header line must be present");
    assert_eq!(header, model.variable_names().join(","));
    assert_eq!(values, without);
}

// ---- separator: comma vs anything else ------------------------------------

/// The separator argument reaches the writer's every join, header line
/// included: a comma hard-coded anywhere in it survives here. Nothing about
/// field *counts* is asserted — that is the two trailing-separator tests'
/// job, and duplicating it here would make one mutation fail three tests.
#[test]
fn the_separator_argument_replaces_the_comma() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_variable_positions(&distinctive_positions(&model));

    let text = robot_state_to_csv(&state, true, ';');

    assert!(!text.contains(','), "no comma may survive: {text}");
    assert!(
        text.contains(';'),
        "the given separator must appear: {text}"
    );
}

/// A `;`-written line reads back through a `;` reader. Upstream's writer
/// takes the whole separator string and its reader only `separator[0]`
/// (`conversions.cpp:509` against `:572`), so the two can be given
/// arguments that disagree; a `char` cannot.
///
/// Compared against the `,` round trip rather than against the source
/// positions, so that a writer which lost precision would fail
/// `a_round_trip_returns_every_position_bit_for_bit` alone and not this
/// test as well.
#[test]
fn the_separator_a_line_was_written_with_reads_it_back() {
    let model = panda();
    let mut written = RobotState::new(&model);
    written.set_variable_positions(&distinctive_positions(&model));

    let mut via_semicolon = RobotState::new(&model);
    csv_to_robot_state(
        &mut via_semicolon,
        &robot_state_to_csv(&written, false, ';'),
        ';',
    )
    .expect("the writer's own output must parse");
    let mut via_comma = RobotState::new(&model);
    csv_to_robot_state(
        &mut via_comma,
        &robot_state_to_csv(&written, false, ','),
        ',',
    )
    .expect("the writer's own output must parse");

    assert_eq!(via_semicolon.positions(), via_comma.positions());
}

// ---- trailing separator: absent un-grouped, present grouped ---------------

/// The un-grouped writer must not end a line with the separator: upstream
/// guards it (`conversions.cpp:508`, `:520`), and a trailing one would make
/// the field count disagree with the header's.
#[test]
fn the_ungrouped_line_has_no_trailing_separator() {
    let model = panda();
    let state = RobotState::new(&model);

    let text = robot_state_to_csv(&state, true, ',');

    for line in text.lines() {
        assert!(
            !line.ends_with(','),
            "no line may end in the separator: {line:?}"
        );
    }
}

/// The grouped writer must end every line with the separator, because
/// upstream does (`conversions.cpp:542`, `:553`, unguarded). Pinned because
/// the asymmetry between the two overloads is deliberate transcription: a
/// later tidy-up that made them agree would be a silent parity change.
#[test]
fn the_grouped_line_keeps_upstreams_trailing_separator() {
    let model = panda();
    let state = RobotState::new(&model);

    let text = robot_state_to_csv_by_groups(&state, &["panda_arm"], true, ',')
        .expect("the fixture has a panda_arm group");

    let mut lines = 0;
    for line in text.lines() {
        assert!(line.ends_with(','), "upstream ends every line: {line:?}");
        lines += 1;
    }
    assert_eq!(lines, 2, "a header line and a value line");
}

// ---- round trip: exact, not approximate -----------------------------------

/// The round trip is bit-for-bit. Upstream's cannot be: its `std::ostream`
/// writes six significant digits, so `0.5123456789012345` comes back as
/// `0.512346` (`doc/upstream-bugs.md`,
/// `robot-state-to-stream-default-ostream-precision`).
#[test]
fn a_round_trip_returns_every_position_bit_for_bit() {
    let model = panda();
    let mut written = RobotState::new(&model);
    written.set_variable_positions(&distinctive_positions(&model));

    let line = robot_state_to_csv(&written, false, ',');
    let mut read = RobotState::new(&model);
    csv_to_robot_state(&mut read, &line, ',').expect("the writer's own output must parse");

    assert_eq!(read.positions(), written.positions());
    assert!(
        line.contains(&FIRST_POSITION.to_string()),
        "the writer must not round to six significant digits: {line}"
    );
    assert_eq!(
        FIRST_POSITION.to_string(),
        "0.5123456789012345",
        "and `to_string` must be the shortest exact form, not a rounded one"
    );
}

// ---- cell count: short, exact, long ---------------------------------------

/// A line one cell short is an error naming the variable that ran out, not
/// a parse of whatever the previous cell held. Upstream logs and then calls
/// `std::stod` on that stale cell anyway (`conversions.cpp:573-574`).
///
/// The short line is built here rather than by trimming the writer's own
/// output, so that the writer's dialect — trailing separator or not — cannot
/// decide whether this reader test sees a short line at all.
#[test]
fn a_line_shorter_than_the_variable_count_is_an_error() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let cells = vec!["0.0"; model.variable_count() - 1];
    let short = cells.join(",");

    let error = csv_to_robot_state(&mut state, &short, ',').unwrap_err();

    let last = &model.variable_names()[model.variable_count() - 1];
    assert_eq!(
        error.to_string(),
        format!(
            "CSV parse error: line holds {} cells, but this robot model has {} variables; \
             missing variable {last:?}",
            model.variable_count() - 1,
            model.variable_count()
        )
    );
}

/// Cells past the variable count are ignored — upstream's own loop bound
/// (`conversions.cpp:569`), and what lets a grouped line, trailing
/// separator and all, read back through this reader.
///
/// The reference is the same line without its extra cells, not the source
/// positions, so a writer that lost precision would fail
/// `a_round_trip_returns_every_position_bit_for_bit` alone.
#[test]
fn cells_past_the_variable_count_are_ignored() {
    let model = panda();
    let mut source = RobotState::new(&model);
    source.set_variable_positions(&distinctive_positions(&model));
    let line = robot_state_to_csv(&source, false, ',')
        .trim_end()
        .to_string();

    let mut exact = RobotState::new(&model);
    csv_to_robot_state(&mut exact, &line, ',').expect("the writer's own output must parse");
    let mut padded = RobotState::new(&model);
    csv_to_robot_state(&mut padded, &format!("{line},99.0,not-a-number"), ',')
        .expect("extra cells must not be an error");

    assert_eq!(padded.positions(), exact.positions());
}

// ---- cell content: numeric vs not ----------------------------------------

/// A cell that is not a number is an error naming both the variable and the
/// cell, rather than a silently defaulted position.
#[test]
fn a_cell_that_is_not_a_number_is_an_error() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let mut cells = vec!["0.0".to_string(); model.variable_count()];
    cells[1] = "nope".to_string();

    let error = csv_to_robot_state(&mut state, &cells.join(","), ',').unwrap_err();

    let name = &model.variable_names()[1];
    assert!(
        error.to_string().starts_with(&format!(
            "CSV parse error: variable {name:?} holds \"nope\""
        )),
        "got {error}"
    );
}

/// A rejected line leaves the state untouched: the reader collects every
/// cell before writing any. Upstream assigns as it parses, so its state is
/// left half-updated when `std::stod` throws.
#[test]
fn a_rejected_line_leaves_the_state_untouched() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let before = state.positions().to_vec();
    let mut cells: Vec<String> = distinctive_positions(&model)
        .iter()
        .map(f64::to_string)
        .collect();
    let last = cells.len() - 1;
    cells[last] = "nope".to_string();

    csv_to_robot_state(&mut state, &cells.join(","), ',').unwrap_err();

    assert_eq!(state.positions(), before.as_slice());
}

// ---- reading marks the transforms dirty ----------------------------------

/// A read must leave forward kinematics recomputed, not stale. Upstream
/// assigns through the raw `getVariablePositions()` pointer, which sets no
/// dirty flag (`doc/upstream-bugs.md`,
/// `stream-to-robot-state-bypasses-dirty-flags`), so the next
/// `getGlobalLinkTransform` there answers from the previous state.
/// The reference pose is recomputed from the positions the read actually
/// landed, not from the source state, so a writer that lost precision would
/// fail `a_round_trip_returns_every_position_bit_for_bit` alone.
#[test]
fn reading_a_state_marks_the_transforms_dirty() {
    let model = panda();
    let mut source = RobotState::new(&model);
    source.set_variable_positions(&distinctive_positions(&model));
    let line = robot_state_to_csv(&source, false, ',');

    let mut target = RobotState::new(&model);
    // Compute and cache FK at the *old* positions first: a missing dirty
    // flag is invisible if nothing was ever cached to go stale.
    let before = target
        .update()
        .global_link_transform("panda_link7")
        .expect("the fixture has a panda_link7");
    csv_to_robot_state(&mut target, &line, ',').expect("the writer's own output must parse");
    let after = target
        .update()
        .global_link_transform("panda_link7")
        .expect("the fixture has a panda_link7");

    let mut reference = RobotState::new(&model);
    reference.set_variable_positions(target.positions());
    let expected = reference
        .update()
        .global_link_transform("panda_link7")
        .expect("the fixture has a panda_link7");

    assert_ne!(
        before.translation.vector, after.translation.vector,
        "the link pose must follow the positions just read"
    );
    assert_eq!(
        after.translation.vector, expected.translation.vector,
        "and it must be the pose those positions imply"
    );
}

// ---- group ordering: given order vs model order --------------------------

/// The grouped writer emits its groups in the order given, not in the
/// model's variable order — reversing the argument reorders the line while
/// keeping the same set of variables.
#[test]
fn groups_are_emitted_in_the_order_they_are_given() {
    let model = panda();
    let state = RobotState::new(&model);

    let names = |groups: &[&str]| -> Vec<String> {
        robot_state_to_csv_by_groups(&state, groups, true, ',')
            .expect("both groups exist in the fixture")
            .split_once('\n')
            .expect("a header line must be present")
            .0
            .trim_end_matches(',')
            .split(',')
            .map(str::to_string)
            .collect()
    };
    let forward = names(&["hand", "panda_arm"]);
    let reversed = names(&["panda_arm", "hand"]);

    assert_ne!(forward, reversed, "the group order must reach the output");
    let sorted = |mut v: Vec<String>| {
        v.sort();
        v
    };
    assert_eq!(
        sorted(forward.clone()),
        sorted(reversed),
        "the same variables, only reordered"
    );
    assert_ne!(
        forward,
        model.variable_names().to_vec(),
        "and it must not collapse to the model's own variable order"
    );
}

/// A group appearing twice is emitted twice: upstream concatenates the
/// named groups without deduplicating (`conversions.cpp:533-555`).
///
/// Compared field-wise rather than as raw text, so that a writer which
/// dropped the trailing separator would fail
/// `the_grouped_line_keeps_upstreams_trailing_separator` alone.
#[test]
fn a_group_named_twice_is_emitted_twice() {
    let model = panda();
    let state = RobotState::new(&model);

    let header_fields = |groups: &[&str]| -> Vec<String> {
        robot_state_to_csv_by_groups(&state, groups, true, ',')
            .expect("the fixture has a hand group")
            .split_once('\n')
            .expect("a header line must be present")
            .0
            .trim_end_matches(',')
            .split(',')
            .map(str::to_string)
            .collect()
    };
    let once = header_fields(&["hand"]);
    let twice = header_fields(&["hand", "hand"]);

    assert_eq!(twice, [once.clone(), once].concat());
}

// ---- group lookup: known vs unknown --------------------------------------

/// An unknown group name is an error, not a null dereference — the whole of
/// `doc/upstream-bugs.md`'s `robot-state-to-stream-group-lookup-unchecked`.
/// The known group before it proves the lookup is per-entry rather than
/// validated once.
#[test]
fn an_unknown_group_is_an_error_rather_than_a_dereference() {
    let model = panda();
    let state = RobotState::new(&model);

    let error = robot_state_to_csv_by_groups(&state, &["panda_arm", "no_such_group"], true, ',')
        .unwrap_err();

    assert_eq!(error.to_string(), "no group named \"no_such_group\"");
}
