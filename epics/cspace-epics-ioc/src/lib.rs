//! EPICS front-end for cspace motion planning.
//!
//! The IOC's compute seam is an `aSub` record (`db/planner.db`'s
//! `$(P)Plan`) whose `SNAM` names [`PLAN_SUBROUTINE`]: inputs are pulled
//! through `INPA..INPE` when the record processes, [`plan_subroutine`] runs
//! [`service::PlannerService::plan`] synchronously, and the results land in
//! `VALA..VALF` (pushed onward by `OUTx` links). A client that writes the
//! start/goal waveforms and then does a put-callback on `$(P)Plan.PROC`
//! observes completion exactly when the plan is done.
//!
//! Field contract (mirrored by `db/planner.db`):
//!
//! | field | meaning |
//! |-------|---------|
//! | `A`   | start joint positions (`DOUBLE[]`) |
//! | `B`   | goal joint positions (`DOUBLE[]`) |
//! | `C`   | planner id (`STRING`, empty → default; set via a direct `caput $(P)Plan.C`) |
//! | `D`   | max velocity scaling (`DOUBLE`, outside `(0,1]` → 1.0) |
//! | `E`   | max acceleration scaling (`DOUBLE`, outside `(0,1]` → 1.0) |
//! | `VALA`| flattened trajectory, row-major `[point][joint]` |
//! | `VALB`| seconds from start, one per point |
//! | `VALC`| number of points (`LONG`) |
//! | `VALD`| number of joints (`LONG`) |
//! | `VALE`| status message (`STRING`, `"OK"` on success) |
//! | `VALF`| joint names (`STRING[]`) |
//!
//! On failure the subroutine empties `VALA`/`VALB`, zeroes `VALC`, writes
//! the reason to `VALE`, and returns `-1` — the framework publishes that
//! as the record's `VAL` and raises a `BRSV`-severity alarm. Matching C
//! `aSubRecord.c:232-239`, the `OUTx` links fire only on a `0` return, so
//! after a failure the fan-out records (`$(P)Message`, `$(P)NPoints`, …)
//! keep their previous values; read the failure reason from
//! `$(P)Plan.VALE` directly.

// Keeps `RrtConnectManager`'s `linkme` registration linked into every
// binary and test binary of this crate; without a named reference the
// linker drops the object file the distributed-slice entry lives in and
// `resolve_planner("rrt_connect")` fails at runtime (see
// ros/cspace-ros/src/lib.rs for the same line).
use cspace_planners as _;

use std::sync::{Arc, RwLock};

use epics_rs::base::error::CaResult;
use epics_rs::base::server::record::Record;
use epics_rs::base::types::EpicsValue;

use crate::service::PlannerService;

pub mod service;

/// The subroutine name `db/planner.db` wires into the `aSub`'s `SNAM`.
pub const PLAN_SUBROUTINE: &str = "cspacePlan";

/// The slot `CspacePlannerConfig` fills at startup and [`plan_subroutine`]
/// reads on every process. `None` until the st.cmd command has run.
pub type SharedPlanner = Arc<RwLock<Option<PlannerService>>>;

/// The `aSub` body: pulls `A..E`, plans, writes `VALA..VALF`.
///
/// Returns `Ok(0)` on success and `Ok(-1)` on any failure (after writing
/// the reason to `VALE`); an `Err` is only raised when a field write
/// itself fails, which is a database-level defect rather than a planning
/// outcome.
pub fn plan_subroutine(slot: &SharedPlanner, record: &mut dyn Record) -> CaResult<i64> {
    match try_plan(slot, record) {
        Ok(planned) => {
            record.put_field("VALA", EpicsValue::DoubleArray(planned.positions))?;
            record.put_field("VALB", EpicsValue::DoubleArray(planned.times_from_start))?;
            record.put_field("VALC", EpicsValue::Long(planned.n_points as i32))?;
            record.put_field("VALD", EpicsValue::Long(planned.joint_names.len() as i32))?;
            record.put_field("VALE", EpicsValue::String("OK".into()))?;
            record.put_field(
                "VALF",
                EpicsValue::StringArray(
                    planned
                        .joint_names
                        .iter()
                        .map(|n| n.as_str().into())
                        .collect(),
                ),
            )?;
            Ok(0)
        }
        Err(message) => {
            record.put_field("VALA", EpicsValue::DoubleArray(Vec::new()))?;
            record.put_field("VALB", EpicsValue::DoubleArray(Vec::new()))?;
            record.put_field("VALC", EpicsValue::Long(0))?;
            record.put_field("VALE", EpicsValue::String(status_string(&message).into()))?;
            Ok(-1)
        }
    }
}

/// The fallible half of [`plan_subroutine`], with a human-readable error.
fn try_plan(
    slot: &SharedPlanner,
    record: &dyn Record,
) -> Result<crate::service::PlannedTrajectory, String> {
    let start = double_array_field(record, "A")?;
    let goal = double_array_field(record, "B")?;
    let planner_id = string_field(record, "C")?;
    let vel_scale = double_field(record, "D")?;
    let acc_scale = double_field(record, "E")?;

    let guard = slot
        .read()
        .map_err(|_| "planner state poisoned".to_string())?;
    let service = guard
        .as_ref()
        .ok_or_else(|| "not configured: run CspacePlannerConfig in st.cmd".to_string())?;

    let planned = service
        .plan(&start, &goal, &planner_id, vel_scale, acc_scale)
        .map_err(|e| e.to_string())?;

    // The aSub's output allocations are fixed at load time; refuse to
    // truncate a trajectory silently.
    let position_capacity = usize_field(record, "NOVA")?;
    if planned.positions.len() > position_capacity {
        return Err(format!(
            "trajectory needs {} elements, NOVA is {position_capacity}; raise TRAJCAP",
            planned.positions.len()
        ));
    }
    let point_capacity = usize_field(record, "NOVB")?;
    if planned.n_points > point_capacity {
        return Err(format!(
            "trajectory has {} points, NOVB is {point_capacity}; raise PTSCAP",
            planned.n_points
        ));
    }
    Ok(planned)
}

/// EPICS `STRING` fields hold 40 bytes; keep the NUL's place.
fn status_string(message: &str) -> String {
    message.chars().take(39).collect()
}

fn double_array_field(record: &dyn Record, name: &str) -> Result<Vec<f64>, String> {
    match record.get_field(name) {
        Some(EpicsValue::DoubleArray(values)) => Ok(values),
        // The FTA/NOA declaration in db/planner.db types this cell as a
        // double array; a scalar means the declaration was lost and the
        // link fetch reduced the source to one element — fail loudly
        // instead of planning on a garbled input.
        other => Err(format!("field {name} is not a double array: {other:?}")),
    }
}

fn double_field(record: &dyn Record, name: &str) -> Result<f64, String> {
    match record.get_field(name) {
        Some(EpicsValue::Double(value)) => Ok(value),
        Some(EpicsValue::DoubleArray(values)) => Ok(values.first().copied().unwrap_or(0.0)),
        other => Err(format!("field {name} is not a double: {other:?}")),
    }
}

fn string_field(record: &dyn Record, name: &str) -> Result<String, String> {
    match record.get_field(name) {
        Some(EpicsValue::String(value)) => Ok(value.to_string()),
        Some(EpicsValue::StringArray(values)) => {
            Ok(values.first().map(|v| v.to_string()).unwrap_or_default())
        }
        other => Err(format!("field {name} is not a string: {other:?}")),
    }
}

fn usize_field(record: &dyn Record, name: &str) -> Result<usize, String> {
    let value = record.get_field(name);
    match value {
        Some(EpicsValue::Long(v)) => usize::try_from(v).map_err(|_| format!("{name} < 0")),
        Some(EpicsValue::ULong(v)) => Ok(v as usize),
        Some(EpicsValue::Short(v)) => usize::try_from(v).map_err(|_| format!("{name} < 0")),
        Some(EpicsValue::UShort(v)) => Ok(v as usize),
        Some(EpicsValue::Int64(v)) => usize::try_from(v).map_err(|_| format!("{name} < 0")),
        Some(EpicsValue::UInt64(v)) => usize::try_from(v).map_err(|_| format!("{name} too big")),
        other => Err(format!("field {name} is not an integer: {other:?}")),
    }
}
