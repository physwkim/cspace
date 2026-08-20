//! In-process roundtrip through the real database and `db/planner.db`:
//! write the start/goal waveforms, process `$(P)Plan`, read the outputs.
//! No sockets — `IocBuilder` builds the same `PvDatabase` the CA server
//! would serve.

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::sync::{Arc, RwLock};

use cspace_epics_ioc::service::PlannerService;
use cspace_epics_ioc::{PLAN_SUBROUTINE, SharedPlanner, plan_subroutine};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::ioc_builder::IocBuilder;
use epics_rs::base::server::record::Record;
use epics_rs::base::types::EpicsValue;

const DB: &str = include_str!("../db/planner.db");

/// Within-limits start pose for the panda arm (joint 4's range is entirely
/// negative, so all-zeros is not a valid state).
const READY: [f64; 7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];

fn panda_service() -> PlannerService {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    PlannerService::from_files(
        Path::new(&format!("{root}/panda.urdf")),
        Path::new(&format!("{root}/panda.srdf")),
        "panda_arm",
        &[],
    )
    .expect("the panda fixture must load")
}

async fn boot(slot: SharedPlanner) -> Arc<PvDatabase> {
    let mut macros = HashMap::new();
    macros.insert("P".to_string(), "T:".to_string());
    let (db, _) = IocBuilder::new()
        .db_string(DB, &macros)
        .expect("db/planner.db must parse")
        .register_subroutine(PLAN_SUBROUTINE, move |record: &mut dyn Record| {
            plan_subroutine(&slot, record)
        })
        .build()
        .await
        .expect("the database must build");
    db
}

fn put(db: &PvDatabase, record: &str, field: &str, value: EpicsValue) {
    let arc = db.get_record(record).expect("record must exist");
    let mut instance = arc.write();
    instance
        .record
        .put_field(field, value)
        .expect("put must succeed");
}

fn get(db: &PvDatabase, record: &str, field: &str) -> EpicsValue {
    let arc = db.get_record(record).expect("record must exist");
    let instance = arc.read();
    instance
        .record
        .get_field(field)
        .unwrap_or_else(|| panic!("{record}.{field} must exist"))
}

async fn process_plan(db: &PvDatabase) {
    let mut visited = HashSet::new();
    db.process_record_with_links("T:Plan", &mut visited, 0)
        .await
        .expect("processing T:Plan must not error");
}

fn as_doubles(value: EpicsValue) -> Vec<f64> {
    match value {
        EpicsValue::DoubleArray(v) => v,
        other => panic!("expected a double array, got {other:?}"),
    }
}

fn as_long(value: EpicsValue) -> i32 {
    match value {
        EpicsValue::Long(v) => v,
        other => panic!("expected a long, got {other:?}"),
    }
}

#[epics_rs::base::epics_test]
async fn plan_roundtrip_through_the_database() {
    let slot: SharedPlanner = Arc::new(RwLock::new(Some(panda_service())));
    let db = boot(slot).await;

    let mut goal = READY;
    goal[0] = 0.5;
    put(
        &db,
        "T:Start",
        "VAL",
        EpicsValue::DoubleArray(READY.to_vec()),
    );
    put(&db, "T:Goal", "VAL", EpicsValue::DoubleArray(goal.to_vec()));
    process_plan(&db).await;

    assert_eq!(
        get(&db, "T:Plan", "VAL"),
        EpicsValue::Long(0),
        "the subroutine status must be success; message: {:?}",
        get(&db, "T:Plan", "VALE"),
    );
    assert_eq!(as_long(get(&db, "T:NJoints", "VAL")), 7);
    let n_points = as_long(get(&db, "T:NPoints", "VAL")) as usize;
    assert!(n_points >= 2, "a plan must have at least two waypoints");

    let traj = as_doubles(get(&db, "T:Traj", "VAL"));
    assert_eq!(
        traj.len(),
        n_points * 7,
        "OUTA must carry the full trajectory"
    );
    for (a, b) in traj[..7].iter().zip(READY) {
        assert!((a - b).abs() < 1e-6, "first waypoint must be the start");
    }
    for (a, b) in traj[traj.len() - 7..].iter().zip(goal) {
        assert!((a - b).abs() < 1e-6, "last waypoint must be the goal");
    }

    let times = as_doubles(get(&db, "T:Times", "VAL"));
    assert_eq!(times.len(), n_points);
    assert_eq!(times[0], 0.0);
    assert!(
        times[n_points - 1] > 0.0,
        "TOTG must produce nonzero timing"
    );
    for pair in times.windows(2) {
        assert!(pair[0] < pair[1], "times must be strictly increasing");
    }

    assert_eq!(
        get(&db, "T:Message", "VAL"),
        EpicsValue::String("OK".into())
    );
    match get(&db, "T:JointNames", "VAL") {
        EpicsValue::StringArray(names) => {
            assert_eq!(names.len(), 7);
            assert_eq!(names[0].to_string(), "panda_joint1");
        }
        other => panic!("T:JointNames must be a string array, got {other:?}"),
    }
}

#[epics_rs::base::epics_test]
async fn unconfigured_service_raises_the_brsv_alarm() {
    let slot: SharedPlanner = Arc::new(RwLock::new(None));
    let db = boot(slot).await;

    put(
        &db,
        "T:Start",
        "VAL",
        EpicsValue::DoubleArray(READY.to_vec()),
    );
    put(
        &db,
        "T:Goal",
        "VAL",
        EpicsValue::DoubleArray(READY.to_vec()),
    );
    process_plan(&db).await;

    assert_eq!(get(&db, "T:Plan", "VAL"), EpicsValue::Long(-1));
    // On a nonzero return the OUTx links do not fire (aSubRecord.c:232-239):
    // the reason lives in Plan.VALE, and the fan-out records keep their
    // previous values (here their initial ones).
    match get(&db, "T:Plan", "VALE") {
        EpicsValue::String(message) => assert!(
            message.to_string().contains("not configured"),
            "VALE must name the cause, got {message:?}"
        ),
        other => panic!("T:Plan.VALE must be a string, got {other:?}"),
    }
    assert_eq!(get(&db, "T:Message", "VAL"), EpicsValue::String("".into()));
    assert_eq!(as_long(get(&db, "T:NPoints", "VAL")), 0);
}

#[epics_rs::base::epics_test]
async fn wrong_joint_count_fails_with_a_message() {
    let slot: SharedPlanner = Arc::new(RwLock::new(Some(panda_service())));
    let db = boot(slot).await;

    put(&db, "T:Start", "VAL", EpicsValue::DoubleArray(vec![0.0; 3]));
    put(
        &db,
        "T:Goal",
        "VAL",
        EpicsValue::DoubleArray(READY.to_vec()),
    );
    process_plan(&db).await;

    assert_eq!(get(&db, "T:Plan", "VAL"), EpicsValue::Long(-1));
    match get(&db, "T:Plan", "VALE") {
        EpicsValue::String(message) => assert!(
            message.to_string().contains("start has 3"),
            "VALE must name the count mismatch, got {message:?}"
        ),
        other => panic!("T:Plan.VALE must be a string, got {other:?}"),
    }
}
