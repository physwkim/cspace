//! cspace planning IOC binary.
//!
//! Usage:
//!   cargo run -p cspace-epics-ioc -- st.cmd
//!
//! One iocsh startup command loads the robot at boot:
//!
//! ```text
//! CspacePlannerConfig(urdfPath, srdfPath, groupName [, meshPackage, meshDir])
//! ```
//!
//! `meshPackage`/`meshDir` resolve `package://<meshPackage>/...` URIs in
//! the URDF's collision elements; omitting them loads the model with no
//! collision meshes.

use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

use cspace_epics_ioc::service::PlannerService;
use cspace_epics_ioc::{PLAN_SUBROUTINE, SharedPlanner, plan_subroutine};
use epics_rs::base::error::CaResult;
use epics_rs::base::server::iocsh::registry::{
    ArgDesc, ArgType, ArgValue, CommandContext, CommandDef, CommandOutcome,
};
use epics_rs::base::server::record::Record;
use epics_rs::ca::server::ioc_app::IocApplication;

fn string_arg(args: &[ArgValue], i: usize) -> Result<String, String> {
    match args.get(i) {
        Some(ArgValue::String(s)) => Ok(s.clone()),
        _ => Err(format!("argument {i} must be a string")),
    }
}

#[epics_rs::base::epics_main]
async fn main() -> CaResult<()> {
    let script = match std::env::args().nth(1) {
        Some(s) if !s.starts_with('-') => s,
        _ => {
            eprintln!("Usage: cspace-epics-ioc <st.cmd>");
            std::process::exit(1);
        }
    };

    // Lets st.cmd reference this crate's directory (db file, fixtures).
    epics_rs::base::runtime::env::set_default("CSPACE_IOC", env!("CARGO_MANIFEST_DIR"));

    let slot: SharedPlanner = Arc::new(RwLock::new(None));

    let mut app = IocApplication::new();

    // CspacePlannerConfig(urdfPath, srdfPath, groupName [, meshPackage, meshDir])
    {
        let slot = slot.clone();
        app = app.register_startup_command(CommandDef::new(
            "CspacePlannerConfig",
            vec![
                ArgDesc {
                    name: "urdfPath",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "srdfPath",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "groupName",
                    arg_type: ArgType::String,
                    optional: false,
                },
                ArgDesc {
                    name: "meshPackage",
                    arg_type: ArgType::String,
                    optional: true,
                },
                ArgDesc {
                    name: "meshDir",
                    arg_type: ArgType::String,
                    optional: true,
                },
            ],
            "CspacePlannerConfig urdfPath srdfPath groupName [meshPackage meshDir]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let urdf = string_arg(args, 0)?;
                let srdf = string_arg(args, 1)?;
                let group = string_arg(args, 2)?;
                let meshes = match (args.get(3), args.get(4)) {
                    (Some(ArgValue::String(package)), Some(ArgValue::String(dir))) => {
                        vec![(package.clone(), PathBuf::from(dir))]
                    }
                    (None, None) => Vec::new(),
                    _ => return Err("meshPackage and meshDir must be given together".into()),
                };
                let service =
                    PlannerService::from_files(Path::new(&urdf), Path::new(&srdf), &group, &meshes)
                        .map_err(|e| format!("CspacePlannerConfig failed: {e}"))?;
                println!(
                    "CspacePlannerConfig: group '{group}' with {} joints",
                    service.joint_names().len()
                );
                *slot
                    .write()
                    .map_err(|_| "planner state poisoned".to_string())? = Some(service);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    {
        let slot = slot.clone();
        app = app.register_subroutine(PLAN_SUBROUTINE, move |record: &mut dyn Record| {
            plan_subroutine(&slot, record)
        });
    }

    app.startup_script(&script)
        .run(epics_rs::ca::server::run_ca_ioc)
        .await
}
