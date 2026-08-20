# cspace planning IOC startup: panda fixture with collision meshes.
#
#   cargo run -p cspace-epics-ioc -- st.cmd
#
# CSPACE_IOC is set by main() to the directory of this crate.

CspacePlannerConfig("$(CSPACE_IOC)/../../fixtures/panda.urdf", "$(CSPACE_IOC)/../../fixtures/panda.srdf", "panda_arm", "moveit_resources_panda_description", "$(CSPACE_IOC)/../../fixtures/meshes/panda_description")

dbLoadRecords("$(CSPACE_IOC)/db/planner.db", "P=CSPACE:PLAN:")

iocInit
