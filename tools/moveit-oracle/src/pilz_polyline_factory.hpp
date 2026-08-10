// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

// `TrajectoryGeneratorPolyline`'s constructor, behind a factory, because its
// header cannot be included alongside `trajectory_generator_lin.hpp`.
//
// Upstream declares three exception classes -- `LinTrajectoryConversionFailure`,
// `JointNumberMismatch`, `LinInverseForGoalIncalculable` -- in BOTH
// `trajectory_generator_lin.hpp` (:48, :50, :51) and
// `trajectory_generator_polyline.hpp` (:49, :51, :52), in the same
// `pilz_industrial_motion_planner` namespace and via the same
// `CREATE_MOVEIT_ERROR_CODE_EXCEPTION` macro, which expands to a class
// definition. Any translation unit including both therefore fails to compile
// with "redefinition of class". Upstream never hits it because each
// `trajectory_generator_*.cpp` includes only its own header; `oracle.cpp`
// wants all four generators in one file and does. See
// `doc/upstream-bugs.md`'s `polyline-header-redeclares-lin-exceptions`.
//
// So the POLYLINE header stays in `pilz_polyline_factory.cpp` alone, and
// `oracle.cpp` reaches the generator through the base-class pointer this
// returns. The base `TrajectoryGenerator` and `LimitsContainer` headers below
// are shared by both translation units without conflict.

#pragma once

#include <memory>
#include <string>

#include <moveit/robot_model/robot_model.hpp>
#include <pilz_industrial_motion_planner/limits_container.hpp>
#include <pilz_industrial_motion_planner/trajectory_generator.hpp>

namespace moveit_oracle
{

/// `TrajectoryGeneratorPolyline(robot_model, limits, group_name)`, returned as
/// the base pointer `oracle.cpp` already dispatches its other three
/// generators through.
std::unique_ptr<pilz_industrial_motion_planner::TrajectoryGenerator>
makePilzPolylineGenerator(const moveit::core::RobotModelConstPtr& robot_model,
                          const pilz_industrial_motion_planner::LimitsContainer& limits,
                          const std::string& group_name);

}  // namespace moveit_oracle
