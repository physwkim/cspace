// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

// The one translation unit that may include
// `trajectory_generator_polyline.hpp` -- see this file's header for the
// upstream redefinition it is isolating.

#include "pilz_polyline_factory.hpp"

#include <pilz_industrial_motion_planner/trajectory_generator_polyline.hpp>

namespace moveit_oracle
{

std::unique_ptr<pilz_industrial_motion_planner::TrajectoryGenerator>
makePilzPolylineGenerator(const moveit::core::RobotModelConstPtr& robot_model,
                          const pilz_industrial_motion_planner::LimitsContainer& limits,
                          const std::string& group_name)
{
  return std::make_unique<pilz_industrial_motion_planner::TrajectoryGeneratorPolyline>(robot_model, limits, group_name);
}

}  // namespace moveit_oracle
