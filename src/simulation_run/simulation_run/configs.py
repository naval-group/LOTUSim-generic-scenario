"""
@file configs.py
@author Naval Group
@brief Data classes for simulation configuration parameters.

@version 0.1
@date 2026-03-03

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

from dataclasses import dataclass


@dataclass
class WaypointFollowerConfig:
    """
    Configuration parameters for the waypoint follower plugin.
    All fields are optional — unset fields are omitted from the XML
    and the C++ plugin falls back to its own defaults.

    Maps directly to the SDF parameters parsed in WaypointFollowerPlugin::load():
        guidance_mode           : Control strategy ("pid" or "bang_bang") (default: "pid")
        loop                    : Whether to loop over waypoints (default: false)
        range_tolerance         : Distance threshold to consider a waypoint reached in meters (default: 0.5)
        linear_accel_limit      : Max linear acceleration in m/s² (default: 0.5)
        angular_accel_limit     : Max angular acceleration in rad/s² (default: 0.5)
        linear_velocities_limits: (min, max) linear velocity in m/s (default: (0.0, 5.0))
        angular_velocities_limits: Max angular velocity in rad/s (default: 1.0)
        linear_pid              : Linear PID gains (Kp, Ki, Kd) (default: (0.5, 0.05, 0.1))
        angular_pid             : Angular PID gains (Kp, Ki, Kd) (default: (0.8, 0.05, 0.4))
    """

    guidance_mode: str | None = None  # "pid" or "bang_bang"
    loop: bool | None = None
    range_tolerance: float | None = None
    linear_accel_limit: float | None = None
    angular_accel_limit: float | None = None
    linear_velocities_limits: tuple[float, float] | None = None  # (min, max)
    angular_velocities_limits: float | None = None
    linear_pid: tuple[float, float, float] | None = None
    angular_pid: tuple[float, float, float] | None = None
