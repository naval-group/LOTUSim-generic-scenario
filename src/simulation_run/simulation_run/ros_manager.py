"""
@file ros_manager.py
@author Naval Group
@brief Core runtime orchestration for generic scenario simulations.

@details
This module manages the execution flow of simulations.

It provides:
- ROS 2 executor management with graceful shutdown
- Agent initialization via `AgentsManager`
- Pose monitoring for agent liveness tracking

@note
Designed to serve as the central runtime layer of the simulation system,
coordinating agent management, monitoring, and performance evaluation.

@version 0.1
@date 2026-03-03

This program and the accompanying materials are made available under the
terms of the Eclipse Public License 2.0 which is available at:
http://www.eclipse.org/legal/epl-2.0

SPDX-License-Identifier: EPL-2.0

Copyright (c) 2025 Naval Group
"""

import logging
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from simulation_run import agents_manager

# -------------------------------------------------------------------------
# Logging Configuration
# -------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# -------------------------------------------------------------------------
# Global Shutdown State
# -------------------------------------------------------------------------
shutdown_flag = False
shutdown_flag_lock = threading.Lock()


# -------------------------------------------------------------------------
# Initialization
# -------------------------------------------------------------------------
def initialize_ros_components(
    executor: Any,
    agents: Dict[str, Any],
    world_name: str,
    aerial_domain: bool = False,
) -> agents_manager.AgentsManager:
    """
    Initialize simulation components using a full agents dictionary.

    Args:
        executor: ROS 2 executor to register agent nodes.
        agents: Dictionary containing agent types, their number, poses, model paths, and xdyn flag.
        world_name: Name of the simulation world.
        aerial_domain: Whether to enable aerial domain.

    Returns:
        AgentsManager instance with all agents initialized and registered.
    """
    time.sleep(1.0)

    # Initialize and spawn all agents
    manager = agents_manager.AgentsManager()

    manager.add_agents(agents, world_name, executor, aerial_domain)
    return manager


# -------------------------------------------------------------------------
# Executor Management
# -------------------------------------------------------------------------


def run_executor(executor: Any, max_simulation_time: Optional[float] = None) -> None:
    """
    Run a ROS 2 executor loop until a shutdown condition is met.

    The loop exits when one of the following occurs:
        - `shutdown_flag` becomes True
        - `rclpy.ok()` returns False
        - `max_simulation_time` (if provided) is exceeded
        - An exception is raised during executor spinning

    Args:
        executor: ROS 2 executor instance to spin.
        max_simulation_time: Optional maximum runtime in seconds.

    Notes:
        The executor is polled using `spin_once(timeout_sec=0.1)`
        to periodically check shutdown conditions.
    """
    start_time = time.time()
    try:
        while rclpy.ok():
            elapsed = time.time() - start_time

            # Check external shutdown flag
            with shutdown_flag_lock:
                if shutdown_flag:
                    logging.info(f"run_executor EXIT: shutdown_flag=True at t={elapsed:.1f}s")
                    break

            # Check maximum simulation time
            if max_simulation_time is not None and elapsed > max_simulation_time:
                logging.info(f"run_executor EXIT: max_simulation_time={max_simulation_time}s reached")
                break

            # Spin executor once
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                logging.error(
                    f"run_executor EXCEPTION at t={elapsed:.1f}s: " f"{type(e).__name__}: {e}",
                    exc_info=True,
                )
                break

        if not rclpy.ok():
            logging.info(f"run_executor EXIT: rclpy.ok()=False at t={time.time() - start_time:.1f}s")

    except BaseException as e:
        logging.error(
            f"run_executor BASE EXCEPTION: {type(e).__name__}: {e}",
            exc_info=True,
        )
        raise

    finally:
        logging.info(f"run_executor: cleanup reached at t={time.time() - start_time:.1f}s")