from pkg_robot.robot import RobotManager
import numpy as np


class Coordinator:
    """
    Coordinator class - TODO document
    """
    def __init__(self, robot_manager: RobotManager, robot_ids: list[str], ts: float) -> None:
        self.robot_manager = robot_manager
        self.robot_ids = robot_ids
        self.ts = ts

    def evaluate(self, kt: int) -> list[float]:
        """
        Placeholder TODO

        TODO Check if my delay/earlyness impacts other robots. If they don't its probably fine to be early
        Args:
            kt: The current time step
        Returns:
            delay: The sum of the schedule delay for all robots, in seconds
        """

        # Skip calculations intially
        if kt == 0:
            return [0.0]*len(self.robot_ids)

        current_time = kt * self.ts
        delays = []
        for rid in self.robot_ids:
            robot_planner = self.robot_manager.get_planner(rid)
            robot_state = self.robot_manager.get_robot_state(rid)
            max_vel = self.robot_manager.get_robot(rid).config.lin_vel_max

            ref_path = robot_planner._ref_path
            ref_path_times = robot_planner._ref_path_time
            target_node = robot_planner.current_target_node
            path_idx = None
            #Find the node that we should be heading towards
            for i, ref_time in enumerate(ref_path_times):
                if current_time > ref_time:
                    continue
                else:
                    path_idx = i
                    break
            
            if path_idx is None:
                # If the robot has reached the finish line, set zero delay
                delays.append(0)
                continue

            scheduled_node = ref_path[path_idx]
            prev_sched_node = ref_path[path_idx - 1] 
            scheduled_departure_time = ref_path_times[path_idx-1]
            scheduled_arrival_time = ref_path_times[path_idx]
            node_idx = i
            
            
            if scheduled_node == target_node:
                # If we're on the right segment, check the local plan: delay = time since we should have been at the last waypoint
                # This is the easy case
                # If we can cover the remaining distance by running at maximum speed, then no delay
                delay = self._calcuate_edge_delay(current_time, robot_state, scheduled_node, prev_sched_node, 
                                                  scheduled_departure_time, scheduled_arrival_time, max_vel)
                delays.append(delay)
                continue
            
            # If we're not, delay = time until next node + time since we should have left next node
            # Node index we are heading towards
            robot_target_i = self._get_target_node_index(ref_path, target_node)

            pre_target = ref_path[robot_target_i - 1]
            robot_target_departure = ref_path_times[robot_target_i - 1]
            robot_target_arrival = ref_path_times[robot_target_i]
            next_node_delay = self._calcuate_edge_delay(current_time, robot_state, target_node, 
                                                            pre_target, robot_target_departure, robot_target_arrival, max_vel)

            # All nodes we should have reached but haven't
            missing_nodes_delay = scheduled_departure_time - robot_target_arrival

            # How far we should have made it along the scheduled current edge
            sched_current_edge_delay = self._calcuate_edge_delay(current_time, prev_sched_node, scheduled_node, prev_sched_node, 
                                                                 scheduled_departure_time, scheduled_arrival_time, max_vel)

            delay = next_node_delay + missing_nodes_delay + sched_current_edge_delay
            delays.append(delay)

        return delays

    def _get_target_node_index(self, ref_path, target_node):
        robot_target_i = None
        for i, node in enumerate(ref_path):
            if node == target_node:
                robot_target_i = i
                break
        return robot_target_i

    def _calcuate_edge_delay(self, current_time, robot_state, scheduled_node, prev_sched_node, departure_time, arrival_time, max_vel = None):
        """
        Estimate time delay between robot and scheduled position via linear interpolation.
        Args:
            current_time (float): observation time.
            robot_state (Sequence[float]): (x, y) position.
            scheduled_node: (x, y) nodes
            prev_sched_node (Sequence[float]): (x, y) nodes.
            departure_time (float): schedule times in seconds
            arrival_time (float): schedule times in seconds.
            max_vel (float|None): optional robot maximum velocity, if specified, the delay will 
                return 0 if the robot can recover on its own when going max speed
        Returns:
            float: estimated delay (seconds).
        """
        
        path_delta_x = scheduled_node[0] - prev_sched_node[0]
        path_delta_y = scheduled_node[1] - prev_sched_node[1]
        time_to_traverse = arrival_time - departure_time
        velocity = np.sqrt(path_delta_x**2 + path_delta_y**2) / time_to_traverse

        if current_time > arrival_time: # If we should have finished already, calculate as though we should be at the end
            scheduled_x = scheduled_node[0]
            scheduled_y = scheduled_node[1]
        else:
            scheduled_x = prev_sched_node[0] + path_delta_x * (current_time - departure_time) / time_to_traverse
            scheduled_y = prev_sched_node[1] + path_delta_y * (current_time - departure_time) / time_to_traverse

        offset_x = robot_state[0] - scheduled_x
        offset_y = robot_state[1] - scheduled_y
        total_offset = np.sqrt(offset_x**2 + offset_y**2)

        # If the robot's maximum velocity is provided, ignore the delay calculation if the MPC controller can recover the delay on its own.
        if max_vel is not None :
            delay = total_offset / max_vel
            fastest_possible = total_offset / max_vel
            if arrival_time > current_time + fastest_possible:
                return 0
            else:
                return delay
        else:
            return total_offset / velocity

    def reschedule(self, kt: int) -> None:
        """
        1. Go to next node, if not possible, go back
        2. Figure out what jobs remain and rebuild the problem json file using these jobs
        3. Figure out what edges are blocked
        4. Call the scheduler
        5. Implement the new schedule
        6. Resume running
        """
        self._set_all_idle()
        time = kt * self.ts

        for rid in self.robot_ids:
            robot_planner = self.robot_manager.get_planner(rid)
            target_node = robot_planner.current_target_node
            ref_path = robot_planner._ref_path
            target_node_index = self._get_target_node_index(ref_path, target_node)
            prev_node = ref_path[target_node_index-1]
            ref_path_times = robot_planner._ref_path_time
            prev_time = ref_path_times[target_node_index-1]
            robot_planner.load_path([prev_node, target_node], [prev_time, time])
    
    def _set_all_idle(self) -> None:
        for rid in self.robot_ids:
            self.robot_manager.set_robot_idle(rid, True)

    

    def build_new_schedule(self) -> None:
        raise NotImplementedError