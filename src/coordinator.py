from configs import CircularRobotSpecification
from pkg_robot.robot import RobotManager
import numpy as np
import pandas as pd
import json
import copy
from itertools import combinations

from pkg_sche.sp_comsat.Compo_slim import Compo_slim
from pkg_motion_plan import GlobalPathCoordinator


class Coordinator:
    """
    This class contains most coordinator code, split over many functions. It is initialized once in the beginning and then maintains its own state.
    """
    def __init__(self, robot_manager: RobotManager, robot_ids: list[str], ts: float, graph_path: str, map_path: str, config_robot: CircularRobotSpecification, schedule: str|None = None) -> None:
        self.robot_manager = robot_manager
        self.robot_ids = robot_ids
        self.ts = ts
        self.task = schedule
        self.graph_path = graph_path
        self.config_robot = config_robot
        self.map_path = map_path
        self.mode = 'normal'  # normal or rescheduling
        self.new_schedule_path = None
        self.occupied_for_reset = []

    def get_mode(self) -> str:
        return self.mode

    def evaluate(self, kt: int, threshold: float) -> list[float]:
        """
        Calculates the delay for each robot and returns a list of delays in seconds. Also sets the coordinator mode to 'rescheduling' if at least one robot has a greater delay than ``threshold``.
        Args:
            kt: The current time step
            threshold: The greatest delay of a single robot that can be tolerated before rescheduling is started
        Returns:
            delays: A list of delays per robot, in seconds
        """

        # Skip calculations at the first time step.
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
            schedule_idx = None

            target_is_final = False
            # If the time is greater than the finishing time, we should be heading towards the final node
            if current_time > ref_path_times[-1]:
                schedule_idx = len(ref_path_times) - 1
                target_is_final = True
            # else:
            # #Find the node that we should be heading towards
            for i, ref_time in enumerate(ref_path_times):
                if current_time > ref_time:
                    continue
                schedule_idx = i
                break
                
            if schedule_idx is None:
                raise ValueError("No valid target node. Something is wrong...")

            scheduled_node = ref_path[schedule_idx]
            prev_sched_node = ref_path[schedule_idx - 1] 
            scheduled_departure_time = ref_path_times[schedule_idx-1]
            scheduled_arrival_time = ref_path_times[schedule_idx]

            if target_is_final:
                # Check if we're close to the target, if we are, we are finished and there is no delay
                target = np.array([*target_node])
                if np.linalg.norm((robot_state[:2] - target), 2) < 0.1:
                    delays.append(0)
                    continue

            
            if scheduled_node == target_node:
                # If we're on the right segment, check the local plan: delay = time since we should have been at the last waypoint
                # This is the easy case
                # If we can cover the remaining distance by running at maximum speed, then no delay
                delay = self.calcuate_edge_delay(current_time, robot_state, scheduled_node, prev_sched_node, 
                                                  scheduled_departure_time, scheduled_arrival_time, max_vel)
                delays.append(delay)
                continue
            
            # If we're not, delay = time until next node + time since we should have left next node
            # Node index we are heading towards
            robot_target_i = self.get_target_node_index(ref_path, target_node)

            pre_target = ref_path[robot_target_i - 1]
            robot_target_departure = ref_path_times[robot_target_i - 1]
            robot_target_arrival = ref_path_times[robot_target_i]
            next_node_delay = self.calcuate_edge_delay(current_time, robot_state, target_node, 
                                                            pre_target, robot_target_departure, robot_target_arrival, max_vel)

            # All nodes we should have reached but haven't
            missing_nodes_delay = scheduled_departure_time - robot_target_arrival

            # How far we should have made it along the scheduled current edge
            sched_current_edge_delay = self.calcuate_edge_delay(current_time, prev_sched_node, scheduled_node, prev_sched_node, 
                                                                 scheduled_departure_time, scheduled_arrival_time, max_vel)

            delay = next_node_delay + missing_nodes_delay + sched_current_edge_delay
            delays.append(delay)

        if max(delays) > threshold:
            self.mode = "delayed"

        return delays

    def get_target_node_index(self, ref_path: list[tuple], target_node:tuple) -> int:
        """
        Given a target node pair coordinate, get that nodes first apperance in the coordinate list
        
        :param ref_path: The reference path
        :param target_node: Description
        :return: Description
        :rtype: Any | Literal[0] | None
        """
        for i, node in enumerate(ref_path):
            if node == target_node:
                return i

    def calcuate_edge_delay(self, current_time: float, robot_state: list[float], scheduled_node: tuple[float], prev_sched_node: tuple[float], departure_time: float, arrival_time: float, max_vel: float|None = None):
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

    def reschedule_and_reposition(self, kt: int) -> None:
        """
        Generate the new schedule based on current robot postion and set repositioning schedule
        
        :param self: Description
        :param kt: Description
        :type kt: int
        """

        time = kt * self.ts

        # Build the new schedule and save it to file
        self.new_schedule_path, rescheduling_info, nodes_dict = self.build_new_schedule(kt)
        self.set_all_idle_mode(True)

        # Send all robots to the node from where they will resume running
        for rid in self.robot_ids:
            if rid not in rescheduling_info:
                continue

            info = rescheduling_info[rid]
            start_node_name = info['start']
            other_node_name = info['other']
            
            start_node_coord = [nodes_dict[start_node_name]['x'], nodes_dict[start_node_name]['y']]
            other_node_coord = [nodes_dict[other_node_name]['x'], nodes_dict[other_node_name]['y']]
            
            robot_planner = self.robot_manager.get_planner(rid)
            ref_path = robot_planner._ref_path
            ref_path_times = robot_planner._ref_path_time
            
            other_idx = self.get_target_node_index(ref_path, other_node_coord)
            
            if other_idx is None:
                 # Try to find start node index and infer other
                 start_idx = self.get_target_node_index(ref_path, start_node_coord)
                 if start_idx is not None:
                     # If start is at i, other is likely i-1
                     other_idx = start_idx - 1
                 else:
                     raise ValueError(f"Could not find nodes in ref_path for {rid}")

            prev_time = ref_path_times[other_idx]
            robot_state = self.robot_manager.get_robot_state(rid)
            
            self.occupied_for_reset.append(start_node_coord)
            # Set the schedule for each robot to go the start node, and as quickly as possible.
            self.robot_manager.add_schedule(rid, robot_state, [other_node_coord, start_node_coord], [prev_time, time])

        self.mode = "stopping_for_rescheduling"

    def write_new_schedule(self, kt: int, schedule_path = None) -> None:
        """
        Docstring for write_new_schedule
        
        :param self: Description
        :param kt: Description
        :type kt: int
        :param schedule_path: Description
        """
        time = kt*self.ts

        # Reset the occupied list for the next rescheduling
        self.occupied_for_reset = []

        if schedule_path is None:
            if self.new_schedule_path is None:
                raise ValueError("No new schedule to write!")
            schedule_path = self.new_schedule_path

        self.set_all_idle_mode(True),

        _, _, _, _, _, solution = Compo_slim(self.new_schedule_path)
        if solution == {}:
            raise ValueError("No valid scheduling solution!")

            
        schedule_df = self.create_schedule_df(solution)
        new_gpc = GlobalPathCoordinator(schedule_df)
        # Loading graph and map, as suggested by documentation
        new_gpc.load_graph_from_json(self.graph_path)
        new_gpc.load_map_from_json(self.map_path, inflation_margin=self.config_robot.vehicle_width+self.config_robot.vehicle_margin)

        for rid in self.robot_ids:
            robot_state = self.robot_manager.get_robot_state(rid)

            path_coords, path_times = new_gpc.get_robot_schedule(rid, time_offset=time)
            
            first_time = path_times[0]
            if first_time != time:
                print(f"{rid} start time is not {time} but {first_time}, setting to time")
                path_times[0] = time
            self.robot_manager.add_schedule(rid, robot_state, path_coords, path_times)
        
        self.set_all_idle_mode(False)
        self.mode = "normal"

    def set_all_idle_mode(self, mode: bool) -> None:
        for rid in self.robot_ids:
            self.robot_manager.set_robot_idle(rid, mode)

    def get_node_coord_from_name(self, nodes: dict, node_name: str):
        if node_name in nodes:
            return [nodes[node_name]['x'], nodes[node_name]['y']]
        raise ValueError(f"Node {node_name} not in nodes")
    
    def get_node_name_from_coord(self, nodes: dict, node_coords):
        for name in nodes:
            if nodes[name]['x'] == node_coords[0] and nodes[name]['y'] == node_coords[1]: 
                return name
        raise ValueError(f"Node at {node_coords} not in nodes")

    def _determine_start_node(self, rid: str, occupied_nodes: set, nodes_dict: dict, preferred_node: str = None) -> tuple[str, str]:
        """
        Determines the best start node for the robot ``rid``, avoiding already occupied node.
        
        :param rid: Robot ID
        :type rid: str
        :param occupied_nodes: The set of nodes already occupied by the other nodes
        :type occupied_nodes: set
        :param nodes_dict: All node names and their coordinates
        :type nodes_dict: dict
        :param preferred_node: Override the nearest node preference
        :type preferred_node: str
        :return: The pair (target, start) for use in repositioning for this robot
        :rtype: tuple[str, str]
        """""
        robot_planner = self.robot_manager.get_planner(rid)
        node_idx = robot_planner._current_target_node_idx
        ref_path = robot_planner._ref_path
        
        # Get coordinates from planner path
        next_node_coord = ref_path[node_idx]
        prev_node_coord = ref_path[node_idx - 1]
        
        # Get names
        next_node_name = self.get_node_name_from_coord(nodes_dict, next_node_coord)
        prev_node_name = self.get_node_name_from_coord(nodes_dict, prev_node_coord)
        
        robot_state = self.robot_manager.get_robot_state(rid)
        
        # Calculate distances
        dist_to_next = np.linalg.norm(np.array(next_node_coord) - robot_state[:2])
        dist_to_prev = np.linalg.norm(np.array(prev_node_coord) - robot_state[:2])
        
        # Use preferred node is available, other take the nearest node
        if preferred_node == next_node_name:
            preferred = next_node_name
            other = prev_node_name
        elif preferred_node == prev_node_name:
            preferred = prev_node_name
            other = next_node_name
        elif dist_to_next > dist_to_prev:
            preferred = prev_node_name
            other = next_node_name
        else:
            preferred = next_node_name
            other = prev_node_name
            
        # Check occupancy
        if preferred in occupied_nodes:
            # Try the other one
            preferred, other = other, preferred
            
        if preferred in occupied_nodes:
            raise ValueError(f"Robot {rid} cannot find a start node. Both {preferred} and {other} are occupied.")
            
        return preferred, other

    def build_new_schedule(self, kt: int, path_to_new_task: str = "./new_task.json") -> tuple[str, dict, dict]:
        """
        Builds the new schedule by looking at all jobs and calculating what jobs remain to be done.
        Also calculates where the start postitions of the robots.
        
        :param kt: Current time step
        :type kt: int
        :param path_to_new_task: Custom path to new task, is also the first return argument
        :type path_to_new_task: str
        :return: Returns the path to the new task, positioning information for the start of the robots and
        all nodes
        :rtype: tuple[str, dict, dict]
        """
        if self.task is None:
            raise ValueError("No task file provided for rescheduling.")
        
        with open(self.task, 'r') as f:
            base_task = json.load(f)
        # Load all nodes and jobs from the base task.
        nodes = base_task["test_data"]["nodes"]
        jobs = base_task["jobs"]

        new_jobs = {}
        new_ATRs = {}
        occupited_start_node = set()
        rescheduling_info = {}

        # Detect swaps and force robots to advance
        robot_edges = {}
        for rid in self.robot_ids:
            robot_planner = self.robot_manager.get_planner(rid)
            node_idx = robot_planner._current_target_node_idx
            ref_path = robot_planner._ref_path
            next_node_name = self.get_node_name_from_coord(nodes, ref_path[node_idx])
            prev_node_name = self.get_node_name_from_coord(nodes, ref_path[node_idx - 1])
            robot_edges[rid] = (prev_node_name, next_node_name)

        forced_starts = {}
        for r1, r2 in combinations(self.robot_ids, 2):
            p1, n1 = robot_edges[r1]
            p2, n2 = robot_edges[r2]
            # Check for swap: R1 on (A, B), R2 on (B, A)
            if p1 == n2 and n1 == p2:
                # Force them to advance to their targets (n1 and n2)
                forced_starts[r1] = n1
                forced_starts[r2] = n2

        for rid in self.robot_ids:
            # Get all jobs per robot..
            robot_planner = self.robot_manager.get_planner(rid)
            node_idx = robot_planner._current_target_node_idx
            ref_path = robot_planner._ref_path
            jobs_for_rid = {
                job: copy.deepcopy(jobs[job])
                for job in jobs
                if jobs[job]["ATR"] == [rid]
            }

            # ... and remove jobs already completed
            planner = self.robot_manager.get_planner(rid)
            jobs_completed = planner.get_jobs_completed()
            if jobs_completed == 0:
                last_completed_job = None
            else:
                last_completed_job = planner.get_job_by_index(jobs_completed - 1)

            if last_completed_job is not None and last_completed_job in jobs_for_rid:
                job_sequence = list(jobs_for_rid.keys())
                cutoff = job_sequence.index(last_completed_job)
                for job_name in job_sequence[:cutoff + 1]:
                    jobs_for_rid.pop(job_name, None)

            if not jobs_for_rid:
                continue
            
            # Build the job dictionary in the format required by the scheduler
            remaining_names = set(jobs_for_rid.keys())
            ordered_jobs = list(jobs_for_rid.items())
            first_key, first_job = ordered_jobs[0]
            first_job["precedence"] = []
            for job_name, job_data in ordered_jobs[1:]:
                job_data["precedence"] = [
                    dep for dep in job_data.get("precedence", [])
                    if dep in remaining_names
                ]

            new_jobs.update(jobs_for_rid)

            # If the robot has a job, determine its start node to avoid conflicting with other robot
            if jobs_for_rid:
                preferred = forced_starts.get(rid)
                start_node, other_node = self._determine_start_node(rid, occupited_start_node, nodes, preferred_node=preferred)
                new_ATRs[rid] = start_node
                occupited_start_node.add(start_node)
                rescheduling_info[rid] = {'start': start_node, 'other': other_node}

        # Copy the strucutre from the base_task and update only what is necessary
        new_json = copy.deepcopy(base_task)
        new_json["jobs"] = new_jobs
        new_json["ATRs"] = new_ATRs

        with open(path_to_new_task, "w") as f:
            json.dump(new_json, f, indent=2)

        return path_to_new_task, rescheduling_info, nodes
    
    def create_schedule_df(self, solution: pd.DataFrame) -> dict:
        """
        Convert the schedule dictionary output from the Composlim scheduler to the Pandas Dataframe that the robot manager expects
        
        :param solution: The solution from Compo_slim.py
        :type solution: dict
        :return: The converted solution
        :rtype: pd.DataFrame
        """
        ROBOT_ID = "robot_id"
        NODE_ID = "node_id"
        ETA = "ETA"
        reformatted = {ROBOT_ID: [], NODE_ID: [], ETA: []}
        rows = []
        for rid in solution:
            for node_eta_pair in solution[rid]:
                node = node_eta_pair[0]
                eta = node_eta_pair[1]
                new_row = [rid, node, eta]
                rows.append(new_row)

        for row in rows:
            reformatted[ROBOT_ID].append(row[0])
            reformatted[NODE_ID].append(row[1])
            reformatted[ETA].append(row[2])

        df = pd.DataFrame(reformatted)
        return df

    def rotate_in_place(self):
        """
        Rotates all robots in place to resolve issues with the controller.
        This follows from the Duckiebots which may rotate in place, but
        this breaks the Unicycle defintion of the robots. Still, required
        to make progress for now
        """
        for rid in self.robot_ids:
            state = self.robot_manager.get_robot_state(rid)
            start_x = state[0]
            start_y = state[1]

            planner = self.robot_manager.get_planner(rid)
            target = planner._ref_path[1] # First goal the target will go to
            direction = np.atan2(target[1] - start_y, target[0] - start_x)
            new_state = np.array([start_x, start_y, direction])
            self.robot_manager.set_robot_state(rid, new_state)

    def force_move_to_start(self, rid: str):
        """
        Set the x/y position of robot ``rid`` to the start node.
        TODO Deprecate this function when a more robust solution can be created elsewhere 
        
        :param rid: Robot ID
        :type rid: str
        """
        path = self.robot_manager.get_planner(rid)._ref_path
        start_node = path[0]
        state = self.robot_manager.get_robot_state(rid)
        new_state = np.array([start_node[0], start_node[1], state[2]])
        self.robot_manager.set_robot_state(rid, new_state)