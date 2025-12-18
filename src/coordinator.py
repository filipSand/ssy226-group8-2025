from configs import CircularRobotSpecification
from pkg_robot.robot import RobotManager
import numpy as np
import pandas as pd
import json
import copy

from pkg_sche.sp_comsat.Compo_slim import Compo_slim
from pkg_motion_plan import GlobalPathCoordinator


class Coordinator:
    """
    Coordinator class - TODO document
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
        Placeholder TODO

        TODO Check if my delay/earlyness impacts other robots. If they don't its probably fine to be early
        Args:
            kt: The current time step
            threshold: The greatest delay of a single robot that can be tolerated before rescheduling is started
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
                # If the robot has reached the finish line, set zero delay
                raise ValueError("This shouldn't be reached")
                delays.append(0)
                continue

            scheduled_node = ref_path[schedule_idx]
            prev_sched_node = ref_path[schedule_idx - 1] 
            scheduled_departure_time = ref_path_times[schedule_idx-1]
            scheduled_arrival_time = ref_path_times[schedule_idx]

            if target_is_final:
                # Check if we're close to the target, if we are, we are finished and there is no delay
                target = np.array([*target_node])
                if np.linalg.norm((robot_state[:2] - target), 2) < 0.5:
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

    def get_target_node_index(self, ref_path, target_node):
        robot_target_i = None
        for i, node in enumerate(ref_path):
            if node == target_node:
                robot_target_i = i
                break
        return robot_target_i

    def calcuate_edge_delay(self, current_time, robot_state, scheduled_node, prev_sched_node, departure_time, arrival_time, max_vel = None):
        """
        Estimate time delay between robot and scheduled position via linear interpolation.
        TODO Update/evaluate the way to calculate delay time
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
        1. Go to next node, if not possible, go back DONE
        2. Figure out what jobs remain and rebuild the problem json file using these jobs DONE
        3. Figure out what edges are blocked TODO
        4. Call the scheduler DONE
        5. Implement the new schedule TODO
        6. Resume running TODO 
        """
        #self._set_all_idle()
        time = kt * self.ts

        self.new_schedule_path = self.build_new_schedule(kt)
        self.set_all_idle_mode(True)

        # Stop all robots
        for rid in self.robot_ids:
            robot_planner = self.robot_manager.get_planner(rid)
            target_node = robot_planner.current_target_node
            ref_path = robot_planner._ref_path
            target_node_index = self.get_target_node_index(ref_path, target_node)
            prev_node = ref_path[target_node_index-1]
            ref_path_times = robot_planner._ref_path_time
            prev_time = ref_path_times[target_node_index-1]
            robot_state = self.robot_manager.get_robot_state(rid)

            distance_to_target = np.sqrt(target_node[0]**2 + target_node[1]**2)
            distance_to_prev = np.sqrt(prev_node[0]**2 + prev_node[1]**2)
            if distance_to_target > distance_to_prev:
                # If the target is nearer, return to previous node instead
                target_node, prev_node = prev_node, target_node
            self.occupied_for_reset.append(target_node)
            self.robot_manager.add_schedule(rid, robot_state, [prev_node, target_node], [prev_time, time])

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

    def build_new_schedule(self, kt: int, path_to_new_task: str = "./new_task.json") -> str:
        if self.task is None:
            raise ValueError("No task file provided for rescheduling.")
        
        with open(self.task, 'r') as f:
            base_task = json.load(f)
        nodes = base_task["test_data"]["nodes"]
        jobs = base_task["jobs"]

        new_jobs = {}
        new_ATRs = {}
        occupited_start_node = set()

        for rid in self.robot_ids:
            robot_planner = self.robot_manager.get_planner(rid)
            node_idx = robot_planner._current_target_node_idx
            ref_path = robot_planner._ref_path
            jobs_for_rid = {
                job: copy.deepcopy(jobs[job])
                for job in jobs
                if jobs[job]["ATR"] == [rid]
            }

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

            if jobs_for_rid:
                next_node = ref_path[node_idx]
                prev_node = ref_path[node_idx - 1]
                next_node_name = self.get_node_name_from_coord(nodes, ref_path[node_idx])
                prev_node_name = self.get_node_name_from_coord(nodes, ref_path[node_idx - 1])
                
                distance_to_target = np.sqrt(next_node[0]**2 + next_node[1]**2)
                distance_to_prev = np.sqrt(prev_node[0]**2 + prev_node[1]**2)

                if distance_to_target > distance_to_prev:
                    # If the target is occupied, return to previous node instead
                    new_ATRs[rid] = prev_node_name
                else:
                    new_ATRs[rid] = next_node_name


        new_json = copy.deepcopy(base_task)
        new_json["jobs"] = new_jobs
        new_json["ATRs"] = new_ATRs

        with open(path_to_new_task, "w") as f:
            json.dump(new_json, f, indent=2)

        return path_to_new_task
    
    def create_schedule_df(self, solution: dict) -> dict:
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
        
        :param self: Coordinator
        """
        for rid in self.robot_ids:
            state = self.robot_manager.get_robot_state(rid)
            start_x = state[0]
            start_y = state[1]

            planner = self.robot_manager.get_planner(rid)
            target = planner._ref_path[1] # Target node, i.e. first node, will be start, which is not helpful
            direction = np.atan2(target[1] - start_y, target[0] - start_x)
            new_state = np.array([start_x, start_y, direction])
            self.robot_manager.set_robot_state(rid, new_state)

    def rotate_90(self):
        for rid in self.robot_ids:
            state = self.robot_manager.get_robot_state(rid)
            new_angle = (state[2] + np.pi/2) % 2*np.pi
            new_state = [*state[0:2], new_angle]
            self.robot_manager.set_robot_state(rid, new_state)


