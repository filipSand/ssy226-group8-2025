#!/usr/bin/env python3

import os
import rospy
import math
import random
import json
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import String
from dt_communication_utils import DTCommunicationGroup

vehicle_name = os.environ['VEHICLE_NAME']

# --- COMMUNICATION GROUP SETUP ---

if vehicle_name == "duck1":        
    group = DTCommunicationGroup('duck1_group', String)
elif vehicle_name == "duck3":
    group = DTCommunicationGroup('duck3_group', String)
elif vehicle_name == "duck4":
    group = DTCommunicationGroup('duck4_group', String)
else:
    rospy.loginfo_once(f"{vehicle_name} not found in database!")

# Twist command parameters 
VELOCITY = 0.3  
OMEGA    = 0  

class TwistControlNode(DTROS):

    def __init__(self, node_name):
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self._v      = VELOCITY
        self._omega = OMEGA
        self.position = None 

        self.goal_pose = None
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._ticks_left = None
        self._ticks_right = None
        
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.subscriber = group.Subscriber(self.callback)
        self.publisher = group.Publisher()
        self.fleet_pose = {}
        self.theta_error_integral = 0.0
        self.theta_bias = 0.0
        self.ki = 0.5
        

    def callback_left(self, data):
        self._ticks_left = data.data

    def callback_right(self, data):
        self._ticks_right = data.data
        
    def callback(self, data, header):
        try:
            message_dict = json.loads(data.data)
            sender_id = message_dict.get("sender")
            
            if sender_id != self._vehicle_name and self.position is None:
                self.position = message_dict.get("init_pose")
                self.goal_pose = message_dict.get("goal_pose")

            if sender_id != self._vehicle_name:
                fleet = message_dict.get("fleet_pose")
                if isinstance(fleet, dict):
                    self.fleet_pose = fleet

        except ValueError:
            pass

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

    def avoid_control(self):
        # --- INIT ---

        # Determine Opponent
        if self._vehicle_name == "duck1":
            opp = "duck3"
        elif self._vehicle_name == "duck3":
            opp = "duck1"
        elif self._vehicle_name == "duck4":
            opp = "duck3"
        else:
            rospy.logwarn("Unknown robot vehicle_name!")
            return

        # Physics Constants
        rate = rospy.Rate(20)
        axis_length = 0.105
        radius = 0.035
        wheel_circ = radius * 2 * math.pi
        Ntot = 135
        dt = 1.0 / 20.0  # Control step (change this if rate is changed)

        switch_var = True

        # Wait for init_pose
        while self.position is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(2, "Waiting for init_pose...")
            rate.sleep()

        position = self.position

        rospy.loginfo(f"Starting Avoidance PI control against {opp}")

        while not rospy.is_shutdown():
            if opp in self.fleet_pose and self.fleet_pose[opp] is not None:
                opp_pose = self.fleet_pose[opp]

                # --- CALCULATE VECTORS ---
                dx_opp = opp_pose[0] - position[0]
                dy_opp = opp_pose[1] - position[1]
                dist_opp = math.sqrt(dx_opp**2 + dy_opp**2)

                dx_g = self.goal_pose[0] - position[0]
                dy_g = self.goal_pose[1] - position[1]
                dist_goal = math.sqrt(dx_g**2 + dy_g**2)

                avoid_radius = 0.6

                # --- NEW VECTOR-BASED CONTROL LOGIC ---
                
                if dist_goal < 0.05:
                    # Arrived at goal
                    self._v = 0.0
                    self._omega = 0.0
                    desired_theta = position[2] 
                    self.theta_error_integral = 0.0
                else:
                    # Goal Vector (Unit vector pointing to goal)
                    v_goal_x = dx_g / dist_goal
                    v_goal_y = dy_g / dist_goal

                    # Avoidance Vector (Direction AWAY from opponent)
                    if dist_opp < avoid_radius:
                        # Unit vector pointing away from opponent
                        v_avoid_x = -dx_opp / dist_opp
                        v_avoid_y = -dy_opp / dist_opp
                        
                        # Weight increases quadratically as we get closer
                        weight = ((avoid_radius - dist_opp) / avoid_radius) ** 2
                    else:
                        v_avoid_x, v_avoid_y = 0, 0
                        weight = 0

                    # Resultant Vector (The blended path)
                    final_x = v_goal_x + (v_avoid_x * weight * 2.5)
                    final_y = v_goal_y + (v_avoid_y * weight * 2.5)

                    desired_theta = math.atan2(final_y, final_x)
                    
                    # Set Velocity 
                    base_speed = 0.5
                    self._v = base_speed * (1.0 - (weight * 0.5))

                # --- PI-CONTROLLER FOR STEERING ---
                if self._v != 0:
                    theta_error = desired_theta - position[2]
                    while theta_error > math.pi: theta_error -= 2 * math.pi
                    while theta_error < -math.pi: theta_error += 2 * math.pi

                    # Update integral
                    self.theta_error_integral += theta_error * dt

                    # PI control
                    self._omega = 7.0 * theta_error + 0.3 * self.theta_error_integral
                    self._omega = max(min(self._omega, 4.0), -4.0)
                else:
                    self._omega = 0.0

                # Publish command
                msg_cmd = Twist2DStamped(v=self._v, omega=self._omega)
                self._publisher.publish(msg_cmd)

                # --- ODOMETRY UPDATE ---
                if self._ticks_left is not None and self._ticks_right is not None:
                    left_motor_tick = self._ticks_left
                    right_motor_tick = self._ticks_right

                    if switch_var:
                        prev_left_motor_tick = left_motor_tick
                        prev_right_motor_tick = right_motor_tick
                        switch_var = False
                    else:
                        dNr = right_motor_tick - prev_right_motor_tick
                        dNl = left_motor_tick - prev_left_motor_tick

                        dr = wheel_circ * (dNr / Ntot)
                        dl = wheel_circ * (dNl / Ntot)

                        d = (dr + dl) / 2
                        dtheta = (dr - dl) / axis_length

                        midpoint_theta = position[2] + dtheta / 2.0
                        new_x = position[0] + d * math.cos(midpoint_theta)
                        new_y = position[1] + d * math.sin(midpoint_theta)
                        final_theta = position[2] + dtheta

                        position = (new_x, new_y, final_theta)
                        self.position = position

                        prev_left_motor_tick = left_motor_tick
                        prev_right_motor_tick = right_motor_tick

                # --- COMMUNICATION ---
                payload = {
                    "sender": self._vehicle_name,
                    "name": self._vehicle_name,
                    "pose": position
                }
                try:
                    message_fleet = String(data=json.dumps(payload))
                    self.publisher.publish(message_fleet)
                except Exception as e:
                    rospy.logerr(f"JSON dump failed: {e}")

                rate.sleep()

 

if __name__ == '__main__':
    node = TwistControlNode(node_name='twist_control_node')
    node.avoid_control()
    rospy.spin()