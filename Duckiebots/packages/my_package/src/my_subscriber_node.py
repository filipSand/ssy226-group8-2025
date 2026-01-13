#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from dt_communication_utils import DTCommunicationGroup
import os
import json
import math

duck1_group = DTCommunicationGroup('duck1_group', String)
duck3_group = DTCommunicationGroup('duck3_group', String)
duck4_group = DTCommunicationGroup('duck4_group', String)
bias_y = 0.3 #compensate for drift (cumulative bias)
bias_x = 0.35
FLEET = ["duck1", "duck3", "duck4"]
INIT_POSE = {
    "duck1": [0,0,0],
    "duck3": [3,0,math.pi],
    "duck4": [3,0,math.pi]
}
GOAL_POSE = {
    "duck1": [3+bias_x,bias_y,math.pi],
    "duck3": [-bias_x,bias_y,0],
    "duck4": [0,0,math.pi]
}

class WatchtowerNode(DTROS):

    def __init__(self, node_name):
        super(WatchtowerNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Subscribers
        self.sub1 = duck1_group.Subscriber(self.callback)
        self.sub2 = duck3_group.Subscriber(self.callback)
        self.sub4 = duck4_group.Subscriber(self.callback)

        # Publishers
        self.pub1 = duck1_group.Publisher()
        self.pub2 = duck3_group.Publisher()
        self.pub4 = duck4_group.Publisher()

        # Store live fleet pose
        self.fleet_pose = dict(INIT_POSE)

    def callback(self, data, header):
        """
        Receives pose updates from robots
        """
        msg = json.loads(data.data)
        sender = msg.get("sender")
        pose = msg.get("pose")

        if sender in FLEET and pose is not None:
            # Removed the loginfo here to prevent console flooding
            self.fleet_pose[sender] = pose   

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        sent_init = False
        tick_counter = 0 

        while not rospy.is_shutdown():

            # --- BROADCAST INFO TO ROBOTS ---
            for bot, pub in [("duck1", self.pub1), ("duck3", self.pub2), ("duck4", self.pub4)]:
                # Running at 10Hz
                payload = {
                    "sender": self._vehicle_name,
                    "fleet_pose": self.fleet_pose
                }
                if not sent_init:
                    payload["init_pose"] = INIT_POSE[bot]
                    payload["goal_pose"] = GOAL_POSE[bot]
                pub.publish(String(data=json.dumps(payload)))

            sent_init = True

            # --- LOGGING ---
            # Running at 1Hz for readability
            if tick_counter % 10 == 0:
                self.log_status()
            
            tick_counter += 1
            rate.sleep()

    def log_status(self):
        """Helper to print a pretty table of the fleet status"""
        output = "\n" + "="*40 + "\n"
        output += f"{'ROBOT':<10} | {'X':<6} | {'Y':<6} | {'THETA':<6}\n"
        output += "-"*40 + "\n"
        
        for bot in FLEET:
            if bot in self.fleet_pose:
                x, y, theta = self.fleet_pose[bot]
                # Format to 2 decimal places for readability
                output += f"{bot:<10} | {x:<6.2f} | {y:<6.2f} | {theta%2*math.pi:<6.2f}\n"
            else:
                output += f"{bot:<10} | {'WAITING...':<20}\n"
        
        output += "="*40
        rospy.loginfo(output)

if __name__ == "__main__":
    node = WatchtowerNode("watchtower_node")
    node.run()
    rospy.spin()