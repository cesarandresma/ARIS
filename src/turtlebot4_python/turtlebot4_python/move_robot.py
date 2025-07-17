#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class NumberCounter(Node): 
    def __init__(self):
        super().__init__('number_monitor')

        self.number = None

        self.number_suscriber = self.create_subscription(
            String,
            'chatter',
            self.number_callback,
            10
        )
    #Callbacks
    def number_callback(self, msg:String):
        self.number = int(msg.data)     


def main(args = None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()
    
    #Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    #Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    #wait for Nav2
    navigator.waitUntilNav2Active()

    #Undock
    navigator.undock()
        
    number_counter = NumberCounter()

    while rclpy.ok():
        rclpy.spin_once(number_counter)
        set_number_pose = number_counter.number

        goal_pose_A = navigator.getPoseStamped([-0.25, -0.22], TurtleBot4Directions.EAST)
        goal_pose_B = navigator.getPoseStamped([-3.59, -0.41], TurtleBot4Directions.EAST)
        goal_pose_C = navigator.getPoseStamped([6.35, -7.09], TurtleBot4Directions.EAST)
        goal_pose_D = navigator.getPoseStamped([8.72, 4.30], TurtleBot4Directions.EAST)
        goal_pose_H = navigator.getPoseStamped([0.59, -14.39], TurtleBot4Directions.WEST)

        goal_pose = []
        goal_pose.append(navigator.getPoseStamped([-0.59, -0.25], TurtleBot4Directions.EAST))
        goal_pose.append(navigator.getPoseStamped([-4.17, 0.27], TurtleBot4Directions.EAST))
        goal_pose.append(navigator.getPoseStamped([-3.91, 2.57], TurtleBot4Directions.EAST))
        goal_pose.append(navigator.getPoseStamped([-1.13, 2.07], TurtleBot4Directions.EAST))
        goal_pose.append(navigator.getPoseStamped([-0.49, 0.59], TurtleBot4Directions.EAST))

        if set_number_pose == 1:
            navigator.startToPose(goal_pose_A)
        elif set_number_pose == 2:
            navigator.startToPose(goal_pose_B)
        elif set_number_pose == 3:
            navigator.startToPose(goal_pose_C)
        elif set_number_pose == 4:
            navigator.startToPose(goal_pose_D)
        elif set_number_pose == 5:
            navigator.startThroughPoses(goal_pose)
        else:
            navigator.info('Welcome to the Aris service')

       
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    