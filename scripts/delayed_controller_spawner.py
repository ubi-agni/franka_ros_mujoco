#!/usr/bin/env python

#  Copyright 2017 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

"""
Launches the controller spawner node after initial joint positions of the robot are initialized. The command line args are passed to the spawner node.
"""
import rospy
import roslaunch
import sys
from sensor_msgs.msg import JointState

def main():
    args = ' '.join(sys.argv[1:])
    package = 'controller_manager'
    node_type = 'spawner'
    rospy.init_node('delayed_controller_spawner')
    rospy.loginfo('Delay spawning of controller. Going to sleep...')
    while True:
        try:
            joint_state = rospy.wait_for_message('joint_states', JointState) # type: JointState
            # check that the 4. joint is not in an invalid range anymore
            if joint_state.position[3] < -0.05:
                break
            rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            rospy.logerr("ROSInterruptException")
            sys.exit(-1)

    rospy.loginfo('Waking up. Spawning controller...')
    node = roslaunch.core.Node(package, node_type, name=node_type, args=args)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    launch.launch(node)
    rospy.loginfo('Controller spawned')
    try:
        launch.spin()
    finally:
        launch.stop()

if __name__ == '__main__':
    main()
