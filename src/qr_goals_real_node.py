#!/usr/bin/env python

# ROS
import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

# Create goal points
def create_goal_points():
    goalA = MoveBaseGoal()
    goalA.target_pose.header.frame_id = "map"
    goalA.target_pose.pose.position.x = 0.25
    goalA.target_pose.pose.position.y = 2.80
    goalA.target_pose.pose.orientation.w = -1.00

    goalB = MoveBaseGoal()
    goalB.target_pose.header.frame_id = "map"
    goalB.target_pose.pose.position.x = 1.85
    goalB.target_pose.pose.position.y = 0.40
    goalB.target_pose.pose.orientation.w = 1.00

    return goalA, goalB

def move_base_client(goal):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal.target_pose.header.stamp = rospy.Time.now()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()   

def callback(data):
    if data.data == "GoToA":
        print("GO A")
        result = move_base_client(goalA)
    elif data.data == "GoToB":
        print("GO B")
        result = move_base_client(goalB)
    else:
        print("INVALID QRCODE")

goalA, goalB = create_goal_points()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('move_base_client_node', anonymous=True)
        rospy.Subscriber("/barcode", String, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
