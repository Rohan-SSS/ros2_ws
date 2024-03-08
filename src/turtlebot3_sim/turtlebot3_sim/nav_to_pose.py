from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Setting the initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -1.97
    initial_pose.pose.position.y = -0.48
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)


    navigator.waitUntilNav2Active()

    # Initializing goal poses array
    goal_poses = []

    # Setting goals
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.53 #1.72 
    goal_pose1.pose.position.y = -0.51 #-0.49
    goal_pose1.pose.orientation.w = 0.70 #0.70
    goal_pose1.pose.orientation.z = 0.70 #0.70
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -0.53 #1.77
    goal_pose2.pose.position.y = 1.84 #0.62
    goal_pose2.pose.orientation.w = 0.0 #0.01
    goal_pose2.pose.orientation.z = 1.0 #-1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.61 #-1.93
    goal_pose3.pose.position.y = 1.84 #0.53
    goal_pose3.pose.orientation.w = 1.0 #0.70
    goal_pose3.pose.orientation.z = 0.0 #-0.70
    goal_poses.append(goal_pose3)

    # Sending msg to navigate through goals
    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Navigation timeout cancels the navigation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                navigator.cancelTask()

            # Navigate to a fixed pose if destination pose hasn't been reached in 30 sec
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose4.pose.position.x = -1.97
                goal_pose4.pose.position.y = -0.48
                goal_pose4.pose.orientation.z = 0.0
                goal_pose4.pose.orientation.w = 1.0
                navigator.goThroughPoses([goal_pose4])

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)
