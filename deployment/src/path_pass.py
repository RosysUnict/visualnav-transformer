import rospy
from nav_msgs.msg import Path
from mbf_msgs.msg import ExePathAction, ExePathGoal
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import actionlib

def path_callback(msg):
    global exe_path_client, marker_pub
    rospy.loginfo("Inside callback")
    # Creazione dell'obiettivo ExePathGoal con il percorso
    exe_path_goal = ExePathGoal()
    exe_path_goal.path = msg
    exe_path_goal.path.header.frame_id = "robot_map"

    # Invio dell'obiettivo al move_base_flex
    exe_path_client.send_goal(exe_path_goal)

    # Creazione dei marker per i punti del percorso
    marker_array = MarkerArray()
    for i, pose_stamped in enumerate(msg.poses):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose = pose_stamped.pose
        marker.id = i
        marker_array.markers.append(marker)

    # Pubblicazione dei marker su RViz
    marker_pub.publish(marker_array)

    # Attendo il completamento dell'obiettivo
    rospy.loginfo("waiting for result")
    exe_path_client.wait_for_result()

def main():
    global exe_path_client, marker_pub

    rospy.init_node('path_executor_node')

    # Inizializzazione del client per ExePathAction
    exe_path_client = actionlib.SimpleActionClient('/robot/move_base_flex/exe_path', ExePathAction)
    
    exe_path_client.wait_for_server()
    rospy.loginfo("sono qua!")

    # Pubblicatore per i marker
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    # Sottoscrizione al topic dei waypoint del percorso
    rospy.Subscriber('/vint_path', Path, path_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
