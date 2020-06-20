import rospy
import utils.node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

def create_arrow(frame_id, start, end, id):
    arrow = Marker()
    arrow.header.frame_id = frame_id
    arrow.header.stamp = rospy.Time.now()
    arrow.action = 0
    arrow.type = 0 
    arrow.ns = "path"
    arrow.id = id
    start_point = Point(start[0],start[1],start[2])
    end_point = Point(end[0],end[1],end[2])
    arrow.points = [start_point, end_point]
    arrow.scale.x = 0.05
    arrow.scale.y = 0.1
    arrow.scale.z = 0.1
    arrow.color.a = 1.0
    arrow.color.r = 0.0
    arrow.color.g = 0.0
    arrow.color.b = 1.0
    return arrow 

def create_sphere(frame_id, coord, id):
    sphere = Marker()
    sphere.header.frame_id = frame_id
    sphere.header.stamp = rospy.Time.now()
    sphere.action = 0
    sphere.type = 2
    sphere.ns = "way_point"
    sphere.id = id
    sphere.pose.position.x = coord[0]
    sphere.pose.position.y = coord[1]
    sphere.pose.position.z = coord[2]
    sphere.pose.orientation.x = 0.0
    sphere.pose.orientation.y = 0.0
    sphere.pose.orientation.z = 0.0
    sphere.pose.orientation.w = 0.0
    sphere.scale.x = 0.1
    sphere.scale.y = 0.1
    sphere.scale.z = 0.1
    sphere.color.a = 1.0
    sphere.color.r = 1.0
    sphere.color.g = 0.0
    sphere.color.b = 0.0
    return sphere 

def visualize_path(frame_id, path):
    vis_path = []
    origin = create_sphere(frame_id,path[0],0)
    wp = [origin]
    for wp_index in range(1,len(path)):
        new_path = create_arrow(frame_id,path[wp_index-1],path[wp_index],wp_index-1)
        new_wp = create_sphere(frame_id, path[wp_index],wp_index)
        vis_path.append(new_path)
        wp.append(new_wp)
    vis_path.append(create_arrow(frame_id,path[-1],path[0],len(path)))
    vis_path.extend(wp)
    return MarkerArray(vis_path)

def talker(path):
    markerPub = rospy.Publisher('rrt_path_vis', MarkerArray, queue_size=10)
    rospy.init_node('rrt_visualizer', anonymous=True)
    to_pub = visualize_path("map",path)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        markerPub.publish(to_pub)
        rate.sleep()

if __name__ == '__main__':
    pth = [(0,0,0),(3,0,0),(4,-4,0),(8,-3,0),(6,4,0),(-2,2,0),(-1,0,0)]
    try:
        talker(pth)
    except rospy.ROSInterruptException:
        pass