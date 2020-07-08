import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def create_arrow(frame_id, start, end, id, name_space):
    """ Create one arrow marker for rviz

    Args:
        frame_id (string): frame_id of the marker, required by ROS
        start (tuple): starting point of the arrow (x,y,z)
        end (tuple): ending point of the arrow (x,y,z)
        id (int): identification number of arrow. Uniquely assigned 
        name_space (string): namespace of the arrow.

    Returns:
        [visualization_msgs.msg.Marker]: an arrow marker to be visualized
    """
    arrow = Marker()
    arrow.header.frame_id = frame_id
    arrow.header.stamp = rospy.Time.now()
    # action number 0 is add/modify a marker
    arrow.action = 0
    # tpye number 0 is arrow markers
    arrow.type = 0
    arrow.ns = name_space
    arrow.id = id
    # Arrow marker requires Point object to specify staring and ending points
    start_point = Point(start[0], start[1], start[2])
    end_point = Point(end[0], end[1], end[2])
    arrow.points = [start_point, end_point]
    # NOTE: Currently hard-coded properties of arrow
    arrow.scale.x = 0.05
    arrow.scale.y = 0.1
    arrow.scale.z = 0.1
    arrow.color.a = 1.0
    arrow.color.r = 0.0
    arrow.color.g = 0.0
    arrow.color.b = 1.0
    return arrow


def create_sphere(frame_id, coord, id, name_space):
    """ Create one sphere marker for rviz

    Args:
        frame_id (string): frame_id of the marker, required by ROS
        coord (tuple): coordination of the center of sphere (x,y,z)
        id (int): identification number of sphere. Uniquely assigned 
        name_space (string): namespace of the sphere.

    Returns:
        [visualization_msgs.msg.Marker]: a shpere marker to be visualized
    """
    sphere = Marker()
    sphere.header.frame_id = frame_id
    sphere.header.stamp = rospy.Time.now()
    # action number 0 is add/modify a marker
    sphere.action = 0
    # type number 2 is sphere markers
    sphere.type = 2
    sphere.ns = name_space
    sphere.id = id
    sphere.pose.position.x = coord[0]
    sphere.pose.position.y = coord[1]
    sphere.pose.position.z = coord[2]
    sphere.pose.orientation.x = 0.0
    sphere.pose.orientation.y = 0.0
    sphere.pose.orientation.z = 0.0
    sphere.pose.orientation.w = 0.0
    # NOTE: Currently hard-coded sphere properties
    sphere.scale.x = 0.1
    sphere.scale.y = 0.1
    sphere.scale.z = 0.1
    sphere.color.a = 1.0
    sphere.color.r = 1.0
    sphere.color.g = 0.0
    sphere.color.b = 0.0
    return sphere


def markerize_path(frame_id, path):
    """ Visualize a path. The waypoints are represented by sphere markers, and the
        directed edges between waypoints are represented by arrows. Automatically 
        draws a edge from destination waypoint to initial waypoint to complete a
        circular path. 

    Args:
        frame_id (string): frame_id of the path, required by ROS
        path (tuple): waypoints that compose the path. In (x,y,z) position

    Returns:
        [visualization_msgs.msg.MarkerArray]: the array of marker that contains 
        both the waypoitns and the edges ready to be shown in rviz
    """
    vis_path = []
    # add the first waypoint for easier looping of both path and waypoints
    origin = create_sphere(frame_id, path[0], 0, "waypoint")
    wp = [origin]
    for wp_index in range(1, len(path)):
        # paths are drawn between every two contiguous waypoints
        new_path = create_arrow(
            frame_id, path[wp_index - 1], path[wp_index], wp_index - 1, "path"
        )
        new_wp = create_sphere(frame_id, path[wp_index], wp_index, "waypoint")
        vis_path.append(new_path)
        wp.append(new_wp)
    # vis_path.append(create_arrow(frame_id,path[-1],path[0],len(path),"path"))
    # combine both path and waypoint together for display
    vis_path.extend(wp)
    return MarkerArray(vis_path)


def markerize_points(frame_id, points):
    """ Visualize the imaginary wall that is built on the grid map. The walls are
        connected cells on the grid map, but the approximated coordinate points 
        are used for display the wall, so the wall may appear to be segmented. 
        This visual does not affect the functionality of the wall.

    Args:
        frame_id (string): frame_id of the walls, required by ROS
        wall (list): coordintes of points that compose of the wall. 

    Returns:
        [visualization_msgs.msg.MarkerArray]: the array of markers ready to be 
        displayed in rviz
    """
    point_markers = []
    for wp_index in range(0, len(points)):
        new_wall = create_sphere(frame_id, points[wp_index], wp_index, "wall")
        point_markers.append(new_wall)
    return MarkerArray(point_markers)
