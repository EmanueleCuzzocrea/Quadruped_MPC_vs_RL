#!/usr/bin/env python3
import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from treeviz import TreeViz
from RRT_code import rrt

def make_plan(req):
  # Convert costmap from a 1-D tuple flat map representation
  map = list(req.costmap_ros)
  # Change values on the map from unknown to free space
  map[map==255] = 1
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # side of each grid map square in meters
  map_resolution = 0.05
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  map_origin = [-40.0, -40.0]

  initial_position = indexToGridCell(start_index, width)
  target_position = indexToGridCell(goal_index, width)
  viz = TreeViz(map_resolution, map_origin, id=1, frame="map")

  # time statistics
  start_time = rospy.Time.now()

  # Calculate a path using RRT
  path = rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, viz)

  if not path:
    rospy.logwarn("No path returned by RRT")
    return_path = []
  else:
    # print time statistics
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('+++++++++++ RRT execution metrics ++++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('RRT: Path sent to navigation stack')

    return_path = []
    for cell in path:
        return_path.append(cell[0]+width*cell[1])

  resp = PathPlanningPluginResponse()
  resp.plan = return_path
  
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

def indexToGridCell(flat_map_index, map_width):
  grid_cell_map_x = flat_map_index % map_width
  grid_cell_map_y = flat_map_index // map_width
  return [grid_cell_map_x, grid_cell_map_y]






if __name__ == '__main__':
  rospy.init_node('rrt_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/velocityParam', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)
    
  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)