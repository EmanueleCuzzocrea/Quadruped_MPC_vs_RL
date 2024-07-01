#! /usr/bin/env python3
import rospy
from random import randrange as rand
from math import atan2, cos, sin

# Node class
class Node:
  def __init__(self, coordinates, parent=None):
    # coordinates: list with [x,y] values of grid cell coordinates
    self.coordinates = coordinates
    # parent: Node object
    self.parent = parent

def calculate_distance(p1, p2):
  dx = p2[0] - p1[0]
  dy = p2[1] - p1[1]
  distance = (dx ** 2 + dy ** 2)**0.5
  return distance

def calculate_angle(p1, p2):
  dx = p2[0] - p1[0]
  dy = p2[1] - p1[1]
  theta = atan2(dy, dx)
  return theta

def find_closest_node(random_pt, node_list):
    nearest_distance = float('inf')
    for n in node_list:
      current_distance = calculate_distance(random_pt, n.coordinates)
      if current_distance < nearest_distance:
        nearest_distance = current_distance
        nearest_node = n
    return nearest_node

def create_new_branch_point(p1, p2, max_distance):
    new_point = list(p1)
    d = calculate_distance(new_point, p2)
    theta = calculate_angle(new_point, p2)

    new_point[0] += int(max_distance * cos(theta))
    new_point[1] += int(max_distance * sin(theta))

    return new_point

def collision_detected(p1, p2, map, map_width):
    # Calcola la distanza tra p1 e p2
    step_size = 0.5
    dist = calculate_distance(p1, p2)
    num_steps = int(dist / step_size)
    
    # Genera i punti tra p1 e p2 con il passo specificato
    covered_cells = []
    for step in range(num_steps + 1):
        t = step / num_steps
        x = int(p1[0] * (1 - t) + p2[0] * t)
        y = int(p1[1] * (1 - t) + p2[1] * t)
        covered_cells.append((x, y))
    
    for cell in covered_cells:
        # Accede a un elemento in un array 1D (map) fornendo l'indice = x + map_width * y
        if map[cell[0] + map_width * cell[1]] > 0.1:
            return True
    return False




def rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, tree_viz):
    # Data structures initialization
    root_node = Node(initial_position)
    nodes = [root_node]
    iteration = 0
    max_iterations = 50
    max_loop = 100
    delta = 10
    path = []
    test = 0

    while iteration < max_loop:
      
      for i in range(max_iterations):

        # q_rand generation
        random_point = [rand(width), rand(height)]

        # Find q_near
        closest_node = find_closest_node(random_point, nodes)

        # q_new computation
        candidate_point = create_new_branch_point(closest_node.coordinates, random_point, delta)

        # Collision-check algorithm
        if not collision_detected(closest_node.coordinates, candidate_point, map, width):
          latest_node = Node(candidate_point, closest_node)
          nodes.append(latest_node)
          
          # Optional: Visualize tree graph in Rviz
          tree_viz.append(latest_node)

      
      # Check if the goal has been reached
      closest_node_t = find_closest_node(target_position, nodes)
      if not collision_detected(closest_node_t.coordinates, target_position, map, width):
          target = Node(target_position, closest_node_t)
          nodes.append(target)
          # Optional: Visualize tree graph in Rviz
          tree_viz.append(target)
          rospy.loginfo('RRT: Goal reached')
          iteration = max_loop
          test = 1
      else:
         iteration += 1
    
    if test == 0:
      rospy.logwarn("RRT: Max loop iterations exceeded")
      return path

    
    
    
    rospy.loginfo('RRT: Path search ended')

    # Path reconstruction
    path.append(target_position)
    node = closest_node_t
    while node.parent:
        path.append(node.coordinates)
        node = node.parent
    path.append(node.coordinates)
    # Reverse list
    path.reverse()
    rospy.loginfo('RRT: Done reconstructing path')

    return path
