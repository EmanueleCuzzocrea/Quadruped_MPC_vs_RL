# Quadruped Control
In this project, a model-based control and a data-driven control have been implemented to control a quadruped robot.
The performances were compared, and the advantages and disadvantages of both approaches were analyzed.

## Implemented controllers

```
📦Quadruped_Control
 ┣ 📂ROS               # MPC controller [1]
 ┣ 📂RaiSim            # RL controller  [2]

 [1] https://github.com/TopHillRobotics/quadruped-robot/tree/main
 [2] https://github.com/raisimTech/raisimLib
```

## MPC controller
Using the MPC controller, the following gaits were implemented:
- Trot
- Bound
- Pace
- Crawl
- Gallop


### Trot gait

https://github.com/user-attachments/assets/607a0042-e4cb-4104-a0cc-bb54e6603f6d

### Bound gait

https://github.com/user-attachments/assets/50fa4426-905c-4d2f-97b7-c88f8edc22b1

### Pace gait

https://github.com/user-attachments/assets/02fcb74e-cac6-4244-8bf4-154ab327787c

## Path planning
Utilizing this controller, an RRT was implemented and tested as a global planner.

https://github.com/user-attachments/assets/72e6b320-07b7-4a51-a96c-78bf76f5c43b

https://github.com/user-attachments/assets/85753b0a-225a-4a8b-a0a0-3d644bfe11f7

## RL controller
All training sessions were carried out using the [RaiSim](https://raisim.com/) platform.
Moreover, training the robot to perform a specific gait was made possible through the use of Reward Machines.

The following gaits were implemented:
- Trot
- Bound
- Pace

### Trot gait

https://github.com/user-attachments/assets/37726eb9-76b9-401a-bb9b-e74bf42f53a4

### Bound gait

https://github.com/user-attachments/assets/a64bd6e6-7c4e-4fb7-b994-fb73a63f55e9

### Pace gait

https://github.com/user-attachments/assets/8a93717c-bd84-4050-bb7f-6f4fca2d3318

### Multiple gaits
By using a Python listener, it was possible to run simulations while changing the gait at runtime (by switching the neural network).

https://github.com/user-attachments/assets/b849906d-f83c-4647-829f-3c34d3770de7


Thank you
