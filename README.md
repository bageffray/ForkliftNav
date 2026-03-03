#  Forklift Robot Navigation & Manipulation Project

This project was completed by two students as part of the ASPIC Master 2 course "Robotics, Mechatronics, Infotronics".

## Authors
- Iwen Jomaa
- Baptiste Geffray

# Project Overview

The goal of this project is to implement a **ground-based robotic system** - represented as a **forklift** - equipped with an **articulated robotic arm** composed of **two joints and magnetic hand**.

The robot must autonomously perform the following mission:

1. Navigate to the coordinates of a target object  
2. Pick up the object using the robotic arm  
3. Return to its initial position  
4. Release the object at the starting location  

This project focuses on **robot navigation**, **path planning**, and **manipulation control** within a simulated environment.

##  Robot Navigation

To ensure smooth and realistic motion of the forklift:

- **Spline-based trajectories** are used to generate smooth and continuous paths
- A **Rapidly-exploring Random Tree (RRT)** algorithm is implemented to generate feasible waypoint paths in constrained environments
- RRT-generated paths can be smoothed using splines for improved motion quality

These approaches allow the robot to navigate complex terrains while avoiding obstacles.

##  Robotic Arm Control

The robotic arm movement is controlled using an **Inverse Kinematics (IK) geometric model**, allowing the arm to:

- Accurately position the gripper in 3D space
- Perform object grasping and releasing actions
- Maintain realistic joint constraints

## Test Scenes

To validate and demonstrate the different subsystems of the robot, several test scenes are provided in the `Scenes` directory:

- **GrabRelease**  
  Tests the grabbing and releasing mechanism of the robotic arm.

- **Spline**  
  Demonstrates spline-based trajectory generation and visualization.

- **SmallSmoothRRT**  
  Tests the RRT path planning algorithm and its smoothing process.

- **ForkliftMoon**  
  A more "realistic" environment simulating uneven terrain.

## How to Launch

### Requirements
- **Unity version:** 2022.3.11f1 (LTS) 
with **Splines** packages : 2.4.0

### Opening the Project
1. Open **Unity Hub**
2. Add the project if it is not already listed
3. Open the project using **Unity 2022.3.11f1**

### Selecting a Scene
- To choose a scenario, simply open the desired scene using **Open Scene** in Unity.
- The available test scenes can be found in the `Scenes` directory.

### Running the Mission
1. Select the **Forklift** object in the scene hierarchy
2. In the **Inspector**, define the target position of the object to be picked up
3. Enable the **RunMission** option in the Forklift inspector to start the mission

### Adjustable Parameters 
Below is a description of the main parameters of the Forklift and Mission, editable directly in the Unity Inspector of the component LoaderArm
All follow the format below:
| Parameter | Type | Description |
#### Mission Handler 
| Parameter | Type | Description |
|-----------|------|-------------|
| `runMission` | bool | Start the autonomous grab-and-drop mission when checked. |
| `Target Positions` | Vector3 | World-space (x,y,z) coordinates of the target object the forklift must reach and grab. |
| `Waypoint` | List<Vector3> | Optional intermediate points for the forklift to pass through during mission. |
| `Distance To Keep` | float | Minimum stopping distance from the target position. |
| `Arm Controller` | ArmController | Reference to the arm controller component. |
| `Car Controller` | CarController | Reference to the vehicle controller component. |

### Car Controller
| Parameter | Type | Description |
|-----------|------|-------------|
| `Avoidance Zones` | List<AvoidanceZone> | Zones to avoid during path planning, (x,y,z) coordinates and size can be changed. |
#### Algorithm Setting
| Parameter | Type | Description |
|-----------|------|-------------|
| `Speed Algorithm` | Enum( Constant, AccelerateStop, Trapezoidal) | Algorithm for vehicle speed profile. |
| `Path Algorithm` | Enum(Spline, RRT, RRTSmoothed, RRTSimplified, RRTSimplifiedSmoothed) | Path generation algorithm. |
#### View Settings
| Parameter | Type | Description |
|-----------|------|-------------|
| `Debug View` | bool | Enables or not debug visualization of splines and explored points (The spline is in yellow, the tangent in green, the projected spline in blue, the avoidance in red, the bypass points in cyan, the node explored in magenta, the simplified path in cyan). |
#### Validation Settings
| Parameter | Type | Description |
|-----------|------|-------------|
| `Max Slope Angle` | float | Maximum slope angle allowed for path validation (degrees). |
| `Min Slope Angle` | float | Minimum slope angle allowed for path validation (degrees). |
| `Width Margin Fraction` | float | Safety margin added to vehicle width for clearance checks. |
#### Smoothing settings
| Parameter | Type | Description |
|-----------|------|-------------|
| `Tangent Smooth Factor` | float | Factor used when smoothing spline tangents, typical range: 0.5-1.0, the higher the smoother the spline. |
| `Limit Maximum Tangent Length` | bool | Whether to clamp the maximum tangent length to avoid overly long tangents. |
| `Maximum Tangent Length` | float | Maximum allowed tangent length if clamping is enabled. |
#### RRT Settings
| Parameter | Type | Description |
|-----------|------|-------------|
| `Rrt Step Size` | float | Step size for RRT path expansion, the higher = fast exploration ,lower = more precise result. |
| `Rrt Max Iterations` | float | Maximum number of iterations of RRT, with higher value RRT has more chance to find a path but will take longer to resolve, with lower value RRT can stop quicker if no path is found. |
#### Repath Setting 
| Parameter | Type | Description |
|-----------|------|-------------|
| `Check Validity` | bool | Enable path validation for collisions and slope constraints. |
| `Max Repath Attempts` | float | Maximum number of attempts to generate a valid bypass around an obstacle.Higher = more attempts but slower, lower = faster but less reliable. |
| `Repath Offset Distance` | float | Lateral offset distance used when generating bypass. |
| `Repath Forward Distance` | float | Forward offset distance used when generating bypass. |
| `Discretization Steps Per Meter` | float | Number of samples per meter along the spline for collision checking and 3D projection. Higher = more precise path validation, but slower. |
| `Check Step Interval` | float | Step interval (in world units) used when checking spline validity along the path. |
| `Path Verification Step` | float | Step size used for verifying each segment of the path during RRT or spline validation. Smaller = more precise, but slower. |

Once launched, the forklift will:
- Navigate to the target object
- Grab it using the articulated arm
- Return to its starting position
- Release the object


## Project Structure
```bash
├───Assets                                                                                                              
│   ├───AshKodeLoaders                                                                                                  
│   │   ├───Demonstration                                                                                               
│   │   ├───Materials                                                                                                   
│   │   ├───Models                                                                                                      
│   │   ├───Prefabs                                                                                                     
│   │   └───Texture                                                                                                     
│   ├───Lunar Landscape 3D                                                                                              
│   │   ├───Prefabs                                                                                                     
│   │   ├───Resources                                                                                                   
│   │   │   ├───Geometry                                                                                                
│   │   │   ├───Materials                                                                                               
│   │   │   └───Textures                                                                                                
│   │   └───Scenes                                                                                                      
│   │       └───LunarLandscape3D_Profiles                                                                               
│   ├───path                                                                                                            
│   ├───Scenes                                                                                                          
│   │   └───DemoScenes                                                                                                  
│   ├───Settings                                                                                                        
│   └───utils                                                                                                           
├───Library  
├───Logs
├───obj
├───Packages
├───ProjectSettings
├───Temp
├───UserSettings
``` 
##  Technologies & Concepts

- Unity Engine  
- Path Planning (RRT)  
- Spline-based Trajectories  
- Inverse Kinematics (Geometric Model)  
- Autonomous Navigation  
- Robotic Manipulation  