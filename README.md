# Robot Control System for ARIAC Environment  

A detailed writeup is available [here](https://www.tudordonca.com/ariac).  
<br/>

## World Representation  
![World Representation Design](https://github.com/tdonca/ariac-2018-controls/blob/master/images/WorldStateDiagram.png)  

#### World State
* Keeps track of robot, parts, and world environment
* Maintains relationships between parts, containers, and robot
* Provides a client interface for the planners and controllers to modify environment relationships
  
#### Load Scene
* Updates the environment collision objects
* Used by controllers to plan collision-free motions
<br/>  

## Control System  
![World Representation Design](https://github.com/tdonca/ariac-2018-controls/blob/master/images/ControlSystemDiagram.png)  

#### Action Manager
* Receives high-level robot-independent action commands from a planner
* Converts action commands to lower level robot-independent tasks
* Sends tasks to the Controller Server for delegation
  

#### Controller Server
* Receives robot-independent tasks from Action Manager
* Translates tasks to robot-specific executions and send them to available robot controllers
  

#### IIWA14 Controller
* KUKA IIWA14 Robot controller
* Receives robot-specific executions from the Controller Server
* Executes using MoveIt for motion planning and collision avoidance
  
