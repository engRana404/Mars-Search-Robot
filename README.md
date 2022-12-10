[image_0]: ./misc/rover_image.jpg

<h1 align="center">NASA Mars Sample & Return Rover</h1>

## Aim of the project

In this project, we’ll do computer vision for robotics where we are going to build a Sample & Return Rover in simulation by controlling the robot from images streamed from a camera mounted on the robot.
###### The project aims to do autonomous mapping and navigation given an initial map of the environment.

## The Simulator
 
You can test out the simulator by opening it up and choosing "Training Mode".  Use the mouse or keyboard to navigate around the environment and see how it looks.


## Data Analysis
Included in the IPython notebook called <code>[NoteBook.ipynb](https://github.com/engRana404/Mars-Search-Robot/blob/main/NoteBook.ipynb)</code> are the functions from the lesson for performing the various steps of this project.

## Navigating Autonomously
The file called `drive_rover.py` is what you will use to navigate the environment in autonomous mode.  This script calls functions from within `perception.py` and `decision.py`.  The functions defined in the IPython notebook are all included in`perception.py` and it's your job to fill in the function called `perception_step()` with the appropriate processing steps and update the rover map. `decision.py` includes another function called `decision_step()`, which includes an example of a conditional statement you could use to navigate autonomously.  Here you should implement other conditionals to make driving decisions based on the rover's state and the results of the `perception_step()` analysis.

`drive_rover.py` should work as is if you have all the required Python packages installed. Call it at the command line like this: 

```sh
python drive_rover.py
```  

Then launch the simulator and choose "Autonomous Mode".  

<img
  src="https://github.com/engRana404/Mars-Search-Robot/blob/main/Pictures/Screenshot%20(1837).png" width = 600 height = 450
  title="Screenshot for Phase 1"
  style="display: inline-block; margin: 0 auto; max-width: 300px">


## Contributors

<a href = "https://github.com/engRana404/Mars-Search-Robot/graphs/contributors">
  <img src = "https://contrib.rocks/image?repo=engRana404/Mars-Search-Robot"/>
</a>
