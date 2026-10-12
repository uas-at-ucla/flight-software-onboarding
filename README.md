# UAS Flight Software Subteam Onboarding Project Guidelines

This is a project for members to complete in order to famarialize themselves with the computer vision and flight software topics covered over previous workshops. Although it is not required to work on future projects, it is heavily recommended. Feel free to look at the `examples/` folder to see how to use the simulator camera.

## Guidelines
There are three different tasks to this project that members must have the drone complete. Each task is similar to but also noticibly more difficult than the last. Although, code from previous tasks can be used and will often be helpful for completing more difficult tasks.

Feel free to use any functions within the MAVSDK-Python Library and any additonal libraries with any method to succeed in the specified tasks.

## Tasks
### Centering and Hovering Over Circle 
![Drone Over Stationary Circle](assets/images/red_circle.png)

In this task the drone spawns next to a red circle of which the drone must center on while in the air. Use and adapt the Week 1 OpenCV workshop color detection code in order to determine the offset of the drone and the red circle. Use this offset in your flight code to adjust the drone accordingly. 

Once the drone is directly over the circle, the drone will land.

The world name is "red_circle".

### Following a Curvy Path
![Drone Over Curvy Path](assets/images/curvy_path.png)

In this task the drone spawns onto a long continous curvy path of which the drone must navigate to the end of. Similarily to the last task, vision server to analyze images sent by flight code to move along the path and adjust during curves.

Once the drone reaches the end of the line, it will land.

The world name is "path".

### Following a Moving Circle
![Drone Over Stationary Circle](assets/images/moving_circle.gif)

In this final task, the drone spawns near a red circle continously moving in a circle. The drone must take off and begin to follow the red circle when it comes into view again. The drone will continue to follow the red circle for an indefinite amount of time while trying to stay centered over it at all times.

The world name is "moving_circle".
## Real World Tests
After successfully writing scripts that can execute these tasks in the simulation, they will be tested during the onboarding project fly day on real drones. More information about the fly day will be released in the future.

## Launching the Simulator
Navigate to the PX4-Autopilot folder and run `make px4_sitl gazebo-classic_typhoon_h480__[world_name]`. This will open the designated world for you to test on. You may need to restart the environment upon every test.
