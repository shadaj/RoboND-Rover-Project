# Rover Project Writeup

## Notebook Analysis
### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
To detect navigable areas, I used the default color threshold of `> rgb(160, 160, 160)`, which worked well in tests. Obstacles are calculated by taking the opposite of the navigable area and applying the mask of the warped image to only include negated parts in the field of view.

For sample detection, I created a new function that could find pixels in between two RGB values so that I could detect the specific color of the samples.

### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.
The `process_image()` function is relatively straightforward, as the only remaining steps after filtering out the navigable terrain, obstacles, and samples are to convert the filtered image to rover coordinates and convert those coordinates to world coordinates based on the robot's pose. I then use these coordinates to update the world map by bumping the color values for each type of coordinate by one.

## Autonomous Navigation and Mapping

### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
The `perception_step()` closely matches my `process_image()` with extra logic to drive up to samples, follow walls, and only record data when the rover is relatively flat.

When building out the map, I first make sure that the pitch and roll of the rover are close to zero. If the rover has detected samples it is likely moving close to a wall, where collected mapping data is usually wrong, so I do not record any navigable terrain or obstacle information. Because the rover is much more confident about navigable area, I significantly bump the map information for navigable terrain at recorded coordinates and lower obstacle values at those coordinates.

To drive up to samples, the rover automatically sets the target angles to the sample angles whenever it detects a sample on the left side or slightly to the right (this is related to the wall following discussed later). In addition, we record obstacle distances with a bump of +5 to account for the fact that samples are near walls and we do not want the robot to turn away from a sample because it is too close to a wall.

The `perception_step()` also includes logic for wall following, which I implemented so that the rover maps more of the terrain. The rover tries to stay on the left, so I apply multiple filters to bias the rover towards following navigable terrain that is on the left side of its view. In addition to angle bias, I also calculate a target velocity for the rover (stored in `max_vel`) by scaling the average navigable terrain distance. If the rover is tilted upwards `pitch > 350`, I increase the max velocity by 1 m/s to account for the rover measuring less navigable terrain.

In `decision_step()`, there are two major changes: using proportional control for maintaining a target velocity and the introduction of a spinning state to work around obstacles. I repurposed the `max_vel` variable to store a target velocity, which I then follow by using proportional control `Rover.throttle = (Rover.max_vel - Rover.vel) / 1.25`. I also added a spinning state, which is used when the rover does not see much navigable terrain directly in front of it to spin in place until there is navigable terrain to move into.

### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

I ran the simulator at a resolution of 1280x720 with fantastic graphics quality.

The rover completes all the tasks of the assignment except returning samples to the initial starting position. It is able to map almost all of the terrain with relatively high fidelity and can detect and pick up all of the samples in the map. The rover is able to move quite fast, since it can adapt its speed depending on how much open terrain is visible in front of it. Due to simulator issues with the camera occasionally seeing through obstacles, the rover sometimes gets stuck, thinking that it can move forward even if it is against a rock. In the future, this can be fixed by detecting when the position has not changed for some time. Another part to improve would be the wall following, which sometimes fails to detect side corners especially if the rover is moving fast.

Simulation results can be seen in the `navigation-demonstration.mp4` file.
