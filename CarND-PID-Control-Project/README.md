# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
##Describe the effect each of the P, I, D components had in your implementation.
* P: Proportional, which means a proportional to cross-track error (CET) term and determines the speed of the control system response as the ratio of output response (defined by weight Kp) to the CET signal.  The oscillation will increase if the value is too high. However, the steering angle will be too small and the toy car will hardly reach the trajectory when the value is too low.
* I: Integral, which is proportional to both the magnitude of the CET and the duration of the CET and is defined by weight Ki. It counteracts a bias in the cross track error, which prevents the PD controller from reaching the center line
* D: Differential, which is proportional to the rate of change of the CET. It considers the rate of change in the error, prevents overshooting the center line.

##Describe how the final hyperparameters were chosen.
* First, I manually chose the value of parameters Kp, Ki, Kd. Here are the experiments record:
1. Tuning Kp, fix Ki=Kd=0
Kp=1, turn right outside the road
Kp=10, the same as P=1
Kp=0.1, turn right slowly, but still, drive out the road and fail to come back
Kp=0.01, turn right slower, but still, drive out the road and fail to come back
Kp= -0.1, it can drive normally for a while, but dive into the water before going to the bridge.
Kp=-1, turn left and right frequently, Kp is too large, the vehicle will begin to oscillate around the target trajectory.

2. Tuning Ki, fix Kp=-0.1, Kd=0
Ki = 1, turn right outside the road
Ki = 0.1, turn right outside the road
Ki = 0.01, turn right outside the road
Ki = -0.1, turn right outside the road and then turn right with big oscillate
Ki = -0.01, turn left and right then go outside the road, but the oscillation is much small
Ki = -0.001, turn left and right then go outside the road, but the oscillation is much small
Ki = -0.0001, it can drive normally for a while, turn left and right frequently when meeting the sharp turn. However, it dives into the water before going to the bridge.
Ki = -1, turn left and right frequently, Ki is too large, the vehicle will begin to oscillate around the target trajectory.

3. Tuning Kd, fix Kp=-0.1, Ki=-0.0001
Kd= 1, it can drive normally for a while, but it turns left and right frequently when meeting the turn.
Kd= 10, the wheel oscillates around the trajectory, and the car is jittering.
Kd= 0.1, it can drive normally for a while, turn left and right frequently when meeting the sharp turn. 
Kd = 0.01, the same as Kd=0.1
Kd = -0.01, more stable than Kd=0.1, but drive outside the road when meeting a long turn.
Kd = -0.1, similar to Kd = -0.01.
Kd = -1, So far the best performance!  It can go through the bridge smoothly but dive into the water when meeting a sharp turn.
Kd = -2, It can go through all tracks, with only a little cross over the border when meeting a sharp turn.
Kd = -4, It can go through all tracks, with only a little cross over the border when meeting a sharp turn. Speed can reach to 30+MPH.

* Second, fine-tuning Kp, Ki, Kd
After manually set Kp = -0.1, Ki = -0.001, Kd = -3 as initial value of model, I adopt twiddle to make a fine-tuning.  The code is completed in update part and we set the dp = [0.2*Kp, 0.2*Ki, 0.2*Kd]. After almost 10 round races, parameters converge to the stable states.

* Finally, throttle tuning and steer_value improvement.
After the fine-tuning in step 2, the toy car still drives too close to the edge when driving through one sharp turn. To overcome this problem, I add a restriction to the throttle: when driving into the sharp turn, the throttle is reduced to 0.2, even reach to 0 when the cte is over 1.6. Accordingly, the steering angle increases 1.2 times. This strategy makes our car drive more smoothly along the track in the speed of 40MPH. 
Here are the final weights in my PID controller: Kp = -0.122, Ki = -0.00014, and Kd = -3.6. 

---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

