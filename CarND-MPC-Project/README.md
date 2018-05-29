# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---

## Model Predictive Control
The [MPC](https://en.wikipedia.org/wiki/Model_predictive_control) is an advanced control technique for complex control problems. MPC is an optimization problem to find the best set of control inputs that minimizes the cost functions based on the prediction (dynamical) model. 
It generally includes 7 steps:
* 1. Set prediction horizon: N and dt.
* 2. Fit the polynomial to the waypoints.
* 3. Calculate initial cross track error and orientation error values.
* 4. Define the components of the cost function (state, actuators, etc). 
* 5. Define the model constraints. 
* 6. Update states
* 7. Deal with latency

### 1. Set prediction horizon: N and dt
The prediction horizon is the duration over which future predictions are made. We’ll refer to this as `T`. `T` is the product of two other variables, `N` and `dt`. `N` is the number of timesteps in the horizon. `dt` is how much time elapses between actuations. For example, if `N` were 20 and `dt` were 0.5, then `T` would be 10 seconds. `N`, `dt`, and `T` are hyperparameters you will need to tune for each model predictive controller you build. However, there are some general guidelines. `T` should be as large as possible, while `dt` should be as small as possible.
In our model, `N` is set to be 10 and `dt =0.1` second.

### 2. Fit the polynomial to the waypoints
As we learned in the previous class, the trajectory is typically passed to the control block as a polynomial. This polynomial is usually 3rd order, since third order polynomials will fit trajectories for most roads. In our project, we try to: 
1. Use `polyfit` to fit a 3rd order polynomial to the given `x` and `y` coordinates representing waypoints.
2. Use `polyeval` to evaluate `y`values of given `x` coordinates.

These two functions are defined as: 
```
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
```
### 3. Calculate initial cross track error and orientation error values

Cross track error: We can express the error between the center of the road and the vehicle's position as the cross track error (CTE). The CTE of the successor state after time t is the state at t + 1, and is defined as:
![](https://upload-images.jianshu.io/upload_images/6982894-99ee4e7525107ef3.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
Orientation Error:
![](https://upload-images.jianshu.io/upload_images/6982894-622a8bb8d1941aee.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/400)

### 4. Define the components of the cost function (state, actuators, etc)
The cost includes three parts: state loss, actuators loss, gap between sequential actuations. The cost of a trajectory of length N is computed as follows:
```
 // The part of the cost based on the reference state.
	  for (size_t t = 0; t < N; t++) {
		  fg[0] += 8000 * CppAD::pow(vars[cte_start + t], 2);
		  fg[0] += 8000 * CppAD::pow(vars[epsi_start + t], 2);
		  fg[0] += 5 * CppAD::pow(vars[v_start + t] - ref_v, 2);
	  }

	  // Minimize the use of actuators.
	  for (size_t t = 0; t < N - 1; t++) {
		  fg[0] += 150 * CppAD::pow(vars[delta_start + t], 2);
		  fg[0] += 50 * CppAD::pow(vars[a_start + t], 2);
	  }

	  // Minimize the value gap between sequential actuations.
	  for (size_t t = 0; t < N - 2; t++) {
		  fg[0] += 5 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
		  fg[0] += 5 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }
```
### 5. Define the model constraints. 
The actuators constraints limits the upper and lower bounds of the steering angle and throttle acceleration/brake. In our project, we define the constraints as:
```
  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
	  vars_lowerbound[i] = -1.0e19;
	  vars_upperbound[i] = 1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++) {
	  vars_lowerbound[i] = -0.436332;
	  vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; i++) {
	  vars_lowerbound[i] = -1.0;
	  vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
```
### 6. Update states
After defining the cost and constraints, the states update as follows:
![](https://upload-images.jianshu.io/upload_images/6982894-c000b0041da3171e.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/1240)
`Lf` measures the distance between the front of the vehicle and its center of gravity. `f(x)` is the evaluation of the polynomial `f` at point `x` and `psidest` is the tangencial angle of the polynomial `f `evaluated at `x`.

### 7. Deal with latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.
This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.
One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.
In our project, we solved this problem by predicting the next state before calling the MPC solver. Here is the code:
```
//predict next state to avoid Latency problems
		  //Latency of .1 seconds
		  double dt = 0.1;
		  const double Lf = 2.67;
		  double x1 = 0, y1 = 0, psi1 = 0, v1 = v, cte1 = cte, epsi1 = epsi;
		  x1 += v * cos(0) * dt;
		  y1 += v * sin(0) * dt;
		  //steer_value is negative
		  psi1 += -v / Lf * steer_value * dt;
		  v1 += throttle_value * dt;
		  cte1 += v * sin(epsi1) * dt;
		  //steer_value is negative
		  epsi1 += -v * steer_value / Lf * dt;
		  Eigen::VectorXd state(6);
		  state << x1, y1, psi1, v1, cte1, epsi1;
```

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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
