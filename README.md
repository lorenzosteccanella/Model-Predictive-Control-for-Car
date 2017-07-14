
# Model Predictive Controll Project



This repository shows up the Model Predictive Control implemented to control a car stearing angle and throttle using a global kinematic model.

# Reflections 


## The Model

The simulator created by Udacity to test our model predictive control alows us to get as input for our MPC the following data:

--> ptsx (x global coordinate of N waypoints ahead on the track)

--> ptsy (y global coordinate of N waypoints ahead on the track)

--> x ( x global coordinate of position of the car)

--> y ( y global coordiante of position of the car)

--> psi ( orientation angle of the car)

--> v ( velocity of the car)

--> delta ( stearing angle of the car )

--> a ( throttle )

In order to predict the state of the vehicle a Global Kinematic Model was used, where:

x_t+1=x_t+v_t∗cos(ψ_t)∗dt

y_t+1=y_t+v_t∗sin(ψ_t)∗dt

ψ_t+1=ψ_t+(v_t/L_f)∗δ_t∗dt

v_t+1=v_t+a_t∗dt

Kinematic model in comparison to Dynamic models doesn't take in account tire forces, longitudinal and lateral forces, inertia, gravity, air resistance, drag, mass, and the geometry of the vehicle but results enough accurate to allow our vehicle drive around the track.

## Timestep Length and Elapsed Duration (N & dt)

    --> Timestep Length (N) = 10
    --> Elapsed Duration (dt) = 0.1
    

The prediction horizon is the duration over witch future predictions are made. 
Prediction horizon is defined by the product of the two variables N and dt.
N is the number of timesteps in the horizon. dt is how much time elapses between actuations.

The value for N and dt where choosen with a trial and error.

## Polynomial Fitting and MPC Preprocessing

As first the points coordinate where transformed from global into vehicle's coordinate from line 104 to 112 in main.cpp. Then using the polyfit() function, a third-degree polynomial line is fit to the transformed waypoints, resulting in the path that the vehicle should try to follow.

The cross_track error is calculated using the poly_eval function evaluated in the poin px.


## Model Predictive Control with Latency

The kinematic model prediction is used for dealing with the controller delay. In fact a latency of 100 ms is simulated, using the kinematic model we can make a prediction based on this latency and in this way we can take in account of this delay in our model

x_t+1=x_t+v_t∗cos(ψ_t)∗dt

y_t+1=y_t+v_t∗sin(ψ_t)∗dt

ψ_t+1=ψ_t+(v_t/L_f)∗δ_t∗dt

v_t+1=v_t+a_t∗dt



# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.













```python

```
