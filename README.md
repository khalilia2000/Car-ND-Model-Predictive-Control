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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

note: CMakeLists.txt was changed by replacing `target_link_libraries(mpc ipopt z uv uWS ssl)` to `target_link_libraries(mpc ipopt z uv uWS -Wl, -whole-archive ssl -Wl,-no-whole-archive)`.  


## Rubric

The rubric for this project is locate [here](https://review.udacity.com/#!/rebrics/896/view).


### Compilation

The code should compile fine using CMake and Make. I did have to make a slight change to the CMakeLists.txt to have it workon Ubuntu 16.04.  

### The Model

The model consists of a kinematic vehicle model that is described in Udacity courses. The state variable consistsof 6 elements (x, y, psi, v, cte, epsi). The last two elements of the state vector are cross track error and psi error. The initial state that is received from the simulator is passed to the MPC class at each step. The model update equations used for predicting the state of the car are as follows (lines 115 to 120 of (MPC.cpp):  
- updating x: x0 + v0 x cos(psi0) x dt
- updating y: y0 + v0 x sin(psi0) x dt
- updating psi: psi0 + v0 x delta0 / Lf x dt
- updating v: v0 + a0 x dt
- updating cte: (f0 - y0) + v0 x epsi0 / dt
- updating epsi: (psi0 - psides0) + (v0 x deltaa0 / Lf x dt)

in which f0 is the evaluatoin of x0 using a 3rd degree fitted polynomial, psides0 is the angle of the tanget to the polynomical, dt is the timestep, and Lf is the distance from the front of the vehicle to the centre of gravity.  

### Timestep Length and Frequency  

N and dt were chosen using trial and error. I found that dt values of smaller than 0.1 seconds generally work OK, however what is really important is the total time interval of prediction ahead of the current state (i.e. N x dt). I experimented with different values, and found out that N x dt of approximately 1 seconds provide a good stable implementation. if N x dt is too much (i.e. 2 secons, it results in instability in some cases due not finding a reasonable answer in the solver for some cases. if N x dt is too small, it does not provide a good actuation recommendation. Given the latency of 100 milliseconds, I adopted 0.9 seconds as the prediction time interval (i.e. N x dt), with N = 18 and dt = 0.05.  

### Polynomial Fitting and MPC Preprocessing

The waypoints received from the simulator are first pre-procesed and are converted to the vehicle coordinate system. This pre-processing is performed in lines 109 to 114 of main.cpp, using the following formulas:
```
ptsx_vcoord(i) =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-[y)*sin(psi);
ptsy_vcoord(i) = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-[y)*cos(psi);
```
Then a third order polynonmial is fitted to the revised waypoints in line 117 of main.cpp:
` auto coeffs = polyfit(ptsx_vcoord, ptsy_vcoord, 3)`

### Model Predictive Control with Latency

A latency of 100 milliseconds is considered in the model. The latency was dealth with using the following approach: 
1- the state of the model at the end of the latency period was predicted assuming that the previous actuator contorls would be used during the whole latency time period. This was done in lines 129 to 145 of the main.cpp.
2- The state at the end of the latency was then passed on to the MPC object for estimating the best actuation control. This was then passed to the simulator as the desired control. 

### Simulation

The is stable and can drive around the track safely at 40 MPH speed. At higher speeds, the car may become unstable.
