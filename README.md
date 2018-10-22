# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


I implemented a MPC  controller in c++ in this project.

![](img/example.png)



### Introduction

Steps:

1. Set N and dt.
2. Fit the polynomial to the waypoints.
3. Calculate initial cross track error and orientation error values.
4. Define the components of the cost function (state, actuators, etc).
5. Define the model constraints. These are the state update equations defined in the Vehicle Models module.

Kinematic model is used in MPC. 

- Global Kinematic Model

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{aligned}&space;x_{t&plus;1}&space;&=&space;x_t&space;&plus;&space;v_t&space;*&space;cos(\psi_t)&space;*&space;dt&space;\\&space;y_{t_1}&space;&=&space;y_t&space;&plus;&space;v_t&space;*&space;sin(\psi_t)&space;*&space;dt&space;\\&space;\psi_{t&plus;1}&space;&=&space;\psi_t&space;&plus;&space;\frac{v_t}{L_f}&space;*&space;\delta&space;*&space;dt&space;\\&space;v_{t&plus;1}&space;&=&space;v_t&space;&plus;&space;a_t&space;*&space;dt&space;\end{aligned}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{aligned}&space;x_{t&plus;1}&space;&=&space;x_t&space;&plus;&space;v_t&space;*&space;cos(\psi_t)&space;*&space;dt&space;\\&space;y_{t_1}&space;&=&space;y_t&space;&plus;&space;v_t&space;*&space;sin(\psi_t)&space;*&space;dt&space;\\&space;\psi_{t&plus;1}&space;&=&space;\psi_t&space;&plus;&space;\frac{v_t}{L_f}&space;*&space;\delta&space;*&space;dt&space;\\&space;v_{t&plus;1}&space;&=&space;v_t&space;&plus;&space;a_t&space;*&space;dt&space;\end{aligned}" title="\begin{aligned} x_{t+1} &= x_t + v_t * cos(\psi_t) * dt \\ y_{t_1} &= y_t + v_t * sin(\psi_t) * dt \\ \psi_{t+1} &= \psi_t + \frac{v_t}{L_f} * \delta * dt \\ v_{t+1} &= v_t + a_t * dt \end{aligned}" /></a>

`[x, y, psi, v]` is the state of the vehicle, Lf is a physical characteristic of the vehicle

`[delta, a]` are the steer value and throttle to the system.


Consider the errors in the kinematic model, the new state is [x, y, psi, v, cte, epsi]

- Cross track error

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{aligned}&space;cte_{t&plus;1}&space;&=&space;cte_t&space;&plus;&space;v_t&space;*&space;sin(e\psi_t)&space;*&space;dt&space;\\&space;cte_t&space;&=&space;y_t&space;-&space;f(x_t)&space;\\&space;cte_{t&plus;1}&space;&=&space;y_t&space;-&space;f(x_t)&space;&plus;&space;v_t&space;*&space;sin(e\psi_t)&space;*&space;dt&space;\end{aligned}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{aligned}&space;cte_{t&plus;1}&space;&=&space;cte_t&space;&plus;&space;v_t&space;*&space;sin(e\psi_t)&space;*&space;dt&space;\\&space;cte_t&space;&=&space;y_t&space;-&space;f(x_t)&space;\\&space;cte_{t&plus;1}&space;&=&space;y_t&space;-&space;f(x_t)&space;&plus;&space;v_t&space;*&space;sin(e\psi_t)&space;*&space;dt&space;\end{aligned}" title="\begin{aligned} cte_{t+1} &= cte_t + v_t * sin(e\psi_t) * dt \\ cte_t &= y_t - f(x_t) \\ cte_{t+1} &= y_t - f(x_t) + v_t * sin(e\psi_t) * dt \end{aligned}" /></a>

`y_t - f(x_t)` is current cross track error

`v_t * sin(epsi_t) * dt` is the change in error caused by the vehicle's movement.


- Orientation error

<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{aligned}&space;e\psi_{t&plus;1}&space;&=&space;e\psi_t&space;&plus;&space;\frac{v_t}{L_f}&space;*&space;\delta_t&space;*&space;dt&space;\\&space;e\psi_t&space;&=&space;\psi_t&space;-&space;\psi&space;des_t&space;\\&space;e\psi_{t&plus;1}&space;&=&space;\psi_t&space;-&space;\psi&space;des_t&space;&plus;&space;\frac{v_t}{L_f}&space;*&space;\delta_t&space;*&space;dt&space;\end{aligned}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{aligned}&space;e\psi_{t&plus;1}&space;&=&space;e\psi_t&space;&plus;&space;\frac{v_t}{L_f}&space;*&space;\delta_t&space;*&space;dt&space;\\&space;e\psi_t&space;&=&space;\psi_t&space;-&space;\psi&space;des_t&space;\\&space;e\psi_{t&plus;1}&space;&=&space;\psi_t&space;-&space;\psi&space;des_t&space;&plus;&space;\frac{v_t}{L_f}&space;*&space;\delta_t&space;*&space;dt&space;\end{aligned}" title="\begin{aligned} e\psi_{t+1} &= e\psi_t + \frac{v_t}{L_f} * \delta_t * dt \\ e\psi_t &= \psi_t - \psi des_t \\ e\psi_{t+1} &= \psi_t - \psi des_t + \frac{v_t}{L_f} * \delta_t * dt \end{aligned}" /></a>

`psi_t - psides_t` is current orientation error.

`v_t/Lf * delta * dt` is  the change in error caused by the vehicle's movement.

- Polynomial Fitting

Due to the coordinate difference between vehicle state and waypoint position, we need to convert  waypoints to vehicle coordinate from map coordinate for convenience of polynomial fitting. We calculate the relative position between vehicle and waypoints first, and then rotate `psi` in  counter clockwise direction. Then fitting the 3rd polynomial curve use the new positions of waypoints.

```
for(size_t i=0; i<ptsx.size(); ++i){
  // relative position to the vehicle
  double x = ptsx[i] - px;
  double y = ptsy[i] - py;
  //transform from map coordinate to vehicle coordinate
  ptsx_r[i] = x*cos(psi) - y*sin(psi);
  ptsy_r[i] = x*sin(psi) + y*cos(psi);
}
```

- Actuation Latency

The actuation command won't execute instantly in real car,  there will be a delay as the command propagates through the system. A 100ms latency is added in the simulator to simulate this condition.
If the controllers  calculate the error with respect to the present state, but the actuation will be performed when the vehicle is in a future state. This can sometimes lead to instability problem. 

This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.

To cope with actuator latency, I add a step to predict future state based on vehicle kinematic model, The resulting state from the simulation is the new initial state for MPC.
`x, y, psi` state are zeros because the transformed vehicle coordinate.

```
// predict state after 100ms           
const double Lf = 2.67;
const double dt = 0.1;
double pred_x = 0 + v * cos(epsi) * dt;
double pred_y = 0 + v * sin(epsi) * dt;
double pred_psi = 0 + v * delta / Lf * dt;
double pred_v = v + acc * dt;
double pred_cte = cte + v * sin(epsi) * dt;
double pred_epsi = epsi + v / Lf * delta * dt;
```

- Parameters setting

`N = 10` and `dt = 0.1` is selected for this MPC controller.

`N * dt` is the time MPC predicted, and the disired velocity is settled as 80mph (~36m/s), I think the predicted duration `1s` is enough for safety. The actuation  latency is `0.1s`, so the same value is used for `dt`, `N = 10` is obtained in the same time. 

In theory, the larger`N*dt` and `N` , the better performance can be predicted. However, more computation resource is needed, which is chanllenge for real time control on self driving car. 



### Dependencies

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
* Read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

