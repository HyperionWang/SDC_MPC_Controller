# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./Doc/MPC_Control_Loop.png
[image2]: ./Doc/StatePredictEquation.png
[image3]: ./Doc/MPC_Path_Plot.png
[image4]: ./Doc/MPC_Driving.gif

---
## Overview

The MPC is the self-driving car's driving controller based on the Kinematic Model to follow the planned driving path by predicting the driving state in next several steps (several seconds range) based on the current state and designed constrain. Comparing to PID controller, MPC not only looks at car's current state and error (cross track error), but also optimize the actuation command (acceleration and yaw rate) to follow the planned path for next few seconds. Therefore, the MPC could be designed with more sophisticated constrains, and be able to handle complicated road conditions with better stability. Therefore, passengers experience in self-driving car would be improved. 

The MPC program consists of modules as follow: state update, polyfitting on planned path, state prediction, future actuation optimization. The state update is to read the current car's state (position, yaw, speed, planned path), and to convert the state info into the car's own coordinate. The polyfitting module is using a polynomial curve (third order is used in the project) to fit the planned path, and then to extract the cross track error (CTE) and yaw error (epsi) by comparing the car's current state with the planned path. In the state prediction module, by using Kinematic model with CTE/epsi info, the controller predicts the car's states in near future (in several seconds) with a series of actuation commands (steering angle and throttle). In the optimization module, the actuation commands are optimized to achieve the minimum error calculated by a designed cost function based on several constrains, including minimizing CTE, epsi, speed error, the change rate of throttle and steering angle, as well as the controller's time delay. Then, the MPC controller applies the optimal throttle and steering until the car's state is updated again. The MPC controller then goes into the next cycle.

![MPC Control Loop][image1]

## The Model in MPC

In the MPC model, the state info includes: x and y poistion (reference to car), yaw, speed, CTE, and epsi. And the actuators are: steering angle (delta), and acceleration (a). The model would predict the next state based on the current state and actuators. The equations are:
![MPC Control Loop][image2]

The method to predict CTE and epsi is to calculate the difference between predict state and planned path (represented by coefficients from the poly-fitting)

In the MPC's code, the update equation is presented through the cost function with the constrain equations. Shown in line 139 to 146 in MPC.cpp, and line 235 to 238. Since the right part of the equation is moved from the right of the equal sign, the result of the prediction equation should be zero for the updating, which should be like the equation as following in general:

state (t+1) - f(state(t)) = 0 

where f(state(n)) is the update equation. 

The code is as following:

```
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);


// Lower and upper limits for the constraints
// Should be 0 besides initial state.          
for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

```

The upper and lower bound on the steering angle and throttle are based on the system requirement (+/-25 degree for steering angle and +/-1 throttle).

For the designed cost function model, the CTE, epsi, speed error, steering angle, throttle, change rate on steering angle, and change rate on throttle are included in the cost function for the minimization. The squared function on those parameters with different factors are added as the value of the output of the cost function. By changing the factor on each paramters, the weigh of each parameter in the cost function is adjusted. The final factors on each parameters are as following:

```
      for (int t = 0; t < N; t++) {

        fg[0] += CppAD::pow(vars[cte_start + t], 2);
        fg[0] += CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += 0.5 * CppAD::pow(vars[v_start + t] - ref_v, 2);
      }

      // To minimize the change of the velocity and direction actuators

      for (int t = 0; t < N - 1; t++) {
        fg[0] += CppAD::pow(vars[delta_start + t], 2);
        fg[0] += 10* CppAD::pow(vars[a_start + t], 2);
      }

      // To minimize the change rate of the acuations

      for (int t = 0; t < N - 2; t++) {
        fg[0] += 580 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += 0.5 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }



```

##Timestep Length and Elapsed Duration

The time step length and elapsed duration (N nand dt) are two important parameters for controlling the update rate and calculation accuracy on the MPC. There is a trade of between the calculation and the prediction accuracy/update speed. The finer time step (dt) & longer elapsed duration, the higher prediction accuracy but the calculation cost would be higher. If the time step is too small (like 0.01), in order to predict the state in the next half second, the number of steps to predicts has to be larger than 50. It will take time longer than the predict time period, and therefore makes the control loop lag too much behind the state update and causes failure on the control. 

After the trial and error, using the timestep of 0.05 and number of step of 12 makes the MPC stable and control the car's driving well. Also, since the system has the latency of 100 ms, which equals to two time steps in the state update. 


##Polynomial fitting and MPC Preprocessing

A third order polynomial fitting is applied to the planned path. The CTE is calcuated through the difference between the planned path polyfitting and the state prediction. And the epsi is calculated by using the derivative of the polyfitting.

Before the polynomial fitting, the updated state data and planned path fed by the system is preprocessed, so that the coordinate of the data is converted into the car's coordinate. 

The conversion is as follow:

```c++

          if (ptsx.size() == ptsy.size()) {

            double dx, dy;
            for (unsigned i = 1; i < szx; i++) {

              dx = ptsx[i] - px;
              dy = ptsy[i] - py;

              car_x.push_back(dx * cos(-psi) - dy * sin(-psi));
              car_y.push_back(dx * sin(-psi) + dy * cos(-psi));
            }

          }

```

After the conversion, the car's position and yaw (current state) is alway x = 0, y=0 and yaw = 0. Therefore, in the following equation to calculate CTE and epsi, x = 0 is used. 

```c++

          double cte = polyeval(coeffs, 0) - 0;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x + coeffs[2] * x^2-> coeffs[1] + 2* x * coeffs[2]
          double epsi = 0 - atan(coeffs[1]);

```

The plot of the planned path and car's MPC predicted path could be seen by the yellow and green line in the following pic:

![MPC Control Loop][image3]


##Model Predictive Control with Latency

The system latency is 100 ms, which is two timestep of 0.05 each. The way to handle the latency in the program is to send the system with actuation info with delayed timestep of t + 2. For example, at t = 0, the MPC calculate the control for time step from t + 1 to t + 12. Instead of sending to system with actuation at t + 0, sending to system with acutation at t + 2, which represent the latency of 100 ms. 

By sending acuation at t + 0, the actuation is two updates lagging behind the car's state, and therefore, the system is unstable (meaning oscillating on the road). With sending actuation with two timestep ahead, the lag is conpensated by the prediction, as long as the prediction are good (which is the case shown in the project), the car's actuation is in phase with the state, and therefore the MPC control the car with no large phase delay and the feedback is stable. 

##Summary

The MPC control system for real-time control on car's steering angle and throttle has been implemented. The MPC is able to drive the car in the simulator continously and stably at speed of 50 - 60 MPS, without any unsafe accident occurs. 

![MPC Control Loop][image4]

The designed state update, cost function, and optimization constrain could have some improvements in future:
1. The Kinematic model could be replaced by dynamic model by taking more factors into consideration, such as Slip and Tire Model.
2. The speed control could be working with the perception better, instead of setting to a constant speed. For example, as the car see the turns, even the current constant speed could pass through, but the passenger's experience may feel unsafe since it is more like a roller coaster. Naturally, the car should slow down a little and to make the pass through more smoothly.
3. The adjustment on the actuation delay on the latency could be dynamic. It is useful since the latency in the real car's control would be changing all the time, and the dynamic adjustment on the actuation with consideration on the latency would make MPC's control more stable and handle more complicated road conditions. 