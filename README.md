# PID Controller Project

This repository contains implementation of the PID controller that allows to control the car driven on the race track in simulation environment. The race track is circular with several left and right corners with different curvature radiuses.

## Some theory behind PID

PID Controller consists of Proportional, Integral and Differential controlling components respectively

The proportional component defines the steering angle of the car proportionally to the distance of the car from lane center line, thus constantly tries to steer the car towards the middle of the lane to minimize the cross-track error at current position. The side effect of this process is that car constantly oscilates along the center line because of inertion effect (overshooting effect). Thus keeping of the car on the road only with P component is hardly possibe. To solve this issue differential component steps onto the stage.

The differential part of PID controller defines the linear dependency of steering angle from the speed of change of cross track error over the driving time. This allows to compensate the constant overshooting of lane center line caused by proportional component and let the steering angle to decrease smoothly over the time, every time when the car approaches the lane center line (fading of oscilation)

The integral component of PID controller accumulates all the cross track erros over the driving time. In this way it is possible to detect whether the car is constantly driving next to the lane center line, and to compensate this deviation, by causing the car to shift toward the middle of the lane.

The resulting steering angle is calculated by PID controller using following formula:
    
    α = (-Kp * error) + (-Kd * d(error)/dt) + (-Ki * ∑ error)
    
 where  Kp, Ki and Kd are parameters tunning the effect of P, I and D components of controller respectively.


## What was intended to make

The goal of the project was to build the PID controller application, that would keep the car constantly driving as close as possible to the middle of the lane and perform smooth left and right turns without running outside the drivable area of the lane (considered as unsafe). Additionally, to make the car control look more natural, it was decided to build the PID controller in the way, that could allow to dinamically define the car speed, that is safe enough for performing the car control in different driving situations

As the simulation environment the Udacity's self driving car simulator is used. It constantly measures the cross track error (CTE) between the lateral position of car and the lane center line. This error, together with current car speed and turning angle, is constantly being sent to the running PID controller application over the web socket communication channel. This information then is used by PID controller to calculate the best values for steering and throttle at each position on the road, that would allow to satisfy the project goals.

## How PID parameter values were selected

The Kp, Ki and Kd parameter values of implemented PID controller were choosen in such a way, that
allows the most smooth while still safe way of driving of the car along the simulated race track. 
To pick such values the parameter calibration process was implemented as part of this application, that allows to fine tune initialaly selected (good enough for keeping the car on the road) values of P, I and D parameters of controller using Gradient Descent method. 

Unfortunatelly this optimization method aims the minimizing of the cross track error over predefined number of optimization steps, means trying to keep the car as close as possible to lane center line all the time, what does not necesseraly imply the most comfortable (car behaves ocasionally bit jerky), although still safe way of driving. 

Thus Kp, Ki and Kd parameter values for PID controller were selected empirically based on 'driving smoothness' criteria during the observation of the car's behavior at different stages of calibration process. 


## Setup instructions

This project requires the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Scripts install-ubuntu.sh and install-mac.sh can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems accordingly. For windows use either Docker, VMware, or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)


## Other Dependencies

* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4


## Build instructions

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make

Alternatively build script has been included to simplify this process, it can be leveraged by executing the following in the top directory of the project:

   ./build.sh


## Execution instructions

Created PID Controller application can be executed in two different modes:

### Calibration mode 

This mode allows to observe the process of fine tunning of initial (good enough) values for PID controller paramters (Kp, Ki, Kd) for proportial, integral and differential components respectively. Calibration has to be triggered separately for each parameter. 

To run the PID controller application in calibration mode:

1. Start created application executable file from build folder:
  
   ./pid calibrate <pi_parameter_name>
   
   where <pid_parameter_name> should be replaced with 'P', 'I' or 'D' respectively, depending on which paramer has to be calibrated. 
   After the finish of calibration process for requested parameter, the PID controller application will display the finetuned value of the parameter and stop the execution automatilcally.  
   
2. Start downloaded Term 2 Simulator application. Select requried resolution and press Play button.
   Switch to 'Project 4: PID Controller' using right arrow an press Select button.
   To run the filter press Start button. 

### Demo mode

In this mode the car is controlled by PID controller using empirically selected values for Kp, Ki, and Kd paramaters of PID controller

To run the PID controller application in demo mode:

1. Start created application executable file from build folder:
  
   ./pid
   
2. Start downloaded Term 2 Simulator application. Select requried resolution and press Play button.
   Switch to 'Project 4: PID Controller' using right arrow an press Select button.
   To run the filter press Start button. 
   