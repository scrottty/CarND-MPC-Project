# Project 10 - Model Predictive Control

## Project Description
The purpose of this project was to impliment the Model Predictive Control Method. This method uses a basic kinematic model of the system to produce an optimised path as a result of control efforts. The control efforts are then applied to the system and the optimised paths recalculated. By recalculating each timestep the inaccuracies of the kinematic models are minimised and thus producing a suitable result. The main benefit of this control method is its ability to take future events into account and react suitable to handle them before they arrive. 

## Solution Description
### The Model
The system is approximated using basic kinematic models. The vechiles state can be described as: `state = [ x, y, ψ, v]` where `x` is the vechiles x position, `y` is the vechiles y position, `ψ` is the orientation of the vechile and `v` is the vechiles velocity.

There are two errors for the system. `cte`, the distance from the intended path and `eψ`, the orientation error. These are added to the state vector so the final state vector is `state = [x, y, ψ, cte, eψ]`

To make the calculations simpiler the cars coordinate system was used as the reference. This meant that the coordinates from the simulator had to be converted. This was done with the following equations:
``` c++
dtx = ptsx[i] - px;
dty = ptsy[i] - py;

xvals(i) = dtx * cos(ψ) + dty * sin(ψ);
yvals(i) = dty * cos(ψ) - dtx * sin(ψ);
```
The vechile is controlled through two actuators, the steering wheel (`ẟ`) and the accelerator (`a`). Both values are bounded, the steering angle to +/- 25° and the accelerator to +/- 1 where positive is acclerate and negative is brake.

The vehicles states are updated using the following kinematic equations:
```c++
x(t+1) = x(t) + v(t) * cos * ψ(t) * dt
y(t+1) = y(t) + v(t) * sin * ψ(t) * dt
ψ(t+1) = ψ(t) + v(t)/Lf * ẟ * dt
v(t+1) = v(t) + a(t) * dt
cte(t+1) = f(x(t)) - y(t) + v(t) * sin(eψ(t)) * dt
eψ(t+1) = ψ(t) - ψdes(t) + v(t)/Lf * ẟ(t) * dt
```
where `f(x)` is the polynomial fit describing the target trajectory and `ψdes(t+1)` is the tangent of `f(x)` evaluated at `x(t)`

### Parameter Tunning
The model has the following hyperparameters to control the output performance:
`N, dt, min_velocity, max_velocity, ẟ_error_weighting, Δẟ_error_weighting, Δa_error_weighting`

`N` and `dt` control how far forward the model looks when optimising the path and the detail at which the path can be defined. These were choosen at 20 and 0.05 respectively which produces a 1 second horizon at a 0.05 second detail. They were selected as they produced the best result in regard to the car staying close to the intended trajectory. A larger `dt` resulted in the vehicle cutting corners as the detail of the optimised path was too coarse. `N` was then selected to produce a horizon of 1 second which suited the simulator course. A longer horizon could occasionally produce erroneous paths.

`Δa_error_weighting` was tunned to stop the vechile from osscilating about the reference speed. This value was found to increase the speed stability without effecting the vechiles ability to slow down around the tight corners

`ẟ_error_weighting, Δẟ_error_weighting` where tunned to find balance between the car getting 'speed wobbles' and being capable of turning tight corners when nessecary. As the cars speed increased so did the tendancy to start osscialting about the trajectory and so increasing these weights in the cost function limited the steerings ability to for rapid movements and therefore removed the osscilations. 

`min_velocity` and `max_velocity` were used to control the velocity of th vehicle. A ratio, based upon the steering angle, was used so the car would speed up on the straights and slow down around the corners. The following equations were used
```c++
ratio = steering_angle * 4 / 25°
referance_velocity = max_velocity + ratio*(min_velocity-max_velocity)
```
These final values (50 and 80) worked well in combination with the steering weights to produce a solution with the vechile cornering well and being quick on the straights




tuning of the control 
ask how to get the car to change speed
