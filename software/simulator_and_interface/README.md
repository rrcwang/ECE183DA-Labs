## paperbot Simulation Code

This is an implementation of the Extended Kalman Filter.

paperbot_simulation.py contains the following declarations:
    class State(xp,yp,theta,theta_dot)
    class StateEstimator(xp,yp,theta,theta_dot)


    For n samples of input of size 2 and n samples of measurement data of size 4,
we can iterate through the Kalman filter step as follows:

# Instructions
Import `paperbot_simulation.py` and call the following functions in order to run the state estimator. Call the file directly to run a simulated example.

## Initialize
`SE = StateEstimator()`

## Dynamics propagation
`SE.dynamics_propagation(input)`
Function returns a priori estimate and auxilliary information to account for distances from sensor to walls

## Measurement update
`SE.measurement_update(measurement)`
Function returns a posteriori estimate

## Get the most current estimate
`SE.get_state_val()`
Returns the current state estimate and a boolean who's truth value indicates whether the state estimate is a priori.
