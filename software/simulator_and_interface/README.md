## paperbot Simulation Code

This is an implementation of the Extended Kalman Filter.

paperbot_simulation.py contains the following declarations:
    class State(xp,yp,theta,theta_dot)
    class StateEstimator(xp,yp,theta,theta_dot)


    For n samples of input of size 2 and n samples of measurement data of size 4,
we can iterate through the Kalman filter step as follows:

# Initialize
SE = StateEstimator()

# Dynamics propagation, function returns a priori estimate and numbers to help deal with the walls
SE.dynamics_propagation(input)
# Measurement update, function returnsa posteriori estimate
SE.measurement_update(measurement)

Repeat this data for the next
SE.get_state_val()
