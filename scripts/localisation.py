import numpy as np

state_matrix = np.matrix('0; 0; 0; 0; 0', dtype = 'float')
#process_covariance_matrix = np.matrix('4,4,1,1,1; 4,4,1,1,1; 1,1,0.25,0.25,0.25; 1,1,0.25,0.25,0.25; 1,1,0.25,0.25,0.25', dtype = 'float')
#process_covariance_matrix = numpy.zeros(5)
#zeros = np.matrix('0.01,0,0,0,0; 0,0.01,0,0,0; 0,0,0.01,0,0; 0,0,0,0.01,0; 0,0,0,0,0.01', dtype = 'float')
dt = 1
control_transition_matrix = np.matrix([[(dt*dt)/2.,0,0], [0,(dt*dt)/2.,0], [0,0,dt], [dt,0,0], [0,dt,0]], dtype = 'float')
error_input = control_transition_matrix * control_transition_matrix.T
process_covariance_matrix = error_input
count = 0






def run_kalman_filter():
    predict_next_state(state_matrix, process_covariance_matrix)

def update_measurement_in_state(state_matrix, process_covariance_matrix):
    global count
    # ALL THE MATRICES NEEDED FOR CALCULATIONS
    #control_values = np.matrix('1,1,45; 3,3,45; 6,6,45; 10,10,45; 15,15,45', dtype = 'float')
    control_values = np.matrix('1,0,45; 3,0,45; 6,0,45; 10,0,45; 15,0,45', dtype = 'float')
    #control_values = np.matrix('1,1,45; 2,2,45; 3,3,45; 4,4,45; 5,5,45; 6,6,45; 7,7,45; 8,8,45', dtype = 'float')
    #control_values = np.matrix('1,0,0; 2,0,0; 3,0,0; 4,0,0; 5,0,0; 6,0,0; 7,0,0; 8,0,0', dtype = 'float')
    #control_values = np.matrix('10,0,0; 20,0,0; 30,0,0; 40,0,0; 50,0,0; 60,0,0; 70,0,0; 80,0,0', dtype = 'float')
    observation_matrix = np.matrix('1,0,0,0,0; 0,1,0,0,0; 0,0,1,0,0', dtype = 'float')
    #measurement_error_covariance_matrix = np.matrix('100,100,30; 100,100,30; 30,30,9', dtype = 'float')
    measurement_error = np.matrix('0.1; 0.1; 0.03',dtype = 'float')
    measurement_error_covariance_matrix = measurement_error * measurement_error.T
    I = numpy.eye(5)
    measurement_matrix = control_values[count].T
    count += 1
    #kalman_gain = np.matrix()

    # CALCULATION OF KALMAN GAIN
    #print process_covariance_matrix
    kalman_gain = (process_covariance_matrix * observation_matrix.T) * np.linalg.inv((observation_matrix * process_covariance_matrix * observation_matrix.T) + measurement_error_covariance_matrix)

    # UPDATE THE STATE MATRIX WITH MEASUREMENT UPDATE
    state_matrix = state_matrix + (kalman_gain * (measurement_matrix - (observation_matrix * state_matrix)))
    
    # UPDATE THE PROCESS COVARIANCE MATRIX AFTER MEASUREMENT
    process_covariance_matrix = (I - kalman_gain * observation_matrix) * process_covariance_matrix

    print "state = ", state_matrix
    print "covariance = ", process_covariance_matrix
    print count
    if count >= 5:
        return

    predict_next_state(state_matrix, process_covariance_matrix)


def predict_next_state(state_matrix, process_covariance_matrix):
    dt = 1.
    # ALL THE MATRICES NEEDED FOR CALCULATIONS
    state_transition_matrix = np.matrix([[1,0,0,dt,0], [0,1,0,0,dt], [0,0,1,0,0], [0,0,0,1,0], [0,0,0,0,1]], dtype = 'float')
    #state_transition_matrix = np.matrix('1,0,0,dt,0; 0,1,0,0,dt; 0,0,1,0,0; 0,0,0,1,0; 0,0,0,0,1', dtype = 'float')
    control_variable_matrix = np.matrix([[1],[0],[0]])
    control_transition_matrix = np.matrix([[(dt*dt)/2.,0,0], [0,(dt*dt)/2.,0], [0,0,dt], [dt,0,0], [0,dt,0]], dtype = 'float')
    #control_transition_matrix = np.matrix('(dt^2.)/2.,0,0; 0,(dt^2.)/2.,0; 0,0,dt; dt,0,0; 0,dt,0', dtype = 'float')
    
    # UPDATE THE STATE MATRIX ACCORDING TO BEST PREDICTION
    state_matrix = state_transition_matrix * state_matrix #+ control_transition_matrix * control_variable_matrix

    # UPDATE THE PROCESS COVARIANCE MATRIX BASED ON INPUTS
    process_covariance_matrix = state_transition_matrix * process_covariance_matrix * state_transition_matrix.T
    
    # GOING TO THE INCORPORATION OF MEASUREMENT
    update_measurement_in_state(state_matrix, process_covariance_matrix)


run_kalman_filter()