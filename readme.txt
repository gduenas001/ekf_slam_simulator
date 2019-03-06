EKF SLAM Simulator


SLAM stands for Simultaneous Localization And Mapping and it has been an area of research for the past twenty years. This code partially implements a SLAM simulator based on the Extended Kalman Filter (EKF), which was the first available solution discovered in the 90’s. Many more efficient SLAM algorithms have been developed during the last 20 years, however EKF SLAM is still used for simple applications with low computing power and it is at the base of more advanced methods.


These are some of the most important files:
* MAIN: this is the script that you’ll run to start the simulation. After the configuration and initialization of variables the loop starts. Each loop does a prediction EKF step and whenever landmarks are detected, it also does an EKF update step. Each loop is divided into five parts: 
   * Control of the robot and computation of true robot location
   * Prediction step using the controls
   * Update step whenever landmarks are observed
   * Data storage for post-processing
   * Animations
* configfile: this file sets the configuration parameters for the simulation. Each parameter is explained in the code so feel free to play with them to observe different outputs (e.g. increase the observation noise until the robot gets lost or reduce the maximum steering angle until the robot can’t follow the trajectory).
* initialization: this script initializes some variables for future use. Not very interesting.
* compute_steering: this is a simple controller for the robot that steers the front wheel towards the next way point, the output of this functions are the actual steering angle and velocity that move the robot before noise is introduced.
* *vehicle_model: function that updates the pose of the actual robot (which would be unknown in real mission, but we know because it is a simulation)
* *predict: EKF prediction step. You will implement this function following the class notes.
* get_observations: measures the range and bearing to landmarks within sensor range.
* data_associate_known: assumes known correspondences between the extracted landmarks and the landmarks in the map that we are building. This is never the real case outside simulation, but it’s a good starting point.
* *data_associate: this substitutes data_associate_known, when the correspondences are not assumed known, which is what happens with real-world data. You will have to implement this function using the nearest neighbor algorithm explained in class.
* *update: EKF update step. You will implement this function.
* augment: this function adds some of the landmark detections that were not associated to previously mapped landmarks to the state vector. It only adds as new landmarks those detected landmarks that are far enough from other landmarks in the map. This is done in order to avoid creating multiple landmarks where there is only one landmark in the map.




There are many other functions in this code that store the data for post processing and create the simulations. Do not change those functions unless you know what you are doing.