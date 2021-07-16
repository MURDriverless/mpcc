# Model Predictive Contouring Control

### Disclaimer
This branch is a ROS implementation of [Alex Liniger's MPCC](https://github.com/alexliniger/MPCC). It is using the version from the fullsize branch.

### Run in ROS
1. Clone this branch as a separate package to build
2. `catkin build` then `source devel/setup.bash`
3. `rosrun alexmpcc alexmpcc`

### To Do
1. Update model parameters to fit 21 Car. Currently only car dimension is put in.
2. Retune controller to work in 21 Car.
3. Make it work in MURsim.
