# Model Predictive Contouring Control

# Disclaimer
This branch is a ROS implementation of [Alex Liniger's MPCC](https://github.com/alexliniger/MPCC). It is using the version from the fullsize branch, commit 302e12a on 15 Jun 2021.

## Run in ROS, with simple sim
1. Clone this package as a separate package to build
2. Run `sudo ./install.sh` to install dependencies into External folder. If it says file not found, try `sudo chmod +x install.sh`
3. Change directory to reflect the directory you put the files in for `main.cpp` and `Params/config.json`
4. `catkin build` then `source devel/setup.bash`
5. `roslaunch alexmpcc alexmpcc`

### Issues with package building
You may want to try and build the package from scratch, follow these steps.
1. Clone Alex Liniger's MPCC from fullsize branch.
2. Follow his instructions to build MPCC, verify if it works by running `./MPCC`
3. Arrange the folders in the ROS template (all files in src, CMakeList.txt is at same level as src)
4. Clone this repo, replace everything except for External.

## To Do Controller
1. Actual "discretize then linearize" formulation.
2. Include front tyre force constraint.
3. Include aero force maybe?
4. Do RIT stuff

## To Do ROS
1. Update model parameters to fit 21Car. Currently using base car from Alex Liniger's fullsize branch.
2. Retune controller to work in 21Car.
3. Make it work in MURsim.

