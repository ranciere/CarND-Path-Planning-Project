# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

The model is based on the course's FAQ solution. The code is reimplemented and refactored to meets good coding practice. The main difference is that the algorithm prefers the middle lane to be able to choose both side lanes in case of barrier. 

## Overall Design
The applicaiton logic is in the `Controller` class (`controller.h/cpp`), and all of the states of the algoritm are stored in this class as members (current lane and velocity). The constructor of the class is called with the map variables and the code will use these for calculations.
During the running the algorithm executes three steps:
1. Checking surroundings (`controller.cpp`: 5-67 lines)
2. Evaluting State Machine (`controller.cpp`: 69-104 lines)
3. Calculating Trajectory (`controller.cpp`: 106-193 lines)

All of the steps use a helper class (`Data`) to keep the number of input/output parameters low and provide an opportunity for further development without big refactoring.

### Checking surroundings
During the update the code maintains three boolean state based on the surrounding cars (eg. sensor fusion data):
1. `too_close`: *True*, If there is a car in front of us within 20 units (`controllers.cpp`: 45-51 lines).
2. `is_car_left`: *True*, if there is a car in the left lane within 40 units (`controllers.cpp`: 52-58 lines).
3. `is_car_right`: *True*, if there is a car in the right lane within 40 units  (`controllers.cpp`: 59-65 lines).

These states are stored in the `Data` variable.

### State Machine
The State Machine is quite simple:
1. `too_close` == *True* then (`controllers.cpp`: 74-88 lines)
  * it will change lane if it is possible (depends on `is_car_left/right`)
  * othwerwise decrease the velocity
2. `too_close` == is *False* then (`controllers.cpp`: 90-103 lines)
  * it will increase the velocity if it is smaller then the target (49.5 MPH)
  * it will switch to the center lane if it is free

### Trajectory generation
The algorithm of the trajectory generation:
1. Creating two X/Y vectors from the last two points of the previous path (`previous_path_x/y`) and three new points (30, 60 and 90 in the expected lane). This guarantees the expected lane continuity.
2. This X/Y coordinates will be the base points of athe spline fitting. The second derivates of splines are continuous so it will guarantee that there won't be jerks.
3. The `previous_path_x/y` is copied to `next_x/y_vals` vector (keeping these coordinates)
4. Then next points are interpolated by the spline and appended to the `next_x/y_vals` vectors. During the interpolation the expected velocity is taken into account.
