# Vehicle-Model-Predictive-Controller

## MPC Controller Code Framework
Implement a Model Predictive Control (MPC) based vehicle control system, primarily for vehicles in the CARLA autonomous driving simulator. MPC is an optimization framework used for control and planning, capable of considering the vehicle's dynamic characteristics and predicting future states.

### MPC Controller Code Workflow
1. **Initialization and Configuration**
   - Prediction horizon (Np) and control horizon (Nc): These determine how far into the future the controller looks and how many control actions are planned within this time frame.
   - Time step (dt): The interval between control and prediction steps, affecting the accuracy of model predictions and computational load.
   - Vehicle dynamic parameters (Lf, etc.): These parameters describe the vehicle dynamics model, foundational for predictive modeling.
   - Initialize variables: Including the vehicle's current state (position, speed, direction, etc.) and the coefficients of the target path (to describe the desired polynomial path).

2. **Setting Up the Optimization Problem**
   - Objective function: Construct an objective function to minimize path deviation (CTE), heading deviation (Epsi), and speed deviation, while also considering reducing actuator usage to smooth the control process.
   - Constraints: Including the vehicle dynamics model constraints to ensure predicted states follow physical laws; actuator limitations, such as steering angle and acceleration bounds; and constraints ensuring the optimization problem's initial state matches the current vehicle state.

3. **Solving the Optimization Problem**
   - Solver selection: Choose an optimization solver like IPOPT, capable of handling constrained nonlinear optimization problems.
   - Compute the control input sequence: The solver calculates the optimal control input sequence based on the objective function and constraints.
   - Select control input: Extract the first set of control inputs (typically including steering angle and acceleration) from the solution for immediate execution.

4. **Execute Control and Update State**
   - Execute control: Apply the first set of control inputs to the vehicle, affecting its actual state.
   - State update: Update the vehicle's state based on the executed control inputs and the vehicle model.
   - Feedback loop: In each control cycle, reset the optimization problem based on the new vehicle state and target path, then repeat the process of solving the optimization problem and executing control, achieving closed-loop control.

### FG_eval Class
This class defines the optimization problem, including the cost function and the dynamics model constraints. IPOPT (a solver for large nonlinear optimization problems) calls this class. FG_eval utilizes the CppAD library, a C++ library for automatic differentiation, enabling gradient and Hessian matrix calculation during the optimization process, essential for most nonlinear optimization algorithms.

**Constructor**
The FG_eval constructor receives the current vehicle state, polynomial path coefficients, target speed, various weight factors, prediction and control horizon lengths, control step length, vehicle parameters, and the state of the previous control action. It initializes cost function weights, control and prediction horizon lengths, control step length, etc., key parameters for MPC adjustment.

**operator() Function**
This function is the core, defining the entire optimization problem. Smoothness of control actions is also considered as part of the cost. It includes constraints from the dynamics model, ensuring vehicle state continuity and physical plausibility.

**1. Cost Function (fg[0])**
The fg array represents a vector containing the cost function and all constraints. fg[0] is the objective function to be minimized. It's a cumulative total cost, including:

> Tracking objective function
> - Square of the path deviation (CTE, cross-track error) multiplied by a weight (cte_weight).
> - Square of the heading deviation (Epsi, orientation error) multiplied by a weight (epsi_weight).
> - Square of the speed deviation (difference from target speed ref_v) multiplied by a weight (v_weight).

The purpose of these cost items is to make the vehicle follow the reference trajectory as closely as possible and travel at the desired speed.

> Comfort objective function
> - Square of the steering angle multiplied by a weight (steer_actuator_cost_weight_fg), penalizing large steering actions.
> - Square of the longitudinal acceleration multiplied by a weight (acc_actuator_cost_weight_fg), penalizing large acceleration or deceleration actions.

The purpose of these cost items is to reduce the use of actuators (steering and throttle in cars) because excessive actuator usage can lead to abrupt vehicle behavior and increased wear.

> Safety objective function
> - Smoothness of control actions
> - Square of the change between consecutive steering actions multiplied by a weight (change_steer_cost_weight).
> - Square of the change between consecutive acceleration actions multiplied by a weight (change_accel_cost_weight).

The purpose of these cost items is to encourage smoothness in actions, reducing abrupt changes between consecutive control actions for smooth driving.

**2. Initialization of Equality Constraints (fg[1 + ...], t=0)**
Match the optimization variables in vars with the actual vehicle state. This effectively tells the optimizer that our starting point is the current state of the vehicle, and optimization should proceed from this state.

Each fg[1 + ...] represents a specific constraint in the optimization problem, starting from index 1 since index 0 stores the cost function value. For each vehicle state variable:

> fg[1 + x_start] constraints for the vehicle's global X coordinate. 
> fg[1 + y_start] constraints for the vehicle's global Y coordinate. 
> fg[1 + psi_start] constraints for the vehicle's heading angle. 
> fg[1 + v_longitudinal_start] constraints for the vehicle's longitudinal speed. 
> fg[1 + v_lateral_start] constraints for the vehicle's lateral speed. 
> fg[1 + yaw_rate_start] constraints for the vehicle's yaw rate. 
> fg[1 + cte_start] constraints for the vehicle's lateral tracking error (CTE). 
> fg[1 + epsi_start] constraints for the vehicle's heading error (Epsi).

Here, the corresponding state variables in vars are directly assigned to the constraint variables in fg, enforcing these initial state constraints to match the vehicle's current state. This ensures that the initial conditions are correct for the optimization problem solving process, and subsequent optimization steps (future states and control actions in the optimization variables) are based on this actual starting point.

**3. Predict model, t(1, Np)**

 1. Enter a for loop.
 2. Calculate the reference path and reference heading angle (psi_des_0), crucial in Model Predictive Control (MPC). MPC aims to guide the vehicle along a predefined path while minimizing heading and path deviations.
     - Reference path (f_0) calculates the y value on the reference path for a given x position (x_0) using a polynomial function defined by coefficients coeffs. This polynomial can be of any degree, as determined by the length of coeffs, with polynomials up to the fifth degree in this example.
     - Reference heading angle (psi_des_0) calculates the tangent angle of the reference path at x_0, representing the theoretical heading angle the vehicle should maintain to follow the reference path. This angle is derived by calculating the derivative of the polynomial f_0 at x_0 and taking its arctan. The derivative represents the slope of the reference path at x_0, with the arctan providing the angle of this slope relative to the horizontal axis, i.e., the reference heading angle.
3. Calculate the vehicle's state variables at times t and t+1 (e.g., x_0, y_0 for time t and x_1, y_1 for time t+1). These variables represent the vehicle's state at time t - 1, used to predict the state at time t.
4. Control input variables: front_wheel_angle_0 for the front wheel angle and longitudinal_acceleration_0 for longitudinal acceleration. These variables are the control quantities adjusted by the optimizer to achieve the predicted vehicle state at time t.
5. Forward iterate the dynamics model, predicting the vehicle's future motion based on the calculated state variables and control input variables. Then satisfy the equation: ***The predicted lateral position's calculated value during the optimization process must equal the value calculated from the vehicle dynamics model***. This is achieved by setting fg[1 + .... + t] equal to the difference between these two predicted values.

> x_1 represents the predicted lateral position x at time step t+1 in the optimization variables.
> x_0 + ...the latter term represents the predicted lateral position x at time step t+1 calculated from the vehicle dynamics model.

### mpc_controller Class

 1. This is the main class for the MPC controller, encapsulating the logic for solving the MPC problem. The Solve method receives the current vehicle state, coefficients of the target path, and other MPC parameters, then configures and solves the MPC optimization problem.
 2. Key is the call to the CppAD::ipopt::solve function, using the IPOPT algorithm to find control inputs that minimize the cost function.
 3. The solver's first control input (steering angle and acceleration) is used to control the vehicle.
 4. In addition to the optimal control commands, it also returns predicted vehicle states, often used for debugging or visualizing the MPC path in simulators.

**Parameter Initialization:**

 1. Nc and Np represent the lengths of the control and prediction horizons, respectively. Typically, the control horizon is less than or equal to the prediction horizon.
 2. Variables like x_start, y_start, psi_start define the indices of various state variables in the optimization variable vector.
 3. Variables such as a_lateral, old_steer_value, old_throttle_value are used to initialize vehicle model parameters or previous control states.
 
**Define Optimization Variables (vars):**
Initialize all optimization variables (including vehicle state variables and control input variables) to zero and set appropriate upper and lower bounds. These variables will be adjusted during the optimization process to minimize the cost function.

**Define Inequality Constraints**
Limits on actuator outputs, such as steering angle and acceleration bounds.
Limits on state variables, such as maximum and minimum vehicle speeds.

**Define Initial State Constraints (constraints_lower_bounds and constraints_upper_bounds):**
Set initial state constraints for the optimization problem to ensure the solver (optimization algorithm) starts from the current actual vehicle state. This is done by setting equal upper and lower bounds on the part of the optimization variable (vars) representing the vehicle's initial state.

**Construct Objective and Constraints Function (FG_eval):**
Utilize the FG_eval class to create an instance of an FG_eval object, defining the objective function and constraints for the Model Predictive Control (MPC) problem. FG_eval calculates the total cost for given optimization variables and evaluates whether all constraints are satisfied.

**Configure and Solve the Optimization Problem:**
Configure IPOPT solver parameters, such as verbosity level and computational optimizations.
Call the CppAD::ipopt::solve function to solve the optimization problem. The solver calculates optimal control inputs based on the objective and constraint functions.

> print_level: Controls the verbosity level of output information.
> Sparse: Indicates to the solver that the constraints and Jacobian matrix are sparse, significantly improving solving speed.
> max_cpu_time: Limits the maximum CPU time for the solver.

**Process Solution Results:**

 1. The result vector stores the optimization results, first extracting control input values, namely the values for steering angle (front_wheel_angle) and acceleration (longitudinal_acceleration). These two control inputs are directly used in the vehicle's control system to guide immediate driving behavior.
 2. Subsequent loops extract predicted future vehicle positions from the optimization results using solution.x[x_start + i] and solution.x[y_start + i], with Np - 1 indicating all predicted positions from the current timestep to the end of the prediction horizon. This information is highly useful for visualizing the MPC-predicted trajectory, helping to understand how the control strategy guides the vehicle closer to the target path over time.

**ROS Node Configuration**

 1. Parameter Configuration and Environment Setup: Use the ROS parameter server to get MPC controller configuration parameters, such as vehicle reference speed, control horizon length, weights, etc., and initialize the MPC controller based on these parameters.
 2. Data Subscription and Processing: Subscribe to vehicle location, speed, and IMU data to get real-time vehicle state information, which is used as input for the MPC algorithm.
 3. Control Command Calculation: Utilize the MPC algorithm defined in mpc_controller.cpp to calculate optimal control commands for path tracking based on the vehicle's real-time state and target path.
 4. Control Command Publication: Publish the calculated control commands to the vehicle, directing it to drive along the predetermined path.
 5. Path Visualization: For ease of observation and debugging, the code also includes publishing the MPC-calculated path and reference path through ROS topics for visualization in tools like RViz.


## Getting Started

### Requirements
- Ubuntu 20.04
- ROS foxy
- Additional installation of carla-ros-bridge packages and other packages is required.
- Carla 0.9.13

1. **Enter Workspace Folder**
    Navigate to the workspace folder where you want to build the project.

    ```bash
    cd your_workspace_path
    source source_env.sh
    ```

2. **Build the Project**
    Use `catkin_make` to build the ROS packages.

    ```bash
    colcon build
    ```

3. **Source the Setup Script**
    To configure the environment variables for the project, source the setup script.

    ```bash
    source ./install/setup.bash
    ```
4. **Run Carla simulator**
     Navigate to the Carla simulator directory and run the following command.

    ```bash
    ./CarlaUE4.sh
    ```
5. **Launching the Project**
     Navigate to the ROS workspace (carla-ros-bridge) and follow these steps in order:

    a. Launch the Carla ROS bridge ego vehicle visualization:
    ```bash
    ros2 launch carla_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
    ```

    b. Run the MPC controller node:

    ```bash
    ros2 launch carla_mpc_controller mpc_launch.py
    ```
