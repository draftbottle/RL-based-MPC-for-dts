# RLMPC - Reinforcement Learning Model Predictive Control

This MATLAB implementation demonstrates the RLMPC (Reinforcement Learning Model Predictive Control) framework, which combines Reinforcement Learning (RL) with Model Predictive Control (MPC) through Policy Iteration (PI).

## Overview

The RLMPC framework integrates RL's ability to learn optimal policies from data with MPC's optimization-based control strategy. The core innovation is using RL to learn an optimal terminal cost function for MPC, which improves control performance without requiring explicit terminal constraints or conditions.


### Key Features

- **Policy Iteration Framework**: Alternates between policy generation (MPC) and policy evaluation (RL)
- **Online Learning**: Learns the terminal cost function while controlling the system
- **Polynomial Basis Functions**: Approximates the value function using polynomial features
- **System Support**: Implementations for both linear systems and nonlinear non-holonomic vehicle systems
- **Comparative Analysis**: Performance benchmarks against standard LQR and MPC approaches

## Algorithm

The RLMPC algorithm implements the following key steps:

1. **Initialization**: Start with a zero terminal cost function
2. **Policy Generation**: Solve MPC optimization with current value function as terminal cost
3. **Policy Evaluation**: Sample states and evaluate current policy to learn value function 
4. **Policy Improvement**: Update value function weights and use as new terminal cost
5. **Iterate**: Repeat until convergence or for a fixed number of iterations

## System Requirements

- MATLAB R2019b or later
- Optimization Toolbox
- Statistics and Machine Learning Toolbox (optional, for some advanced features)

## Repository Structure

- `main_RLMPC.m`: Main script to run RLMPC simulations
- `LinearSystem.m`: Class implementing linear system dynamics and costs
- `NonlinearVehicle.m`: Class implementing nonlinear non-holonomic vehicle dynamics
- `ValueFunctionApproximator.m`: Class for polynomial basis function approximation
- `MPC.m`: Model Predictive Control implementation with learned terminal cost
- `CustomTerminalCostMPC.m`: MPC implementation with manually specified terminal cost
- `LQRController.m`: Linear Quadratic Regulator controller for comparison
- `simulate_system.m`: Function to simulate systems with various controllers
- Visualization utilities:
  - `plot_comparison_linear.m`: Visualization for linear system results
  - `plot_comparison_nonlinear.m`: Visualization for nonlinear system results
  - `plot_comparison_nonlinear_extended.m`: Enhanced visualization for nonlinear systems

## Usage

To run the simulations:

```matlab
% Run the main script
main_RLMPC
```

You can select the system type by modifying the `system_type` variable in the main script:

```matlab
system_type = 'linear';    % For linear system
% OR
system_type = 'nonlinear'; % For non-holonomic vehicle
```

## Configuration Parameters

### Linear System
- System matrices: A = [1, 0.5; 0.1, 0.5], B = [1; 0]
- Cost matrices: Q = eye(2), R = 0.5
- Prediction horizon: N = 3
- Polynomial basis order: 2
- Learning rate (alpha): 1e-2
- Regularization (lambda): 0.01

### Nonlinear Vehicle System
- State: [x, y, θ] (position and orientation)
- Input: [v, ω] (linear and angular velocity)
- Input constraints: |v| ≤ 1, |ω| ≤ 4
- State constraints: x, y bounded within [0, 2]
- Prediction horizon: N = 5 (RLMPC), N_mpc = 20 (traditional MPC)
- Polynomial basis order: 4 (35 features)
- Learning rate (alpha): 2e-3
- Regularization (lambda): 0.01

## Implementation Details

### Learning Process
The implementation uses a hybrid approach for learning:
- **Online Learning**: The main loop implements online learning where the value function is updated at each time step as the system evolves
- **Stochastic Gradient Descent**: SGD is used to update the value function weights
- **Gradient Clipping**: Prevents exploding gradients for stable learning
- **L2 Regularization**: Prevents overfitting and stabilizes learning

### MPC Implementation
The MPC controller is implemented with several features:
- **Multiple Solvers**: Attempts different algorithms (SQP, interior-point, active-set) if optimization fails
- **Warm Starting**: Uses previous solution to initialize new optimization
- **Robust Constraints**: Handles both state and input constraints
- **Adaptive Terminal Cost**: Incorporates the learned value function as terminal cost

## Experimental Results

The implementation produces several visualizations:
- State trajectories
- Control inputs
- Accumulated costs
- Policy iteration convergence
- Weight convergence during learning
![image](https://github.com/user-attachments/assets/cd22c41d-de86-47bf-b812-0002d1ee8bbe)
![image](https://github.com/user-attachments/assets/f67f5a30-45d3-4b03-9cfd-d15304e60a09)
![image](https://github.com/user-attachments/assets/2bb0f49d-c23f-4b60-8af9-d2428c95f604)
![image](https://github.com/user-attachments/assets/cbba04b0-5883-491d-a244-7cd2a416ff16)
Performance Metrics (Nonlinear Vehicle with MPC-TC):
- RLMPC Total Cost: 38.3546
- MPC-Long Total Cost: 37.7445
- MPC w/o TC Total Cost: 41.2104
- MPC-TC Total Cost: 38.3546
- RLMPC vs MPC-Long improvement: -1.6164%
- RLMPC vs MPC w/o TC improvement: 6.9299%
- RLMPC vs MPC-TC improvement: 0%
- MPC-TC vs MPC w/o TC improvement: 6.9299%

### Linear System Results
For linear systems, RLMPC is compared with:
- Standard LQR controller
- MPC without terminal cost
- MPC with manually tuned terminal cost

### Nonlinear System Results
For nonlinear systems, RLMPC is compared with:
- Long-horizon MPC (N_mpc = 20)
- Short-horizon MPC without terminal cost (N = 5)
- MPC with manually tuned terminal cost (N = 5)

## Citation

 this code is based on "Reinforcement Learning-Based Model Predictive Control for Discrete-Time Systems" -IEEE TRANSACTIONS ON NEURAL NETWORKS AND LEARNING SYSTEMS, VOL. 35, NO. 3, MARCH 2024.
 
 my code is a simple test for the paper ,so ,there might be so many faults.  If anyone find the mistakes in this code or some improvement for it ,iy's my pleasure for you to tell me or change it.


## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=lmcggg/RL-based-MPC-for-dts&type=Date)](https://www.star-history.com/#lmcggg/RL-based-MPC-for-dts&Date)
```



## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- This implementation is based on the RLMPC framework described in relevant literature
- Special thanks to contributors and researchers in the field of learning-based control 
