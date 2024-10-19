import numpy as np
from scipy.optimize import minimize
from uuv_mission.controller import PDController, PIDController
from uuv_mission.dynamic import Submarine, ClosedLoop, Mission

def cost_function(params, controller_type, mission_csv_path):
    kp, ki, kd = params
    sub = Submarine()
    
    if controller_type == 'PD':
        controller = PDController(kp, kd)
    elif controller_type == 'PID':
        controller = PIDController(kp, ki, kd)
    else:
        raise ValueError("Invalid controller type")

    closed_loop = ClosedLoop(sub, controller)
    mission = Mission.from_csv(mission_csv_path)
    trajectory = closed_loop.simulate_with_random_disturbances(mission)
    
    # Define your cost function here. For example, the sum of squared errors
    errors = trajectory.errors
    cost = np.sum(np.square(errors))
    
    return cost

def optimize_controller(controller_type, mission_csv_path):
    initial_guess = [0.1, 0.1, 0.1]  # Initial guess for kp, ki, kd
    bounds = [(0, 10), (0, 10), (0, 10)]  # Bounds for kp, ki, kd

    result = minimize(cost_function, initial_guess, args=(controller_type, mission_csv_path), bounds=bounds)
    
    if result.success:
        optimal_params = result.x
        print(f"Optimal parameters for {controller_type} controller: kp={optimal_params[0]}, ki={optimal_params[1]}, kd={optimal_params[2]}")
        return optimal_params
    else:
        raise ValueError("Optimization failed")

if __name__ == "__main__":
    mission_csv_path = "C:/Users/lgbus/B1/b1-coding-practical-mt24/data/mission.csv"
    optimize_controller('PD', mission_csv_path)
    optimize_controller('PID', mission_csv_path)