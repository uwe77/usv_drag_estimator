#!/usr/bin/env python3

from utils.simulator import Simulator
import rosbag
import matplotlib.pyplot as plt
import numpy as np
import glob
from scipy.optimize import minimize


def fit_force_axis(vel_array, force_array, fit_func, outlier_range=2.0):
    """
    clear NaN and do curve fitting
    - add outlier correction within range
    - add constraint: all parameters >= 0 (representing physical drag/damping)
    - ensure the model is convex (quadratic coefficient >= 0)
    - Parameter b is the offset(concider as waves force), R and RR are the quadratic coefficients
    """

    valid_mask = (~np.isnan(vel_array)) & (~np.isnan(force_array)) & (vel_array > 0.01)
    v = vel_array[valid_mask]
    f = force_array[valid_mask]

    # sorting by velocity
    sort_idx = np.argsort(v)
    v = v[sort_idx]
    f = f[sort_idx]

    # remove outlier
    def outlier_filter(y, threshold):
        y_new = y.copy()
        for i in range(1, len(y) - 1):
            local_avg = (y[i - 1] + y[i + 1]) / 2
            if abs(y[i] - local_avg) > threshold:
                y_new[i] = local_avg  # replace with local average
        return y_new

    f = outlier_filter(f, outlier_range)

    def biased_fit_func(v, b, R, RR):
        return b + fit_func(v, R, RR)

    # building MSE loss
    def loss(params):
        b, R, RR = params
        pred = biased_fit_func(v, b, R, RR)
        return np.mean((pred - f) ** 2)

    # Optimization using Boundry for convex space
    bounds = [(None, None), (0, None), (0, None)]  # R, RR ≥ 0
    result = minimize(
        loss,
        x0=[0.0, 1.0, 0.1],
        bounds=bounds,
        method='SLSQP',
        # constraints={"type": "ineq", "fun": max_force_constraint},
    )

    if result.success:
        b_out, R_opt, RR_opt = result.x
        return v, f, [b_out, R_opt, RR_opt]
    else:
        print("[ERROR] minimize failed:", result.message)
        return v, f, [0, 0, 0]
    
def fit_and_plot(sim, lin_vel_stack, ang_vel_stack, drag_force, drag_torque):
    axes = ['x', 'y', 'z']
    for i, axis in enumerate(axes):
        axis_obj = getattr(sim.axis_drag, axis)

        # Linear drag fit
        v, d_f, popt = fit_force_axis(
            lin_vel_stack[:, i],
            drag_force[:, i],
            axis_obj.get_linear_force
        )
        bias, axis_obj.R, axis_obj.RR = popt

        # only draw when there are enough data points
        if len(v) > 1:
            v_range = np.linspace(min(v), max(v), 100)
            pred = axis_obj.get_linear_force(v_range, axis_obj.R, axis_obj.RR)+bias
            plt.figure()
            plt.scatter(v, d_f, s=2, color='blue', label=f'{axis.upper()} Linear Data')
            plt.plot(v_range, pred, color='red', label='Fitted Curve')
            plt.xlabel("Linear Velocity (m/s)")
            plt.ylabel("Drag Force (N)")
            plt.title(f"Linear Drag (Convex Fit) - Axis {axis.upper()}")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()

        # Angular drag fit
        a, d_t, popt = fit_force_axis(
            ang_vel_stack[:, i],
            drag_torque[:, i],
            axis_obj.get_angular_force
        )
        bias, axis_obj.U, axis_obj.UU = popt

        if len(a) > 1:
            a_range = np.linspace(min(a), max(a), 100)
            pred_torque = axis_obj.get_angular_force(a_range, axis_obj.U, axis_obj.UU) + bias
            plt.figure()
            plt.scatter(a, d_t, s=2, color='green', label=f'{axis.upper()} Angular Data')
            plt.plot(a_range, pred_torque, color='orange', label='Fitted Curve')
            plt.xlabel("Angular Velocity (rad/s)")
            plt.ylabel("Drag Torque (Nm)")
            plt.title(f"Angular Drag Torque vs Angular Velocity - Axis {axis.upper()}")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()

def print_result(sim):
    output = f"""
Final Result:
x: 
    linear:  F = {sim.axis_drag.x.R:.4f} S = {sim.axis_drag.x.RR:.4f}
    angular: F = {sim.axis_drag.x.U:.4f} S = {sim.axis_drag.x.UU:.4f}
y: 
    linear:  F = {sim.axis_drag.y.R:.4f} S = {sim.axis_drag.y.RR:.4f}
    angular: F = {sim.axis_drag.y.U:.4f} S = {sim.axis_drag.y.UU:.4f}
z: 
    linear:  F = {sim.axis_drag.z.R:.4f} S = {sim.axis_drag.z.RR:.4f}
    angular: F = {sim.axis_drag.z.U:.4f} S = {sim.axis_drag.z.UU:.4f}
"""
    print(output)

def process_rosbag(bag_file, cmd, cmd_to_force_func, imu_topic, posestamp_topic, sim, thrust_pose):
    # compute thrust
    
    force_global, torque_global = cmd_to_force_func(cmd, thrust_pose)

    drag_force_list = []
    drag_torque_list = []
    linear_vel_list = []
    angular_vel_list = []

    bag = rosbag.Bag(bag_file)
    for topic, msg, _ in bag.read_messages(topics=[imu_topic, posestamp_topic]):
        if topic == imu_topic:
            sim.veh.imu_cb(msg)
        elif topic == posestamp_topic:
            sim.veh.posestamp_cb(msg)

        veh_force = np.array([
            sim.veh.axis_force.x.get_linear_force(sim.veh.local_acc[0]),
            sim.veh.axis_force.y.get_linear_force(sim.veh.local_acc[1]),
            sim.veh.axis_force.z.get_linear_force(sim.veh.local_acc[2]),
        ])
        veh_torque = np.array([
            sim.veh.axis_force.x.get_angular_force(sim.veh.angular_vel[0]),
            sim.veh.axis_force.y.get_angular_force(sim.veh.angular_vel[1]),
            sim.veh.axis_force.z.get_angular_force(sim.veh.angular_vel[2]),
        ])

        df = np.abs(force_global - veh_force)
        dt = np.abs(torque_global - veh_torque)

        drag_force_list.append(df)
        drag_torque_list.append(dt)
        linear_vel_list.append(np.abs(sim.veh.local_vel))
        angular_vel_list.append(np.abs(sim.veh.angular_vel))
    bag.close()

    return (np.array(drag_force_list),
            np.array(drag_torque_list),
            np.array(linear_vel_list),
            np.array(angular_vel_list))

def run_drag_optimizer(config):
    sim = Simulator(config["veh_mass"], config["veh_inertia"])
    drag_f_all, drag_t_all, lin_v_all, ang_v_all = [], [], [], []

    for i, bag_path in enumerate(config["bag_list"]):
        df, dt, lv, av = process_rosbag(
            bag_file=bag_path,
            cmd=config["cmd_list"][i],
            cmd_to_force_func=config['cmd_to_force_func'],
            imu_topic=config["imu_topic"],
            posestamp_topic=config["posestamp_topic"],
            sim=sim,
            thrust_pose=config["thrust_pose"]
        )
        drag_f_all.append(df)
        drag_t_all.append(dt)
        lin_v_all.append(lv)
        ang_v_all.append(av)

    # Combine and fit
    lin_vel_stack = np.vstack(lin_v_all)
    ang_vel_stack = np.vstack(ang_v_all)
    drag_force = np.vstack(drag_f_all)
    drag_torque = np.vstack(drag_t_all)

    fit_and_plot(sim, lin_vel_stack, ang_vel_stack, drag_force, drag_torque)
    print_result(sim)

if __name__ == "__main__":
    bag_ids = ['1235', '1251', '1253', '1247', '1248']
    bags = [glob.glob(f'../bags/0402_{i}/*.bag')[0] for i in bag_ids]

    def cmd_to_force(cmd=np.zeros(2), thrust_pose=np.zeros(3)):
        
        def force_to_N(x, hp=150):
            operator = lambda x: 1 if x > 0 else -1
            u = lambda x: 0.0321 + -0.5263*x + 2.1166*x**2 + -0.6257*x**3
            return operator(x)*u(abs(x)) * hp

        def normalized_to_angular(a):
            return a*np.pi*5/72
        
        thrust = force_to_N(cmd[0])
        thrust_dir = normalized_to_angular(cmd[1])
        thrust_vec = np.array([
            thrust * np.cos(thrust_dir),
            thrust * np.sin(thrust_dir),
            0.0
        ])
        torque_vec = np.cross(thrust_pose, thrust_vec)
        return thrust_vec, torque_vec
    
    veh_mass = 2600.0
    veh_inertia = np.array([2000.0, 10000.0, 10000.0]) # raw pitch yaw
    unbalance_masses = np.array([230, 132])
    unbalance_masses_pose = np.array([
        [-3.822,  0.00, -0.75],
        [-3.822, -0.92, -0.75],
    ])
    veh_mass += np.sum(unbalance_masses)
    # roll inertia (x-axis): sum(m * (y^2 + z^2))
    roll_delta = np.sum(unbalance_masses * (unbalance_masses_pose[:, 1]**2 + unbalance_masses_pose[:, 2]**2))
    # pitch inertia (y-axis): sum(m * (x^2 + z^2))
    pitch_delta = np.sum(unbalance_masses * (unbalance_masses_pose[:, 0]**2 + unbalance_masses_pose[:, 2]**2))
    # yaw inertia (z-axis): sum(m * (x^2 + y^2))
    yaw_delta = np.sum(unbalance_masses * (unbalance_masses_pose[:, 0]**2 + unbalance_masses_pose[:, 1]**2))
    veh_inertia += np.array([roll_delta, pitch_delta, yaw_delta])
    
    config = {
        "bag_list": bags,
        "cmd_list": np.array([
            [0.5, 0.0],
            [0.6, 0.0],
            [0.7, 0.0],
            [0.5, 1.0],
            [0.5, -1.0],
        ]),
        "cmd_to_force_func": cmd_to_force,
        "imu_topic": "/mavros/imu/data",
        "posestamp_topic": "/mavros/local_position/pose",
        "veh_mass": veh_mass,
        "veh_inertia": veh_inertia,
        "thrust_pose": np.array([-3.822, 0.0, -0.75])
    }

    run_drag_optimizer(config)