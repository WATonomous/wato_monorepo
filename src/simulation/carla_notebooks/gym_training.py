import gymnasium as gym
import gym_carla
from gym_carla.carla_env import CarlaEnv
import carla

from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import DQN, SAC

# parameters for the gym_carla environment
params = {
'display': False,
'number_of_vehicles': 0,
'number_of_walkers': 0,
'display_size': 256,  # screen size of bird-eye render
'max_past_step': 1,  # the number of past steps to draw
'dt': 0.07,  # time interval between two frames
'discrete': True,  # whether to use discrete control space
# 'discrete_acc': [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0],  # discrete value of acceleration (postive) and brake (negative)
# 'discrete_steer': [-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0],  # discrete value of steering
'discrete_acc': [-0.2, -0.1, 0.0, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
'discrete_steer': [-0.9, -0.6, -0.3, -0.15, 0.0, 0.15, 0.3, 0.6, 0.9],
'continuous_accel_range': [-1.0, 1.0],  # continuous acceleration range
'continuous_steer_range': [-1.0, 1.0],  # continuous steering angle range
'ego_vehicle_filter': 'vehicle.mini.cooper_s_2021',  # filter for defining ego vehicle
'port': 2000,  # connection port
'town': 'Town05',  # which town to simulate
'task_mode': 'random',  # mode of the task, [random, roundabout (only for Town03)]
    'max_time_episode': 10000,  # maximum timesteps per episode
'max_waypt': 7,  # maximum number of waypoints
'obs_range': 80,  # observation range (meter)
'lidar_bin': 0.125,  # bin size of lidar sensor (meter)
'd_behind': 12,  # distance behind the ego vehicle (meter)
'out_lane_thres': 1.5,  # threshold for out of lane
'desired_speed': 10,  # desired speed (m/s)
'max_ego_spawn_times': 200,  # maximum times to spawn ego vehicle
'display_route': True,  # whether to render the desired route
'pixor_size': 64,  # size of the pixor labels
'pixor': False,  # whether to output PIXOR observation
}

# Set gym-carla environment
env = CarlaEnv(params)
obs = env.reset()

# load model
model = DQN.load("dqn_carla_test", env=env, device='cuda')

# model = DQN("MultiInputPolicy", env, tensorboard_log='./dqn_carla_tensorboard',learning_rate=1e-4, buffer_size=10000, learning_starts=100, batch_size=36, train_freq=100, exploration_fraction=0.3, exploration_final_eps=0.2, optimize_memory_usage=False, device="cuda")

TOTAL_TIMESTEPS = 50000

model.learn(total_timesteps=TOTAL_TIMESTEPS, tb_log_name="DQN5")

model.save('dqn_carla_test')


