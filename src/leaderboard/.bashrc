export CARLA_ROOT="/home/docker"
export SCENARIO_RUNNER_ROOT="/home/docker/scenario-runner"
export LEADERBOARD_ROOT="/home/docker/leaderboard"
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}
