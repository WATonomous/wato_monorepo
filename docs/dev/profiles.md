# Profiles
You need to specify what profiles you want to use by setting `ACTIVE_PROFILES` in your `dev_config.local.sh`. For example, your `dev_config.local.sh` might include `ACTIVE_PROFILES="carla tools matlab perception path_planning"`. 

Profiles are defined as `profiles/docker-compose.<PROFILE>.yaml`. If you want to overwrite your current profile and run the command against all profiles, use `watod2 --all COMMAND`

```bash
from dev_config.sh
# Space-delimited list of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - production    		:   configs for all containers required in production
#   - samples           :   starts sample ROS2 pubsub nodes
# Example override in dev_config.local.sh: 
#   ACTIVE_PROFILES="samples production"

ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}
```