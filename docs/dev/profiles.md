# Profiles
All autonomous software in the monorepo is divided into profiles. Profiles represent arbitrary groups of software compenents which share a similar purpose (eg. Detection, Behaviour Planning). Profiles give you the freedom to specify which parts of the autonomy stack to run without the need to run the entire stack everytime.

You need to specify what profiles you want to use by setting `ACTIVE_PROFILES` in your `watod-config.sh`. For example, your `watod-config.sh` might include `ACTIVE_PROFILES="carla tools matlab perception path_planning"`. 

Profiles are defined as `profiles/docker-compose.<PROFILE>.yaml`. If you want to overwrite your current profile and run the command against all profiles, use `watod --all COMMAND`

```bash
# Space-delimited list of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - production    		:   configs for all containers required in production
#   - samples           :   starts sample ROS2 pubsub nodes
# Example override in watod-config.sh: 
#   ACTIVE_PROFILES="samples production"

ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}
```

## List of Profiles
To be written