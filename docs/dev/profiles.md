# Profiles
All autonomous software in the monorepo is divided into modules. Profiles represent arbitrary groups of software compenents which share a similar purpose (eg. Detection, Behaviour Planning). Profiles give you the freedom to specify which parts of the autonomy stack to run without the need to run the entire stack everytime.

You need to specify what modules you want to use by setting `ACTIVE_MODULES` in your `watod-config.sh`. For example, your `watod-config.sh` might include `ACTIVE_MODULES="carla tools matlab perception path_planning"`. 

Profiles are defined as `modules/docker-compose.<PROFILE>.yaml`. If you want to overwrite your current profile and run the command against all modules, use `watod --all COMMAND`

```bash
# Space-delimited list of active modules to run, defined in docker-compose.yaml.
# Possible values:
#   - production    		:   configs for all containers required in production
#   - samples           :   starts sample ROS2 pubsub nodes
# Example override in watod-config.sh: 
#   ACTIVE_MODULES="samples production"

ACTIVE_MODULES=${ACTIVE_MODULES:-""}
```

## List of Profiles
To be written