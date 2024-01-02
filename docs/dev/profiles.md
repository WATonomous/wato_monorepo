# Modules
All autonomous software in the monorepo is divided into modules. Modules represent arbitrary groups of software components which share a similar purpose (eg. Detection, Behaviour Planning). Modules give you the freedom to specify which parts of the autonomy stack to run without the need to run the entire stack everytime.

You need to specify what modules you want to use by setting `ACTIVE_MODULES` in your `watod-config.sh`. For example, your `watod-config.sh` might include `ACTIVE_MODULES="carla tools matlab perception path_planning"`. 

modules are defined as `modules/docker-compose.<module>.yaml`. If you want to overwrite your current module and run the command against all modules, use `watod --all COMMAND`

```bash
# Space-delimited list of active modules to run, defined in docker-compose.yaml.
# Possible values:
#   - production    		:   configs for all containers required in production
#   - samples           :   starts sample ROS2 pubsub nodes
# Example override in watod-config.sh: 
#   ACTIVE_MODULES="samples production"

ACTIVE_MODULES=${ACTIVE_MODULES:-""}
```

## List of modules (TODO: write description)
Samples

Production

Perception

World Modeling

Action

Simulation

Data Streaming