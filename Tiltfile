############################ Tiltfile ############################
# Runs immediately when Tilt is called in the terminal. 
#
# It sets up .env variables, and docker-compose.yml configurations.
# Afterwards, it functions just like docker compose.
#
# Written in Starlark, a configuration-specialized dialect of
# Python.
##################################################################

# Load Tiltfile Configurations
backend_configs = read_yaml(tilt)
user_configs = read_yaml()