import carla
import os
import random

## SETUP ##
client_name = os.environ.get("CLIENT_NAME", "DOES NOT EXIST")
if client_name == "DOES NOT EXIST":
    raise Exception("The environment variable for the container name of the carla server has not been set")

# Connect to the client and retrieve the world object
client = carla.Client(client_name, 2000)
world = client.get_world()
spectator = world.get_spectator()