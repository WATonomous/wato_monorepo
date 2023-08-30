# watod2
`watod2` is a wrapper for docker-compose. The format is `watod2 [watod2 options] [docker-compose options]`. See watod2 options using `watod2 -h`. `docker-compose` interface can be found here: https://docs.docker.com/compose/ 

By default, `watod2` will use and create images tagged based on your current branch. For example, `perception/debug-develop`. If you switch to a new branch, you will need to rebuild your images (automatic with `watod2 up`)

For any environment variable found in `dev-config.sh`, you can overwrite it on the command line as follows: `ENV=x ENV2=y ./watod2 ...`. For example, if I am on a different branch but I want to start `develop` images, I can use `TAG=develop ./watod2 up`

**Starting Containers**: `watod2 up`
- Runs `docker-compose up` after generating your `.env` file. Your terminal will start, print out a bunch of logs, then hang while waiting for more logs. **This command does not exit**. To stop your containers, press `ctrl-c`.
- use `watod2 up -h` to see other arguments you can pass to `watod2 up`

**Stopping containers**: `watod2 down`

**Building images**: `watod2 build`

**Seeing exposed ports**: `watod2 --ports`
- Your docker containers expose a certain number of applications that can be accessed publicly. For example, [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing)
- Start your containers with `watod2 up` then in another terminal use `watod2 --ports`
- `watod2 -lp` will  also print information if you want to forward the ports from the external server to your local machine over SSH.
  
**Opening a shell inside a docker container**: `watod2 -t <SERVICE_NAME>`
- Opens a bash shell into the specified service. Find a list of your services using `watod2 ps --services`
- From here, you can execute commands inside the docker container. For example, ROS2 commands. 