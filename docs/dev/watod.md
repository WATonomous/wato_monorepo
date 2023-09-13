# watod
`watod` is a wrapper for docker-compose. The format is `watod [watod options] [docker-compose options]`. See watod options using `watod -h`. `docker-compose` interface can be found here: https://docs.docker.com/compose/ 

By default, `watod` will use and create images tagged based on your current branch. For example, `perception/debug-develop`. If you switch to a new branch, you will need to rebuild your images (automatic with `watod up`)

For any environment variable found in `dev-config.sh`, you can overwrite it on the command line as follows: `ENV=x ENV2=y ./watod ...`. For example, if I am on a different branch but I want to start `develop` images, I can use `TAG=develop ./watod up`

**Starting Containers**: `watod up`
- Runs `docker-compose up` after generating your `.env` file. Your terminal will start, print out a bunch of logs, then hang while waiting for more logs. **This command does not exit**. To stop your containers, press `ctrl-c`.
- use `watod up -h` to see other arguments you can pass to `watod up`

**Stopping containers**: `watod down`

**Building images**: `watod build`

**Seeing exposed ports**: `watod --ports`
- Your docker containers expose a certain number of applications that can be accessed publicly. For example, [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing)
- Start your containers with `watod up` then in another terminal use `watod --ports`
- `watod -lp` will  also print information if you want to forward the ports from the external server to your local machine over SSH.
  
**Opening a shell inside a docker container**: `watod -t <SERVICE_NAME>`
- Opens a bash shell into the specified service. Find a list of your services using `watod ps --services`
- From here, you can execute commands inside the docker container. For example, ROS2 commands. 