# VSCode
## **Over SSH**

1. Download and install [Visual Studio Code](https://code.visualstudio.com/) on your local machine
3. In VS Code, install the [Remote Development extension pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
4. Select the "Remote Explorer" tab in the left taskbar
5. Make sure the Remote Explorer is targeting "SSH Targets" and click plus to add the server you are trying to access.  For example, "wato-tr.uwaterloo.ca".
6. Right click on the added target and connect. Select your workspace, and you can edit code in `src`.
7. To make an integrated VScode terminal that runs on the host, use `ctrl->p` `> Create New Integrated Terminal`.

## **Over SSH and Docker**

If you want to attach VScode to a specific container instead of to the host, you need to follow some extra steps as the interaction between vscode-remote and vscode-docker is buggy. The issue is currently being tracked here: https://github.com/microsoft/vscode-remote-release/issues/2514

1. Download and install [Docker](https://www.docker.com) on your local machine
2. Follow the "Over SSH" instructions
3. Once your VScode is connected to the server, install the [Docker extension pack](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) on the remote VScode
4. in VScode, open `settings.json` (`ctrl-p` `>settings.json`)
5. Add the line: `"docker.host": "tcp://localhost:23750",` near the top
6. In a terminal on your local machine, start an SSH tunnel for the docker daemon
    `ssh -L localhost:23750:/var/run/docker.sock user@hostname.`
7. Enter the VS Code Docker extension tab and find your container in the "Individual Containers" section.
8. Right click on your container and select "Attach VS Code to Container"
9. Done! At this point you should be able to edit code and run Catkin/ROS commands in the container through the VS Code terminal