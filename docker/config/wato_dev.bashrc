# WATonomous development bashrc additions
# This file is sourced by ~/.bashrc in development containers

# Enable terminal colors
export TERM=xterm-256color
# Used by the calling interactive bashrc.
# shellcheck disable=SC2034
force_color_prompt=yes

# Colored prompt - clearly indicates we're in a container
PS1='\[\033[01;35m\][container]\[\033[00m\] \[\033[01;32m\]\u\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

# Enable color support for ls and grep
alias ls='ls --color=auto'
alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'

# Useful ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Source the same underlays as the entrypoint. Shells opened through docker
# exec (watod -t / VSCode attach) do not run the container entrypoint again.
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -n "${ISAAC_ROS_INSTALL:-}" ]; then
  source "${ISAAC_ROS_INSTALL}/local_setup.bash"
fi
if [ -f /opt/watonomous/local_setup.bash ]; then
  source /opt/watonomous/local_setup.bash
fi

# Allow ANSI color codes in ROS2 log output
export RCUTILS_COLORIZED_OUTPUT=1

# Add ~/.local/bin to PATH
export PATH="$HOME/.local/bin:$PATH"

# Add any custom aliases or functions here
