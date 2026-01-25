# WATonomous development bashrc additions
# This file is sourced by ~/.bashrc in development containers

# Enable terminal colors
export TERM=xterm-256color
force_color_prompt=yes

# Colored prompt
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

# Enable color support for ls and grep
alias ls='ls --color=auto'
alias grep='grep --color=auto'
alias fgrep='fgrep --color=auto'
alias egrep='egrep --color=auto'

# Useful ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Add ~/.local/bin to PATH
export PATH="$HOME/.local/bin:$PATH"

# Add any custom aliases or functions here
