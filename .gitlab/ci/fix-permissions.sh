#!/bin/bash

# This script makes permissions consistent across all systems so that Docker caching doesn't break
# (git does not preserve some permissions that Docker uses for caching)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# remove all setgid bits
find "${SCRIPT_DIR}/../.." -exec chmod g-s {} \+
# set all directories to rwx by owner, rx by everyone else
find "${SCRIPT_DIR}/../.." -type d -exec chmod 755 {} \+
# set all files to rw by owner and r by everyone else
find "${SCRIPT_DIR}/../.." -type f -exec chmod 644 {} \+
# add executable bit for executable files
find "${SCRIPT_DIR}/../.." -type f \( -name '*.sh' -o -name '*.exp' -o -name '*.py' -o -name 'watod2' \) -exec chmod +x {} \+

