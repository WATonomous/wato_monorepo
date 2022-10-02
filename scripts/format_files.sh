#!/bin/bash

echo "Warning: please commit your changes before formatting"
echo "do you wish to continue? (y/n)"
read
if [ -z $REPLY ] || [ $REPLY != "y" ]; then
    echo "aborting..."
    exit
fi

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
cd "$SCRIPT_DIR/.."

# finding the list of all changed C/C++ source files between this
# HEAD and the commit that branched off of the main trunk (develop)
# now ignoring file mode changes with -G.
files=$(git diff --name-only -G. develop... -- "*.cpp" "*.hpp" "*.c" "*.h")

if [ -z "$files" ]; then
    echo "No source files changed"
    exit
fi

clang-format -i --style=file $files

echo "Formatted $files"
echo "Please perform your unit tests one more time after formatting"
echo "before MR to make sure clang-format didn't break anything"
