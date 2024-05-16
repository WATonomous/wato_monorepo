#!/bin/bash
set -e

# Takes in a branch name and sanitizes the name for docker tagging
# (eg. no / in the branch name)
sanitize_branch_name() {
    echo $(echo $1 | sed 's/[^a-zA-Z0-9._]/-/g' | cut -c 1-128)
}

################# Setup Source and Target Branch #################
SOURCE_BRANCH_NAME=$(sanitize_branch_name $SOURCE_BRANCH)
TARGET_BRANCH_NAME=$(sanitize_branch_name $TARGET_BRANCH)

if [ -z "$TARGET_BRANCH_NAME" ]; then
    TARGET_BRANCH_NAME=$SOURCE_BRANCH_NAME
fi

echo "source_branch=$SOURCE_BRANCH_NAME" >> $GITHUB_OUTPUT
echo "target_branch=$TARGET_BRANCH_NAME" >> $GITHUB_OUTPUT

################# Debug notices #################
echo "::notice:: Using $SOURCE_BRANCH_NAME as the source branch"
echo "::notice:: Using $TARGET_BRANCH_NAME as the target branch"
