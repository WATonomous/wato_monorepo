#!/bin/bash
#Write documentation below
#
#
#
#
#

# Run 2 instances on gpu 0, 2 instances on gpu 1 and 2 instances on gpu 2
# GPU_SETUP=(0 0 1 1 2 2)
GPU_SETUP=(1 1 1 2 2 2 3 3 4 4)

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PROFILES_DIR="${SCRIPT_DIR::-7}/profiles"

COMPOSE_FILE_PATH="$PROFILES_DIR/docker-compose.carla_instances.yaml"

file_lines=()
original_file_lines=()

if [ -f $COMPOSE_FILE_PATH ]; then
    echo "Found carla instances docker compose file"
    # instance_num=2
    # for((instance_num = 0; instance_num < $NUM_CARLA_INSTANCES; instance_num++)); do
    declare -i instance_num
    instance_num=0
    declare -i total_file_lines
    total_file_lines=1
    for GPU in "${GPU_SETUP[@]}"; do
        original_file_lines=()
        total_file_lines=1
        while IFS= read -r line; do
            original_file_lines+=("$line")
            # Process line by line
            if ! echo "$line" | grep -q "version" && ! echo "$line" | grep -q "services"; then
                if echo "$line" | grep -q "_0"; then
                    file_lines+=("${line//"_0"/"_$instance_num"}")
                elif echo "$line" | grep -q "NVIDIA_VISIBLE_DEVICES"; then
                    file_lines+=("${line//"0"/"$GPU"}")
                else
                    file_lines+=("$line")
                fi        
            fi
            total_file_lines+=1
        done <$COMPOSE_FILE_PATH
        instance_num+=1
        file_lines+=("\n")
    done
    if [ "$total_file_lines" -lt "60" ]; then
        echo "Overwriting $COMPOSE_FILE_PATH ..."
        printf "# Template Code: \n" > $COMPOSE_FILE_PATH
        for file_line in "${original_file_lines[@]}"; do
            printf "# $file_line\n" >> $COMPOSE_FILE_PATH
        done
        printf 'version: "3.8"\nservices:\n' >> $COMPOSE_FILE_PATH
        for file_line in "${file_lines[@]}"; do
            printf "$file_line\n" >> $COMPOSE_FILE_PATH 
        done
        printf 'networks:\n' >> $COMPOSE_FILE_PATH
        for ((instance=0 ; instance < instance_num ; instance+=1 )); do
            printf "  carla_instance_network_$instance:\n" >> $COMPOSE_FILE_PATH
        done
    fi
    # watod2 up
else
    echo "Did not find file... stopping"
fi
    
