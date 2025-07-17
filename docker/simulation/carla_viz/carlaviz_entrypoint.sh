# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
is_backend_up=""

function wait_for_backend_up() {
    max_wait_time=5
    for ((i=0; i<=$max_wait_time; ++i)) do
        cat output.txt | grep -q "Connected to Carla Server"
        if [ $? == 0 ]; then
            is_backend_up="1"
            break
        fi
        sleep 1
    done
}

function cleanup_backend() {
    backend_pid=$(pidof backend)
    kill -9 $backend_pid
    echo "Killed Backend process $backend_pid"
}

echo -e "CARLAVIZ_BACKEND_HOST=${CARLAVIZ_BACKEND_HOST}" >> /home/carla/.env
echo -e "CARLAVIZ_BACKEND_PORT=${CARLAVIZ_BACKEND_PORT}" >> /home/carla/.env

echo "Make sure you have launched the Carla server."
echo "Launching backend."
./backend/bin/backend ${CARLA_SERVER_HOST} ${CARLA_SERVER_PORT} |& tee output.txt &
wait_for_backend_up
if [[ -z "$is_backend_up" ]]; then
    echo "Backend is not launched. Please check if you have already started the Carla server."
    cleanup_backend
    exit 1
fi

echo "Backend launched."

echo "Launching frontend"

# enable nginx
service nginx restart
echo "Frontend launched. Please open your browser"
sleep 10
sed -i s/:8081/:$CARLAVIZ_BACKEND_PORT/g /var/www/carlaviz/bundle.js

while [ "$is_backend_up" = "1" ]
do
    cat output.txt | grep -q "time-out of"
    if [ $? == 0 ]; then
        is_backend_up=""
        cleanup_backend
        exit 1
    fi
    sleep 5
done
