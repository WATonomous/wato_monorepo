FROM mjxu96/carlaviz:0.9.13

ENV CARLAVIZ_BACKEND_HOST localhost
ENV CARLAVIZ_BACKEND_PORT 8081
ENV CARLA_SERVER_HOST localhost
ENV CARLA_SERVER_PORT 2000

WORKDIR /home/carla/carlaviz

COPY docker/simulation/carla_viz/carlaviz_entrypoint.sh /home/carla/carlaviz/docker/carlaviz_entrypoint.sh

RUN chmod +x ./docker/carlaviz_entrypoint.sh

ENTRYPOINT ["/bin/bash", "-c", "./docker/carlaviz_entrypoint.sh > /dev/null 2>&1"]
