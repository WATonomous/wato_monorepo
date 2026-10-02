# Copyright (c) 2025-present WATonomous. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

FROM python:3.12-slim-bookworm

# CARLA's converter needs PROJ's coordinate reference database at runtime.
RUN apt-get update \
    && apt-get install -y --no-install-recommends proj-data \
    && rm -rf /var/lib/apt/lists/*
ENV PROJ_LIB=/usr/share/proj

WORKDIR /app

# The repo's CARLA 0.10.0 wheel does not include Osm2Odr.
RUN pip install --no-cache-dir carla==0.9.16 \
    && python -c "import carla; assert hasattr(carla, 'Osm2Odr'); assert hasattr(carla, 'Osm2OdrSettings')"

COPY osm_to_xodr.py lanelet_to_osm.py ./

ENTRYPOINT ["python", "/app/osm_to_xodr.py"]
