# Copyright (c) 2025-present WATonomous. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py .
COPY templates templates/

EXPOSE 8888

CMD ["python", "app.py"]
