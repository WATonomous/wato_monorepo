# Developer Guidelines for the Eve Monorepo

If you have not already, you can get a better understanding of working with Docker + ROS2 monorepo's with our onboarding assignment.

Developing in the Eve Monorepo is very similar, except there are a few caveats.

## Base Images and Docker Registry

WATonomous hosts a docker registry where we store various docker images on the internet. We currently use ghcr.io

```bash
# BEFORE YOU RUN, you need to create a personal access token with write:packages
docker login ghcr.io
# Username: <your github username>
# PasswordL <your personal access token>
```

## Pre-commit

Pre-commit is used to handle all of our code formatting and linting

```bash
pip install pre‑commit  # if you haven't installed it already
pre-commit install
pre-commit run --all-files
```
