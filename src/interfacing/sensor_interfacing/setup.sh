#!/bin/sh
set -e

# Display configuration
echo Using following configuration
echo VCAN_INTERFACE: "${VCAN_INTERFACE:=vcan0}"
echo VCAN_SOURCE_PORT: "${VCAN_SOURCE_PORT:=6000}"
echo VCAN_TARGET_IP: "${VCAN_TARGET_IP:=172.17.0.1}"
echo VCAN_TARGET_PORT: "$VCAN_TARGET_PORT"

# Setting up vcan
ip link add "$VCAN_INTERFACE" type vcan
ip link set up "$VCAN_INTERFACE"
echo Created vcan

# Setting up can2udp
can2udp -fp "$VCAN_SOURCE_PORT" "$VCAN_INTERFACE" "$VCAN_TARGET_IP" "$VCAN_TARGET_PORT"
echo Started can2udp

# Listening on vcan
echo Listening on vcan
candump "$VCAN_INTERFACE"
