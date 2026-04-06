#!/bin/bash

# quick script to record all topics but uncompressed images
watod bag record -a --exclude-regex "/.*image_(rect|raw|color|mono)$"