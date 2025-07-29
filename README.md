# V2V SDR Project

## Introduction

- The goal of this project is creating a testbed to facilitate the development of SDR wireless communication, by providing malleable ROS nodes packaged in a docker container.


## How to use

### Docker ROS Container
First you have to set up your docker image so run:
``` 
docker pull pollyjuice74/ros_lidar_base_image:latest 
```

Then:
``` 
docker run -it pollyjuice74/ros_lidar_base_image:latest /bin/bash

docker exec -it ros_lidar_base_node_env bash    # same container, multiple terminals
```

Within the container run:
```
roscore

PYTHONPATH=$(pwd)/ros_node_logic:$PYTHONPATH python3 ros_node_logic/lidar_fusion_node.py

rviz
```

### Matlab Pluto SDR
To configure the SDR change your matlab folder path to where you git cloned the project.
```
path/to/this/project
```

Install matlab dependencies:
```
- Dep1
- Dep2
```

Make sure matlab can recognize the files:
```
addpath(genpath('/home/pollyjuice74/REU-Smart-Cities-clean/v2v_sdr_project/matlab/SDR-HalfDuplex-WLAN-Transceiver'));
```

Make sure to plug Pluto in and that it's recognized!
```
findPlutoRadio
```

Then run main.m on your prefered mode
```
main('discovery')       # modes: rx, tx, duplex, discovery
```