## Jazzy Nav2 with modified MPPI controller for critics behavior publisher.

This publisher was officially added to the MPPI controller in the Main branch, but since this was after Nav2 underwent an overhaul, this version was not compatible with ROS2 Jazzy.

This repository contains a Jazzy MPPI controller with a custom critics behavior publisher.
In nav2_params.yaml: set ```publish_critics_debug: true``` .