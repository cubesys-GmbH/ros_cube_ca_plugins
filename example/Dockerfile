ARG ROS_DISTRO=jazzy
FROM ghcr.io/cubesys-gmbh/cube-its:${ROS_DISTRO} AS cube-its

FROM ghcr.io/cubesys-gmbh/workspace:${ROS_DISTRO} AS buildenv
COPY --from=cube-its /home/cube/cube-its/dev_ws /home/cube/cube-its/dev_ws
COPY dev_ws /home/cube/dev_ws
WORKDIR /home/cube/dev_ws
RUN source /home/cube/cube-its/dev_ws/install/setup.bash && colcon build

FROM cube-its AS deploy
USER root
# use custom ROS workspace (overlay of cube-its/dev_ws) when launching container
RUN sed -i "s|/home/cube/cube-its/dev_ws/install/setup.bash|/home/cube/dev_ws/install/setup.bash|g" /entrypoint.bash
USER cube
COPY --from=buildenv /home/cube/dev_ws/install /home/cube/dev_ws/install
# set ROS parameter to load custom plugin (quick and dirty)
RUN printf "\n\n/its/ca_tx:\n  ros__parameters:\n    dissemination_rules_plugin: \"custom_ca_plugins::CustomDisseminationRules\"" >> \
  /home/cube/cube-its/dev_ws/install/launchpad/share/launchpad/config/default.yaml

