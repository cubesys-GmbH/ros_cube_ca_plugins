ARG ROS_DISTRO=jazzy
FROM ghcr.io/cubesys-gmbh/cube-its:${ROS_DISTRO} AS cube-its

FROM ghcr.io/cubesys-gmbh/workspace:${ROS_DISTRO} AS buildenv
ARG ROS_DISTRO
COPY --chown=cube:cube dev_ws /home/cube/dev_ws
WORKDIR /home/cube/dev_ws
RUN source /opt/cube/${ROS_DISTRO}/setup.bash && colcon build

FROM cube-its AS deploy
USER root
# additionally load custom workspace when launching container
RUN sed -i -e '/^source /a source "/home/cube/dev_ws/install/local_setup.bash"' /entrypoint.bash
# set ROS parameter to load custom plugin (quick and dirty)
RUN printf "\n\n/its/ca_tx:\n  ros__parameters:\n    dissemination_rules_plugin: \"custom_ca_plugins::CustomDisseminationRules\"" >> \
  /home/cube/cube-its/dev_ws/install/launchpad/share/launchpad/config/default.yaml
COPY --from=buildenv /home/cube/dev_ws/install /home/cube/dev_ws/install

