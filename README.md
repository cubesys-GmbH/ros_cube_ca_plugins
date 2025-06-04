# cube_ca_plugins

The *cube_ca* ROS2 components running on [cube devices](https://www.cubesys.io#hardware-section) can be customized via [plugins](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html).
This repository includes the C++ interfaces and base implementations.

## Dissemination Rules

ETSI [specifies](https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.04.01_60/en_30263702v010401p.pdf) rules when and how often Cooperative Awareness Messages (CAMs) shall be transmitted (disseminated).
The `VehicleDisseminationRules` class implements these rules for vehicle stations (section 6.1.3) whereas `RsuDisseminationRules` implements them for Road Side Units (section 6.1.4).
The *cube_ca::TxNode* component uses one of these classes by default depending on its *station_type* parameter.

Primarily, the dissemination rules indicate if a CAM should be transmitted at the current time point.
Furthermore, they can also modify the provided CAM object, e.g. disabling optional containers which shall only be included at lower rates.
Please refer to the documentation in the header file of `DisseminationRules` for the full API.

Users can explicitly request the usage of particular rules, including custom implementations of `DisseminationRules`, by setting the *dissemination_rules_plugin* parameter.
For example, with the value *"my_plugins::MyDisseminationRules"* the *cube_ca* transmitter instantiates the user's `MyDisseminationRules` object from the *my_plugins* package.
