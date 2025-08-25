# Fire hazard prevention system

The goal of this project is to identify an early stage fire in a bulding or outside in zone without internet connection. This project is done by Adrien, Rémi and Matéo for the purpose of our intership at OIT: Osaka's Institute of Technology.

## Robot

The robot logic is available in the `./robot/scripts/` folder. As it works with ROS noetic and rviz, only the scripts are included. To run the robot in a simulation, you'll need to have set up ROS and catkin, as well as [KMiyawaki/oit_navigation_minibot_light_01](https://github.com/KMiyawaki/oit_navigation_minibot_light_01) within the catkin folder.

For more information, please read the [`./robot/README.md`](./robot/README.md).

## Computer vision

Structure:

- [`./computer-vision/infrastructure/`](./computer-vision/infrastructure/): Kubernetes resources to train the model on a server with a GPU.
- [`./computer-vision/server/`](./computer-vision/server/): Server-side code used on the robot. It uses the model we have trained, mainly to identify fires.
- [`./computer-vision/training/`](./computer-vision/training/): Code used to train the model.

### Citations

- For the D-Fire dataset: Pedro Vinícius Almeida Borges de Venâncio, Adriano Chaves Lisboa, Adriano Vilela Barbosa: [An automatic fire detection system based on deep convolutional neural networks for low-power, resource-constrained devices.](https://link.springer.com/article/10.1007/s00521-022-07467-z) In: Neural Computing and Applications, 2022.
