# Self driving with ROS, AutoNOMOS Simulator, Gazebo, Keras y Tensorflow

This convolutional neural network is based on PilotNet but includes a set of convolutional layers dedicated to process information from a LIDAR sensor as an image:

https://user-images.githubusercontent.com/47704357/159609637-f5589152-954e-4159-abfe-c469e469486a.mp4

An implementation of PilotNet by Nvidia:

https://user-images.githubusercontent.com/47704357/159609500-d74c3e72-141e-4dad-a784-cc00acf443eb.mp4


Neural Network Training Code:
https://github.com/Exusai/EK_AutoNOMOS_Sim/blob/master/src/autonomos_gazebo_simulation/scripts/autodrive_evation_train.ipynb

[Scripts used to train the network](https://github.com/Exusai/EK_AutoNOMOS_Sim/tree/master/src/autonomos_gazebo_simulation/scripts), includes manual drive to collect data and the notebooks used to train the network. [Trainded models here](https://github.com/Exusai/EK_AutoNOMOS_Sim/tree/master/src/autonomos_gazebo_simulation/scripts/models).

<details>
  <summary>About AutoNOMOS</summary>
  <p>Stand alone repo for the AutoNOMOS Gazebo simulation. Please check the <a href="https://github.com/EagleKnights/Eagle_Knights-Wiki/wiki">wiki</a> for information on how to use the simulation. </p> 
  <p>
    The files are divided in two: <br>
    * Gazebo plugin: Files to interact with the gazebo model. <br>
    * Autonomos: files of the model, world and example launch files. <br>
  </p>
</details>

 
