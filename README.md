# Farming Simulator
## Developer's Guide
### Scripting
Scripts are stored in Assets/Scripts. Some important scripts are:

 - Interface/PathMaker.cs: Using PathMaker.Instance serves as a singleton, and stores data and variables to be accessed from any script. It also stores and generates waypoints for robots to move to.
 - Interface/SelectMenu.cs: Generates the list of robot selectables in the main menu. The prefabs of robots are referenced by a list of RobotInfo objects, which are separated by type.
 - Robots/HumanoidRobot.cs: Defines the main behavior for the humanoid robot.
 - Training/MLHumanoidController.cs: Implemented in a separate GameObject attached to the humanoid robot, this class is used for training the humanoid robot to walk. This is optional if you are manually controlling the humanoid, or using kinematic self-driving for the humanoid.
### Training

Inside the repo is a folder called ml-agents-release_22, which is a repackaged re-upload of Unity's ML-Agents repo. By navigating to this folder, you can run the command:

    mlagents-learn humanoidconfig.yaml --run-id=run_name

To run the trainer, which will save the results in results/run_name.
Then, run the game, and select the Humanoid (Training) option, and start it. The humanoid will automatically connect, and begin training. Fitness is defined in the Director script, and currently is set to try to train for walking forwards.