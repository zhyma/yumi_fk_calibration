For calibrating YuMi's FK function

Need to setup:
- Trac-IK: `sudo apt-get install ros-noetic-trac-ik`
- transforms3d `pip install transforms3d`
- RealSense (librealsense)
- ar_track_alvar
- [yumi](https://github.com/zhyma/yumi/tree/coil_dev)(fork from [kth-ros-pkg](https://github.com/kth-ros-pkg/yumi/). Please use `coil_dev` branch)
- [ariadne_plus](https://github.com/zhyma/ariadne_plus/tree/coil_dev)(fork from [lar-unibo/ariadne_plus](https://github.com/lar-unibo/ariadne_plus), using the [same trained model](https://drive.google.com/file/d/1rwyuUeltodsZjm53q6_46a8T-dRh1pnw/view?usp=sharing). Please select `coil_dev` branch, with `srv` file modified)
	- `pip3 install arrow termcolor igraph scikit-image pytorch-lightning==1.7.1 torch==1.8.2 torchvision==0.9.2 torchaudio==0.8.2 --extra-index-url https://download.pytorch.org/whl/lts/1.8/cpu`
  - Or get the GPU version for pytorch, such as `https://download.pytorch.org/whl/lts/1.8/cu111`, depending on the CUDA version.

To run the package
  - Use `roslaunch yumi_fk_calibration demo.launch`
  - Run python scripts 


To run the simulator
  - Use `roslaunch yumi_fk_calibration yumi_gazebo_moveit.launch`
  - Then use `roslaunch yumi_fk_calibration rviz_sim.launch`

- `Failed to load library /opt/ros/noetic/lib/libmoveit_motion_planning_rviz_plugin.so`
  - Error log:
    ```
    [ERROR] [1678130372.698112823]: PluginlibFactory: The plugin for class 'moveit_rviz_plugin/MotionPlanning' failed to load.  Error: Failed to load library /opt/ros/noetic/lib/libmoveit_motion_planning_rviz_plugin.so. Make sure that you are calling the PLUGINLIB_EXPORT_CLASS macro in the library code, and that names are consistent between this macro and your XML. Error string: Could not load library (Poco exception = libmoveit_background_processing.so.1.1.11: cannot open shared object file: No such file or directory)
    ```
  - Solution:
    ```
    sudo apt-get update
    sudo apt-get dist-upgrade
    ```