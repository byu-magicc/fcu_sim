FCU Sim
===============

fcu_sim is a MAV gazebo simulator.  It has been modified for use in the MAGICC ros_plane and ros_copter framework.

It based on the RotorS simulator created by researchers at eth-Zurich, however much of that package has been removed.  If interested, the original package can be found at https://github.com/ethz-asl/rotors_simulator.

Tutorials
==============
We have created tutorials to help you get started building models, worlds and plugins.  They located at [magiccvs.byu.edu/wiki/Gazebo_Tutorials](http://magiccvs.byu.edu/wiki/Gazebo_Tutorials).  These should be publicly available for anyone who is interested in using this simulator to help with their research.


File Descriptions
---------------------------

This is a metapackage.  The packages are organized as follows:

* fcu_sim is the meat of the package.  Most of the configuration for the simulation is found in fcu_sim.  The subfolder organization is a bit cryptic.  Here is how it is organized, besides what is obvious.

	** meshes:  the meshes folder contains the 3D geometry for displaying multirotors.  Since the original file was based on Ascending Tech MAV's the Pelican, Hummingbird, and Firefly have been programmed in.  The shredder model uses the Firefly, even though Shredder is quite a bit larger than the Firefly, and is also a Hex + configuration instead of a Hex X like the Firefly.  It just works for now.

	** worlds:  The worlds folder contains the 3D geometry for different environments.  If we make new worlds, we should put them in this folder.  The world used is specified in test.launch while launching the gazebo_ros node.

	** urdf:  URDF files are used to define the "actors" or robots in the simulation.  The way it works right now is when the gazebo simulator is started up, it is fed the shredder_base.xacro file.  This file calls a bunch of other xacro files, such as the "rotor" xacro which defines propeller models, sensors, such as the xtion, imu, laser scanner etc... but it also calls a controller for the motors, and the "shredder_mechanics" xacro, which defines the model used in the simulation.  The data in the shreder_mechanics.xacro file needs to match the data in the shredder.yaml file in the param folder.  shredder_mechanics is read by gazebo under the hood to propogate the states forward.  shredder.yaml is read by ROS for the various nodes to function properly.  It's unfortunate that these files aren't linked.

	** param:  The param folder holds only shredder.yaml, which is used to define the model for the ROS nodes, specifially the attitude controller for accurate control.  The attitude controller does automatic mixing of propeller outputs based on the inputs in the shredder.yaml file.  The attitude control gains are also specifed in shredder.yaml.

	** code:  There is some C++ ROS code in this package, but it's mostly legacy.  At some point, it can probably all be removed.

* fcu_sim_plugins
	This package is used to basically manipulate the model in gazebo.  At the time of this writing, there are 10 plugins.  The plugins are defined by the files in src/ and include/, and are hooked into gazebo via the xacro files.  The xacro files in this package have been expanded to include some customized sensor xacros whose plugins are implemented in the ROS system files.  These are the rgbd camera, laser scanner and odometry sensor files.  You can import the xacro files for the various sensors and models into your shredder_base.xacro file to use them on your model.  They have arguments much like ROS launch files to customize them to your needs.

* rotor_sim_joy
	This package was made specifically for testing the simulation.  In this mode the Joystick functions like a RC transmitter while the multirotor files in "stabilize" mode.  It cannot work at the same time as joy_commander.  It assumes that you are using xboxdrv by default, but parameters can be used to map different joy inputs.

* attitude_controler
	This implements a standard PID controller around roll, pitch, and yawrate, and outputs thrust based on inputs from a relative_nav::Command message.  This should be a drop-in replacement for mavros so it hooks in with the rest of the code seamlessly.

	It automatically maps outputs by calculating the inverse transform of the allocation matrix.  The allocation matrix is formed using the data from the shredder.yaml file, and gains for the controller are also found in the same file.  The mass and inertia information in the yaml file is also used to inform the controller.

* sim_reset
	This is a very simple message that listens to the joy messages and resets the simulator given a button press on the "Y" button (when using xboxdrv)

How To Use
-------------------------

To use for simulating shredder, one can simply just copy the launch file and customize it for their needs.  However, creating a new model may be more complicated.

* First, you will need to duplicate a number of files, those are
	1 - fcu_sim/urdf/shredder_base.xacro
	2 - fcu_sim/urdf/shredder_mechanics.xacro
	3 - fcu_sim/param/shredder.yaml

* Go through these files and reconfigure them to match your desired MAV.  If you want to import new geometry from a .dae mesh file, that can be saved in the fcu_sim/meshes folder.

* Copy fcu_sim/launch/simulator.launch, which is a bare-bones launch file for simply creating the simulation environment.  Modify the model launched by the spawn_mav.launch command, and the param file loaded by the attitude_controller node to match the files you copied earlier.

Try launching simulator.launch.  It should function, and you will see your MAV flying in Gazebo.  You can additionally run rotor_sim_joy to control your MAV like it were being commaned by RC inputs.
