FCU Sim
===============

fcu_sim is a MAV gazebo simulator.  It has been modified for use in the MAGICC ros_plane and ros_copter framework.

It based on the RotorS simulator created by researchers at eth-Zurich, however almost all of that package has been removed or changed.  If interested, the original package can be found at https://github.com/ethz-asl/rotors_simulator.

Installation
===============
To install, simply clone this metapackage into the `src` directory of your catkin workspace. You will then need to pull the ROSflight2 submodule for the software-in-the-loop package to successfully build.

```bash
cd fcu_sim
git submodle update --init --recursive
```

Feedback and Contributing
==============
We welcome feedback and contributions via Issues and Pull Requests.  This software is distributed with an Apache license like the RotorS simulator, which allows for use in both open source and closed-source applications, all you need to do is keep a copy of the license at the top of all redistributed and derivative work.  The license can be found [here](http://www.apache.org/licenses/LICENSE-2.0)

Tutorials
==============
We have created tutorials to help you get started building models, worlds and plugins.  They located at [magiccvs.byu.edu/wiki/Gazebo_Tutorials](http://magiccvs.byu.edu/wiki/Gazebo_Tutorials).  These should be publicly available for anyone who is interested in using this simulator to help with their research.


File Descriptions
---------------------------

This is a metapackage.  The packages are organized as follows:

* fcu_sim is the meat of the package.  Most of the configuration for the simulation is found in fcu_sim.  The subfolder organization is a bit cryptic.  Here is how it is organized, besides what is obvious.
* 
	** launch: contains launch files for launching common simulations.  This includes `fixedwing.launch` which launches a simulation using the `junker` files and the fixedwing autopilot from BYU-MAGICC/ros_plane, and `simple.launch` which launches a multirotor while simulating RC inputs from a joystick.

	** meshes:  the meshes folder contains the 3D geometry for displaying multirotors.  Since the original file was based on Ascending Tech MAV's the Pelican, Hummingbird, and Firefly have been programmed in.  The shredder model uses the Firefly, even though Shredder is quite a bit larger than the Firefly, and is also a Hex + configuration instead of a Hex X like the Firefly.  It just works for now.  Collada (`.dae`) files can be created using standard CAD models found at various websites and converting using Meshlab or similar software.  We have had a lot of success using StarWars models as our 3D geometry.  Makes for a bit of fun.

	** worlds:  The worlds folder contains the 3D geometry for different environments.  If we make new worlds, we should put them in this folder.  The world used is specified in the launch file	 while launching the gazebo_ros node.

	** urdf:  URDF files are used to define the "actors" or robots in the simulation.  The way it works right now is when the gazebo simulator is started up, it is fed the `shredder_base.xacro` file.  This file calls a bunch of other xacro files, such as the `multirotor_forces_and_moments.xacro` which defines propeller models, sensors, such as the xtion, imu, laser scanner etc...

* fcu_sim_plugins
	This package is used to basically manipulate the model in gazebo.  At the time of this writing, there are 10 plugins.  The plugins are defined by the files in src/ and include/, and are hooked into gazebo via the xacro files.  The xacro files in this package have been expanded to include some customized sensor xacros whose plugins are implemented in the ROS system files.  These are the rgbd camera, laser scanner and odometry sensor files.  You can import the xacro files for the various sensors and models into your shredder_base.xacro file to use them on your model.  They have arguments much like ROS launch files to customize them to your needs.

How To Use
-------------------------

To use for simulating shredder, one can simply just copy the launch file and customize it for their needs.  However, creating a new model may be more complicated.

* First, you will need to duplicate the base `xacro` file, specifically fcu_sim/urdf/shredder_base.xacro

* Go through this file and reconfigure it to match your desired MAV.  If you want to import new geometry from a .dae mesh file, that can be saved in the fcu_sim/meshes folder.

* Copy fcu_sim/launch/simulator.launch, which is a bare-bones launch file for simply creating the simulation environment.  Modify the model launched by the spawn_mav.launch command to match the files you copied earlier.

Try launching `simulator.launch`.  If all goes well, you should see your MAV fall to the ground in Gazebo (This is because it not being provided with any commands).  You can additionally run fcu_common/joy to control your MAV like it were being commaned by RC inputs.  `simple.launch` should run this joystick RC teleop as well as launching your MAV in Gazebo.  Look at the documentation for `fcu_common/joy` to know how to use this node.

To use the ROSflight SIL plugin instead of the simplified multirotor dynamics, simply uncomment line 60 of shredder_base.xacro, and comment out line 64.  This instead loads the software-in-the loop plugin and runs the embedded ROSflight code in Gazebo.

Technical Details
=======================
Simulation of multirotors is done in two ways:

1. First-order models of applied body-fixed torques.  Torques are applied to the body-fixed axes of the UAV using PID control and with a first-order response.  The time constants and thrust/torque response for several motors/propeller combinations have been measured using the thrust test stand at BYU, and the information in the shredder and mikey configurations are accurate.  We have data for all the 3DR motor/propeller combination, a number of 250-size quad systems, MikroCopter systems, and old Ascending Technologies systems.  We have found that this first-order model is appropriate for most simulations.  It is somewhat simplistic, bu it is faster to calculate than other methods.

2. ROSflight SIL.  This method uses the actual ROSflight code to perform estimation and control.  Forces are then applied as if propellers at the various locations on the airframe.  Currently, propellers not aligned with the body-fixed Z axes are not supported, but likely could be if you need something like that.  Individual motor response is modeled using a quadratic fit, and a first-order response.  This parameterization fits very well the data collected on actual thrust stands.  For more information on how to collect the motor data for your platform, please contact one of the authors via an Issue or personal message.

Simulation of Fixed wing MAVs is performed using the method described in "Small Unmanned Aircraft - Theory and Practice" by Randy Beard and Tim McLain.  This model uses stability derivatives which can be calculated by AVL for a particular airframe, and lift coefficients from a particular airframe.  A detailed explanation of each term in the aircraft_forces_and_moments plugin can be found in that book.  Currently, ROSflight SIL is not well supported for fixed wing MAVs.  We expect this to change in the near future.
