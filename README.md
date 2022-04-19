# Multi-Level-Adaptation-for-Automatic-Landing-with-Engine-Failure-under-Weather-Uncertainties

This repository contains MASC Online navigation scheme, with support of a high fidelity simulation environment(X-Plane), DSP toolbox and User Datagram Protocol (UDP) based communication. The system takes in landing zone coordinates. It later navigates the engine malfunction airplane to configured safty landing position in real-time. A demonstration of the system can be found here -> https://www.youtube.com/watch?v=2c1hPMZSF4I

This picture blow is MASC Online Path Planning Framework.
<p align='center'>
    <img src="/MASC(Online Navigation Module)/graph/MASC Autopilot.png" alt="drawing" width="700"/>
</p>


## Dependency

- [MATALB R2021a](https://www.mathworks.com/products/new_products/previous_release_overview.html)
- [DSP System Toolbox](https://www.mathworks.com/products/dsp-system.html)
- [UAV Toolbox](https://www.mathworks.com/products/uav.html)
- [Aerospace Blockset](https://www.mathworks.com/products/aerospace-blockset.html)



## Set up input data channel and output data channel in X-Plane

   <p align='center'>
    <img src="/MASC(Online Navigation Module)/graph/IO_configure.png" alt="drawing" width="300"/>
   </p>

## Configure IP address in MATLAB/Simulink

- The fixedWingPathFollowing model integrates the nonlinear guidane logic , UDP intterface
  with the high fidelity simulation environment. This model is to extract necessary information
  from the airplane status output bus signal and feed them into the waypoint follower to form 
  a control loop. The model assembles the control and environment inputs for the guidance model
  block.
 
   ```
   open_system('fixedWingPathFollowing');
   ```
-  Configure sender and publisher: Please follow instructions shown in the picture to set up 
   IP adddress for subscriber and publisher. Likewise  IP address configuration box in X-Plane.
   
   <p align='center'>
    <img src="/MASC(Online Navigation Module)/graph/Publisher_Config.png" alt="drawing" width="200"/>
   </p>
   
   <p align='center'>
    <img src="/MASC(Online Navigation Module)/graph/Subscriber_Config.png" alt="drawing" width="200"/>
   </p>
## Drop Airplane to any where we hope it be engine out position

- [Client matlab program](https://github.com/haotiangu/XPlaneConnect.git)
    
## Run the online path planning module
   
1.  Open the program and then first to put airplane to wherever the engine is broken, in which define the 
   engine-out latitude,longitude and altitude. And then click the stop button at the up right corner of 
   the X-Plane window.
   
2.  Set up the engine out global position in X-Plane in malfunction position configuretion block
3.  Set up the airport coordinates in simulink framework
4.  Click the run button to start the simulink model first.   
   ```
   sim("fixedWingPathFollowing");
   ``` 
6.  And then click the start button in the up right
   corner of the X-Plane window. Or run command in the matlab terminal.
 
   ```
   close_system("fixedWingPathFollowing");
   ``` 

## Cite *MASC*

Thank you for citing [our *MASC* paper](./AIAA_SciTech_2023___Automatic_Emergency_Landing.pdf) if you use any of this code: 
```
@inproceedings{legoloam2018,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```

## Simulation under the windy weather and Turbulence

