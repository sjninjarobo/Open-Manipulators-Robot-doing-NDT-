# Open-Manipulators-Robot-doing-NDT-
Ultrasonic imaging of a metallic test object using a robotic platform was my dissertation project in the final year of MTech at IISc Bangalore.
# Background
A nondestructive test can be defined as testing the physical properties of an object without
destroying the future usefulness of the test object.NDT is primarily used in industrial setups
to detect flaws, cracks, fracture defects, and voids in materials used to build aeroplanes, power
plants, bridges, etc. A nondestructive test evaluates an energy/defect interaction
which is accomplished by applying pulsed energy in the form of ultrasound to the test object.
The presence of flaws is inferred from the received reflected ultrasonic pulses from the test
specimen. X-rays are also prevalent for NDT testing
# Ultrasonic Imaging: A-Scan, B-Scan and C-Scan
Ultrasonic data can be collected and displayed in several different formats. The three most
common formats are known in the NDT world as A-scan, B-scan and C-scan presentations. Each
presentation mode provides a different way of looking at and evaluating the region of material
being inspected.
The A-scan presentation displays the amount of received ultrasonic energy as a function of
time. The relative amount of received energy is plotted along the vertical axis and the elapsed
time (which may be related to the sound energy travel time within the material) is displayed
along the horizontal axis.
The B-scan presentation is a profile (cross-sectional) view of the test specimen. In the B-scan,
the time-of-flight (travel time) of the sound energy is displayed along the vertical axis and
the linear position of the transducer is displayed along the horizontal axis. From the B-scan,
the depth of the reflector and its approximate linear dimensions in the scan direction can be
determined.
The C-scan presentation provides a plan-type view of the location and size of test specimen
features. The plane of the image is parallel to the scan pattern of the transducer. C-scan
presentations are produced with an automated data acquisition system, such as a computer-controlled immersion scanning system 

![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/e615035d-07be-422f-bfca-5a9c19dd6364)
# Experimental Procedure
![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/7c5ac77d-cb73-4ac1-b2f8-00241dd881d2)

 Figure 4.1: Original Hardware setup
# Procedure
Fig 4.1 shows the complete setup for our project. The intel Realsense depth camera d435i
mounted on the robot is used to capture the depth image of the test object along with some
background. So the camera’s output is (x,y,z) of all the points in the image. What do we do
from that?
The x,y, and z data are colourized according to the z data of all the points/pixels in the image as shown
in Fig 4.2. After getting the depth stream data the image stream is dissected into the foreground(test
object pixels) and background data, after which an inbuilt MATLAB package using lazy snap
the algorithm gives us a black-and-white image with the test object as white as shown in Fig 4.4
![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/98980256-d59e-45dc-8055-92b40c4ca8a1)
Figure 4.2: Using Intel’s cross-platform APIs to stream Depth frames from the camera and then
colourize it. Output is an 848x480x3 RGB image

![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/13bca41c-00bd-45d9-8839-041506e0fae6)
Figure 4.3: test object with a hole of 5mm

![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/e7e1db76-8cb3-4630-9def-03f778250c6c)
Figure 4.4: Extracted test object as white

After we get the white image of the test object we find the indices of pixels that are white and find
out the x,y, and z of those points and generate the point cloud data in the camera coordinate frame.Fig
4.5 below shows the visualisation of a generated point cloud. After point cloud generation of
our target object for NDT we also need to know the normals at all those points of testing.The
reason to know the normals is that in our experiment we are using a single pulser/receiver US
probe, the probe needs to be aligned perpendicular to the surface of the test object to get a clear
reflection from defects and surfaces.

![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/6e69732b-149e-4774-a3b3-99d2820b0f9d)
Figure 4.5: visualisation of generated point cloud

So the next step is to calculate the normals at all points.
To calculate the normals ”CreateNormals” function is used to get a unit vector of normals at the
point cloud data in the Robot’s base coordinate frame. It uses KNN to create a surface with N nearest
neighbour points 6(default) neighbouring points are used to fit a local plane to determine the
normal vector. The normals formed in the robot’s base coordinate frame can be seen in fig4.6
below
Nevertheless, once we get the normals and points the last thing before
giving the kinematic poses as input to our inverse kinematics solver. There needs to be a transformation matrix
between the end effector and the robot’s base. Now the robot’s design is slightly changed as per
the requirements of the project. The camera along with its holder, the robot’s base holder, a US probe
holder along with probe. So the URDF file also needed to be changed. In this project, we have
neglected the dynamics and we are focusing only on kinematics. 
 # Kinematics of robot
 ![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/d9b24584-f427-485b-853e-90b1280c47d7)

Figure 4.6: visualisation of generated point cloud normals in robot’s base frame
The configuration of a robot manipulator is determined by its joint variables such as joint angles
or joint distance. Task specification is usually given in a Cartesian space, i.e. the desired position
and orientation of the end effector (Ultrasonic probe) is given in a Cartesian coordinates frame.
The problem is How do we relate the joint variables to the position and orientation of the endeffector?
This is resolved by the inverse kinematics solver of the robot. In our project, the main
joints focus on our end effector joint which has been downshifted in the -ve z direction and
forward shifted in the +ve x direction by some distance because of the US probe being put in
the holder which is fixed to the gripper joint. Similarly, the camera holder is uplifted in the +ve Z
direction to remove the hindrance of cables in the camera FOV(Field of View).All the changes in
both coordinate frame are shown in fig below.
![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/4c0c74bb-d1ea-4090-b7ca-6074ac60f97d)

Figure 4.7: Changed robot’s design and joints coordinate frame\

After doing changes in the URDF file as well as in the original hardware now the robots are controlled
by giving joint angles as input through the ROS service
goal joint space path(open manipulator msgs/SetJointPosition.Refer finalcodev2.m in appendix
to get an idea about the ik solver algorithm and how the robot’s trajectory is being planned and
controlled.

 # Normals at point cloud data
The normals that we get from createNormals are in the base coordinate frame. Now to ensure
that US probe orientation while traversing all the points in point cloud data is perpendicular
to test the object, the Z axis of the end effector needs to be aligned parallel to the normals at each
of the test point. This is ensured by calculating the rotational transformation matrix around X,Y
and Z axis respectively.
From previous knowledge, we know that the rotation matrix of a coordinate system around its
x,y,z axis by φ,θ,ψ angle respectively.
The equation of rotations for the same is given below 4.8: For our case, we need our probe’s Z
![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/67850883-5ebc-4303-87ef-d2adfaa1d021)

Figure 4.8: rotational matrix equation
axis [0 0 1] to align with normal[u v w] which brings down the final equation to be: for that the
end effector’s coordinate frame needs to be transformed along the X-axis by an angle(φ-atan(v/w)),
along Y axis by an angle(θ=asin(u)) and no rotation along Z axis i.e.ψ=0.

# System Limitations and Challenges Faced
• Maintaining the pose of a 4 DOF manipulator over a complex path.
• No access to alter the dynamics of the robot.
• System integration: Manipulator, ROS (as a middleware), Camera, DSO and MATLAB.
• No locking of torques of Manipulator joints in case of power failure.
• Humidity and moisture tolerance of ICs in the manipulator as well as the depth camera was
found to be very low.

# Experimental Results
To qualitatively analyse the volumetric data regenerated by robotic method, it is compared
with data regenerated experimentally using a combination of an automated And Manual
LTS (Linear Travel Stage) for a 30 mmx30mm area on the same sample. Figure 5.1 shows

![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/08947460-391a-43e3-a646-7d53a19520b9)

Figure 5.1: Using Linear travel stage
the 3D volumetric reconstruction of the 30mmX30mm area over the test object inclusive of the
hole. Figure 5.2 shows the 3D volumetric reconstruction of the test object generated by the
movement of the robot over the top surface of the test object.

![image](https://github.com/sjninjarobo/Open-Manipulators-Robot-doing-NDT-/assets/171892063/a03523eb-0287-4ee9-ab9e-76e8a99d907e)

Figure 5.2: Reconstructed image using Robotic movement
