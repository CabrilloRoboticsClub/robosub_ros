# Model 

## URDF
See: [ROS2 URDF Tutorial - Describe Any Robot (Links and Joints)](https://www.youtube.com/watch?v=LsKL8N5Iwkw)

### Overview
```xml
<!-- <? xml version=""> -->
<robot name="robot_name">
	<link name="link_name">
		<visual></visual>
		<collision></collision>
		<inertial></inertial>
	</link>
	
	<joint name="joint_name">
		<axis .../>
		<parent link .../>
		<child link .../>
		<origin .../>
	</joint>
</robot>
```
- Used to describe a robot's visual, physics and connections through **joints** and **links**.
- File has an `xml` version at the top then the robot's name. The robot itself is broken up into two sections, `link` and `joint`.
### Links
```xml
<link name="link_name">
	<visual></visual>
	<collision></collision>
	<inertial></inertial>
</link>
```
- A link is used to describe a segment of a robot where you describe the `visual`, `collision` and `inerial` properties. 
- The `geometry` and `origin` properties are used in `visual` and `collision` properties to describe the shape and location of the link.
- `visual`
	- `visual` is used to describe the geometry of the link. Can be an accurate or simplified representation depending on the goal. For an accurate representation, an `stl` file is included in the geometry: `<mesh filename="file_path" scale ="x y z">`
	- `origin` defines the location of the link in terms of the rotation in radians (rpy) and the translation (xyz). Rotation is applied first then translation.
	- `material` can be used to describe colors in terms of `rgba` normalized between 0 to 1
	```xml
	<visual> <!-- OPTIONAL -->
		<geometry> <!-- REQUIRED -->
		</geometry>
		<origin rpy="r p y" xyz="x y z"/> <!--OPTIONAL -->
		<material name="color_name"/> <!--OPTIONAL -->
	</visual>
	```
- `collision`
	- `collision` describes a bounding shape around the link for collision detection. Usually a simplfied shape is chosen for faster calculations.
	```xml
	<collision> <!-- OPTIONAL -->
		<geometry> <!-- REQUIRED -->
		</geometry>
		<origin rpy="r p y" xyz="x y z"/> <!--OPTIONAL -->
	</collision>
	```
- `inertial`
	- `inetrtal` describes the inertia and mass of the link
	```xml
	<inertial>
		<mass value="mass_in_kg"/>
		<inertia 
			ixx="ixx" ixy="ixy" ixz="ixz"
			iyy="iyy" iyz="iyz"
			izz="izz"
		/>
	</inertial>
	```
### Joint
```xml
<joint name="joint_name" type="joint_type">
	<!--OPTIONAL -->
	<parent link="parent_link_name">
	<child link="child_link_name">
	<!-- REQUIRED -->
	<origin rpy="r p y" xyz="x y z"/>
	<axiz rpy="r p y" xyz="x y z"/>
	<limit lower="lower" upper="upper"
	 effort="max_effort" velocity="max_velocity"/>
	<dynamics damping="damp_coef"       
	 friction="friction_coef"/>
</joint>
```
- Joints are used to describe the connection between two links, a `parent link` and `child link`
- Types of joints
	- `fixed`: No motion between links
	- `continuous`: Rotation about axis, no limit
	- `revolute`: Rotation around axis, with limits
	- `prismatic`: Translation about an axis
	- `floating`: 6 DOF
	- `planar`: Motion on a plane
- `origin` describes the location of the child frame relative to the parent frame.
- `axis` describes the axis of rotation for the joint
- `limit` is for `revolute` and `prismatic` joints only.
- `dynamics` includes the damping (expressed in N s/m) and friction (expressed in N).

## What we need
- A single `stl` of the entire robot
- `sdf` 

| Links | Joint | 
| :---- | :---- |
| IMU                           | fixed |
| DVL                           | fixed |
| Each camera (separate link)   | fixed |
| Each propeller                | continuous |


## SDF
Convert a `URDF` to `SDF`:
```sh
gz sdf -p ./lazershark_sim.urdf > ./lazershark_sim.sdf
```