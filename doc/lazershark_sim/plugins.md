# Gazebo Plugins Documentation

## Buoyancy Plugin
### Overview
The buoyancy force opposes gravity exerted on the robot. The parameters of the plugin are configured via SDF in the world file. The `graded_buoyancy` plugin is used to model two discrete densities, in this case the density of water and "air". 

- **Surface Level:** The surface is defined as the plane $ z = 7 \, \text{ft} $ (depth of the RoboSub pool).
- **Density Below Surface:** The fluid density below the surface is set to $ 1000 \, \text{kg/m}^3$ (density of freshwater).
- **Density Above Surface:** The density above the surface is $ 1 \, \text{kg/m}^3 $. Since the AUV will remain underwater, this value does not need to be precise, only significantly lower than water.

### Implementation Notes

- **Buoyant Force Calculation:** The buoyancy plugin calculates the buoyant force based on the `<collision>` elements which determine the volume of fluid displaced by the model.
  
- **Specific Link Configuration:** The link to which buoyancy should apply can be specified. In our `lazershark_sim.sdf` model, we use a collision element named `collision_buoyancy` to define the displacement volume:
  
  - **Volume Consideration:** Set the volume to be slightly larger than $ v = \frac{m}{\rho} $, for slight positive buoyancy.
  - **Center of Buoyancy:** The collision element's pose should be positioned at the model's center of buoyancy.
  
  ```xml
  <collision name='collision_buoyancy'>
      <pose>0 0 0.1 0 0 0</pose>
      <geometry>
          <box>
              <size>0.2502 0.2502 0.2502</size>
          </box>
      </geometry>
  </collision>
  ```

- **Supported Geometries:** When configured in graded buoyancy mode, the plugin supports only `<box>` and `<sphere>` geometries for collision elements.

### References
- Gazebo documentation: [The buoyancy plugin](https://gazebosim.org/api/sim/8/theory_buoyancy.html)
- Example use of the graded buoyancy plugin: [graded_buoyancy.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim9/examples/worlds/graded_buoyancy.sdf)
- Class documentation: [Buoyancy Class Reference](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1Hydrodynamics.html)


## Hydrodynamics Plugin
### Overview

Hydrodynamics refers to the behavior of bodies in water. It includes forces like linear and quadratic drag.


### Parameters
| **Parameter** | **Description** | **Units** | **Using** | 
| ---- | ---- | ---- | :---: | 
| `<xDotU>` | Added mass in x direction | `[kg]` | 
| `<yDotV>` | Added mass in y direction | `[kg]` | 
| `<zDotW>` | Added mass in z direction | `[kg]` | 
| `<kDotP>` | Added mass in roll direction | `[kgm^2]` | 
| `<mDotQ>` | Added mass in pitch direction | `[kgm^2]` | 
| `<nDotR>` | Added mass in yaw direction | `[kgm^2]` | 
| `<xUabsU>` |  Quadratic damping, 2nd order, x component | `[kg/m]` |  | 
| `<xU>` | Linear damping, 1st order, x component | `[kg]` | X |
| `<yVabsV>` | Quadratic damping, 2nd order, y component | `[kg/m]` |  |
| `<yV>` | Linear damping, 1st order, y component | `[kg]` | X | 
| `<zWabsW>` | Quadratic damping, 2nd order, z component | `[kg/m]` |  |
| `<zW>` | Linear damping, 1st order, z component | `[kg]` | X |
| `<kPabsP>` | Quadratic damping, 2nd order, roll component | `[kg/m^2]` |  | 
| `<kP>` | Linear damping, 1st order, roll component | `[kg/m]` | X  |
| `<mQabsQ>` | Quadratic damping, 2nd order, pitch component | `[kg/m^2]` |  | 
| `<mQ>` | Linear damping, 1st order, pitch component | `[kg/m]` | X | 
| `<nRabsR>` | Quadratic damping, 2nd order, yaw component | `[kg/m^2]` |  | 
| `<nR>` | Linear damping, 1st order, yaw component | `[kg/m] ` | X | 

### Implementation Notes

- **Absolute Values:** Ensure the parameters use the absolute value version, `<xUabsU>` instead of `<xUU>`
- **Duplicate Parameters:** Do not use any of the added mass parameters, as this duplicates the inertial values specified in the `base_link`.
- **Quadratic damping** The sim currently does not use quadratic damping for simplicity. 

### References
- Class documentation: [Hydrodynamics Class Reference](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1Hydrodynamics.html)
- Example use of the hydrodynamics plugin: [acoustic_comm_demo.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/acoustic_comms_demo.sdf)

## Thruster Plugin
### Overview

The thruster plugin simulates thrusters for underwater vehicles. Reads a force in Newtons from a topic and applies at the location of a joint.

### Implementation Notes
Joints must be specified in model `.sdf` as either fixed or revolute. 
Values for each thruster must be sent on different topics.

### Example Usage
```xml
<plugin
    filename="gz-sim-thruster-system"
    name="gz::sim::systems::Thruster">
    <namespace>thruster_values</namespace>
    <joint_name>thruster_joint_0</joint_name>
    <topic>thruster_0</topic>
    <propeller_diameter>0.0762</propeller_diameter>
</plugin>
```

### References
- Class documentation: [Thruster Class Reference](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1Thruster.html)

## IMU Sensor
### Overview
The Inertial Measurement Unit (IMU) sensor outputs (to a specified topic) the simulated robot's orientation as a quaternions.
### Implementation Notes
The sensor must be defined within the `<world>` tag. Note this is likely in a separate file than the model:
```xml
<plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
</plugin>
<plugin 
    filename="gz-sim-imu-system"
    name="gz::sim::systems::Imu">
</plugin>
```
The IMU itself must be defined within a `link` (apart of the model itself). The IMU link should be positioned at the actual `pose` of the physical IMU. Note the mass attribute is required and must be greater than zero, however it may be made extremely small. Within the link, the imu sensor is defined as follows:
```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <topic>imu/sim</topic>
</sensor>
```
### References
- Sensor sdf documentation: [Sensor Element](http://sdformat.org/spec?elem=sensor)
- Sensors tutorial: [Sensors](https://gazebosim.org/docs/latest/sensors/)