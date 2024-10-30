# Gazebo Plugins Documentation

## Buoyancy Plugin
### Overview

`lazershark_sim` uses the `graded_buoyancy` plugin to model the density of water and "air". 

- **Surface Level:** The surface as the plane $ z = 7 \, \text{ft} $ (depth of the RoboSub pool).
- **Density Below Surface:** The fluid density below the surface is set to $ 1000 \, \text{kg/m}^3$ (density of freshwater).
- **Density Above Surface:** The density above the surface is $ 1 \, \text{kg/m}^3 $. Since the AUV will remain underwater, this value does not need to be precise, only significantly lower than water.

### Implementation Notes

- **Buoyant Force Calculation:** The buoyancy plugin calculates the buoyant force based on the `<collision>` elements which determine the volume of fluid displaced by the model.
  
- **Specific Link Configuration:** The link to which buoyancy should apply can be specified. In our `lazershark_sim.sdf` model, we use a collision element named `collision_bouyancy` to define the displacement volume:
  
  - **Volume Consideration:** Set the volume to be slightly larger than $ v = \frac{m}{\rho} $, for slight positive buoyancy.
  - **Center of Buoyancy:** The collision element's pose should be positioned at the model's center of buoyancy.
  
  ```xml
  <collision name='collision_bouyancy'>
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
- Example use of the graded bouyancy plugin: [graded_buoyancy.sdf](https://github.com/gazebosim/gz-sim/blob/gz-sim9/examples/worlds/graded_buoyancy.sdf)