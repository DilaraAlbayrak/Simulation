# 700105-2425-FinalLab

The purpose of this assessment is to design and implement an efficient network-based simulation of a series of physics-based scenarios. 

## Concept : Networked AntiGravity Chamber

You are required to create a distributed "simulation" for FOUR users.

Your simulation is to include a number of scenarios. 

In each scenario, objects will be randomly assigned to one of the four users - and will be coloured according to that user - Red, Green, Blue and Yellow.

Scenarios are to be changed without restarting the application.

Your simulation is to make efficient use of the compute capacity within the PCs in RBB-335.

## Mandatory Simulation Features

These features, when properly implemented, will provide approximately 30% of the final mark. In additon, the list of features are a guide to the building blocks and functionality expected of the solution.

### Newtonian Physics

- Objects are treated as particles with dimensions. They do not rotate.
- When gravity is applied, objects should move according to appopriate gravitational force.
- Linear Newtonian physics is simulated with a fixed timestep using Euler integration.


### Collision Detection

- Collision detection between moving spheres and fixed spheres.
- Collision detection between moving spheres and fixed capsules or fixed cylinders.
- Collision detection between moving spheres and moving spheres.

### Collision Response

- Inelastic collision response between moving spheres and fixed objects.
- Inelastic collision response between moving spheres and moving spheres with the same mass.

### Mandatory Scenarios

The scenarios are consists of a number of fixed and moving objects. 

Moving objects should experience appropriate gravitational force whilst gravity is being applied.

All moving objects should move, collide with and bounce off of the fixed objects and other moving objects. 

All scenarios should take place inside a 3m x 3m x 3m cube. Moving objects should collide with and bounce off of the walls of the cube.

When a scenario uses the term *many* to indicate the number of objects, it implies that you decide the quantity your application can include and still be able to run at a reasonable framerate.  For example *many* in a simple simulation might be as high as 10k - 100k.

#### Scenario 1

Add moving spheres one by one from a random points above a scene filled with several fixed spheres with different positions and radii.

#### Scenario 2

Add moving spheres one by one from a random points above a scene filled with several fixed capsules or cylinders with different positions, radii, lengths and orientations. 

#### Scenario 3

Add *many* balls from random points in a scene with no fixed objects (apart from the surrounding cube). Moving spheres should be of different radii.

#### Scenario 4

Add *many* balls from random points above a scene filled with several fixed spheres with different positions and radii, and fixed capsules or cylinders with different positions, radii, lengths and orientations.

In addition to these scenarios you should include your own small well defined scenarios to demonstrate specific elements of your solution that you use to verify your physics is working correctly. For example a direct collision between a moving ball and a stationary ball of the same mass.

### User Interface Controls

A user interface is to be implemented using ImGui.

The following are to be displayed:

- The total number of objects within the simulation, by type (*).
- Target delta time within the simulation step (in secs), minimum 0.005s [changeable in 0.005s intervals] (*)
- Target frequency of the simulation (in Hz), minimum 1Hz [changeable in 1Hz intervals] (*)
- Actual frequency of the simulation (in Hz),
- Target frequency of the graphics (in Hz), minimum 1Hz [changeable in 1Hz intervals]
- Actual frequency of the graphics (in Hz),
- Target frequency of the networking (in Hz), minimum 1Hz [changeable in 1Hz intervals] (*)
- Actual frequency of the networking (in Hz)

Interactions are required to:
- Adjust all targets specificed above.
- Pause the simulation (*)
- Turn on or off gravity (*)
- Reverse gravity (*)

Attributed marked with an (*) are considered part of the global state and should be consistant across all machines within the distributed simulation.

### Time
 
Time in the game is to be calculated per simulation loop (delta_time) and used in the physics equations. 

This feature will be very useful when debugging your PBM

## Advanced Simulation Features

These features, when properly implemented, will provide approximately 20% of the final mark.

### Newtonian Physics

  - Include at least 2 additional integration methods and be able to switch between integration methods locally at any time.
  - Moving objects have angular velocity.
  
### Collision Detection

  - Add collision detection for fixed cubes
  - Add a method for spatial segmentation to reduce unnecessary calculation

### Collision Response

  - Spheres have different masses, where the collision response between moving spheres takes into account the mass of each sphere.
  - Objects have materials with elasticity. Collision response between moving spheres takes into account elasticity between different pairs of materials.
  - Moving spheres should be modelled as rigid bodies and therefore rotation should be fully modelled within the physics equations including inertia and friction.
  - Moving capsules or cylinders should be modelled as rigid bodies and therefore rotation should be included within the physics equations along with inertia and friction.
  - Moving cubes modelled as rigid bodies and therefore rotation should be included within the physics equations along with inertia and friction.
  - Cubes can move and should be modelled as rigid bodies and therefore rotation should be fully modelled within the physics equations including inertia and friction.

### Advanced Scenarios

As a result of the advanced simulation features you may need to add additional scenarios. e.g. if you have moving cylinders you should add moving cylinders to your scene.

Note: Marks for any advanced features will be greatly reduced if the advanced feature is not included in a suitable scenario.

### Additional User Interface Controls

As a result of the advanced simulation features you may need to add additional user interface controls to ImGui e.g. You may also need to assign material properties to objects. The material properties should be aligned to the colour of a material.

- User interface elements to set the coefficient of elasticity for pairs of materials. (*)
- User interface elements to set static and dynamic friction for pairs of materials. (*)
- User interface elements to set the density of materials. (*)

## Distributed Architecture

The application is to be designed as a distributed system. Each node (peer) within the system will have its own dedicated graphics, dedicated physics and a common state. The rendered images displayed on each peer must be identical.

A peer-to-peer network infrastructure is required with a minimum of two peers.

**distributed ownership**

Three or more peers can be implemented (Advanced).

In addition to the objects within each scenario, all atttributes identified as belonging to the global state should also be synchronised across all machines.  Thus when a user interacts with one peer, that interaction is sent to all other peers.

elements of the simulation can be implemented on the GLU using Compute Shaders (Advanced)

A client-server architecture must not be implemented, and will be penalised.

### Data integrity

The application should ensure that the data stored in each peer is as accurate as possible. With the Simulation being performed on each peer, the data will inevitably begin to drift. The application should seek to address any inconsistencies. Correction algorithms will be required e.g. interpolation.

The process of correcting the data will itself create more communications, as the corrections are distributed to other peers.

Marks will be awarded for the smoothness of the correction process, from the user's perspective (Advanced).

### Networking

Either TCP or UDP network protocols can be used to transfer the data between nodes.

The minimum requirement is two peers on separate physical PCs in RBB-335.

A network simulation tool will be inserted between the peers to simulate network latency and packet loss. Your application must be able to cope with the following worst-case network quality of service:

Latency: 100ms ± 50ms

Packet loss: 20%

The IP address and ports should be stored in a configuration file.

Preferably, a UDP broadcast mechanism can be used to identify peers on the network.

## Parallel Architecture

The major components within each application are required to operate asynchronously. The choice of threading architecture is not defined.

As part of the final demonstration, you will be required to run various parts of your system at different frequencies. For example, the graphics may be requested to be run at 30 frames per second (Hz), whilst the Simulation at 1000 Hz.  This is controlled by ImGui as previously specified.

This ability of individual components to run asynchronously is a major part of this assessment. Failure to implement this correctly will greatly reduce your marks.

### Process Affinity

In order to ease debugging and to simplify the assessment process, you will be required to use a very specific thread to processor mapping.

Core Application

- Visualization - core 1
- Networking - core 2
- Simulation - core 3+

You can use any number of threads in your applications provided you have sufficient to meet the processor mapping in the table above. For example, it is acceptable to use 4 networking threads on the server, provided that they are all located on core 2.

## Implementation

The software MUST be demonstrated on the PCs in RBB-335.

### Graphics

Only simple graphics are required, but you may choose either DirectX 11 or DirectX 12.

### Simulation

You are to implement all of the physics. You may use the example code that has been provided to you in the lab work.

Only DirectX Math library is permitted.

### Networking

Only the Winsock 2 library is permitted.

### Threads

Either the Win32 or C++ 23 threading library is permitted.

The PC's in RBB-335 are considered the target platform. Threading is to be used to leverage the performance of these processors.

## Report

The structure of the ACW report is as follows:

- System architecture, including where threads and networking have been used (1000 words max), plus UML diagrams
- How the motion physics has been implemented for the balls, and how the collision detection and response has been implemented between the balls and the other elements in the scene (1000 words max)

Marks will be lost if the word limit is exceeded.

## Mark scheme

A detailed mark scheme will be provided.
