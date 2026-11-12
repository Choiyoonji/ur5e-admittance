# Hand-guiding admittance control for a UR5e

Project page: **https://choiyoonji.github.io/ur5e-admittance/**

A 500 Hz admittance controller on a UR5e's built-in wrist force/torque sensor, designed, tuned
and analysed on the real robot: sensor characterisation, controller design, open-loop system
identification and stability analysis.

By [Yoon-Ji Choi](https://choiyoonji.github.io/), Interactive Robotics Lab, Dongguk University.

This repository holds only the web page (HTML, figures, videos and the in-page push simulation in
`sim/`). The robot footage is a 3D replay of logged real-robot runs (measured joint angles, hand
force and controller state). The simulation runs the controller's 500 Hz cycle against a robot
model fitted to the open-loop identification data, drawn with three.js (MIT, vendored in
`sim/vendor/three`) and the UR5e meshes (Universal Robots, BSD-3-Clause, simplified into
`sim/ur5e.glb`).
UR5e meshes in the videos: Universal Robots, BSD-3-Clause.
