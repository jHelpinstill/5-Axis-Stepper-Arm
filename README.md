# 5-Axis-Stepper-Arm
An arduino-based project I wrote to control a 5-axis robot arm with stepper motor actuators

This project was written to control a 5-axis CNC arm I was designing in July 2021. I intended for it to automate a process at my father's company which involved drilling holes in five faces of a small metal box, so it was designed specifically to position a spindle motor and drill bit around the faces of this box. The project ultimately ended after I realized that the machine would not be stiff enough to maintain sufficient accuracy under the load of drilling, but I would consider this controller software to have been a success.

Some important aspects of the code:

- Inverse kinematics: user can specify (x, y, z) position and (a, b) tool angle
- Velocity and Acceleration control of tool head
- Manual control through serial monitor command line interface: Planned to add GRBL compatibility, but never finished

I wouldn't recommend using this code for any actual projects, other than as inspiration. I'm just uploading it to a repo for posterity. For that reason, I'm not adding a license to this code.
