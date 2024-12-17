# DTU Digital Control

### Report

TODO


### Full Simulink 

![complete_simulink](https://github.com/user-attachments/assets/f4544f6c-ecb2-44b9-8997-ee3d7ebd6c4d)


### Demo

https://github.com/user-attachments/assets/112950e6-941c-443e-a6e3-5e666a00de33


### File structure

- The `matlab` folder contains the Simulink and Matlab files required for design, tuning and simulation.
  - The main code simulation is run by `workspace.m`, which refers to the model `final_robot_model.slx`
  - The file `get_ref.m` is the reference generator
- The C++ code is instead contained in the `basebot` folder.
  - The main is `src/basebot_6.ino`
  - Other files are in `src/utils/`
