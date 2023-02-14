# Quadrotor Simulator

Basic quadrotor simulator using ode45 and an LQR controller.

## How to use
1. Add `helperFiles`, `storedData`, and `images` to your paths 
    - select all -> right click -> add to path -> selected folders)

### Simulation:
1. Open the `cf2_sim.m` script.
2. Edit the user settings
    - hover point: `px`, `py`, `pz` indicate the coordinates of the hover point
    - maximum simulation duration: `t_run` in seconds
    - video name: `video_name` 
    - select whether or not to make and save a video: `make_video`
3. Run the script

### Impulse response and bode plots:
1. Open `cf2_transfer_function_plots`
2. Choose whether to save the figures or not (`save_figures`)
3. Run the script
