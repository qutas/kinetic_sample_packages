## qutas_sample_packages
#### Notes
- all using std_msgs
- no external dependencies outside ros-comm (?)
- python and c++ examples for both
#### Nodes
- **qutas_sample_listener**
  - subscriber
  - spinner
- **qutas_sample_controller**
  - timer
  - publisher
- **qutas_sample_interface**
  - publisher
  - basic sleep / loop
- **qutas_sample_offload**
  - service server
  - spinner
  - **qutas_sample_offload_test**
    - once-off
    - merge with server package as a simple node?
- **qutas_sample_dynamic_params**
  - launch file
  - yaml example
  - dynamic parameters with defaults
  - once-off

## qutas_sample_packages_extras
#### Notes
- preface with dependencies for each package
- python and c++ examples for both
#### Nodes
- **qutas_sample_mavros_guider**
  - spinner
  - position callback
  - 2 waypoints
  - Use proper waypoint management
- **qutas_sample_cam_opencv**
  - spinner
  - using the camera subscriber?
  - cv_bridge
- **qutas_sample_solve_pnp**
  - spinner
  - extend off of previous node?
- **qutas_sample_database_conn**
  - spinner
  - 2 subscribers

## qutas_navigation_stack
#### Notes
- is this a good idea?
