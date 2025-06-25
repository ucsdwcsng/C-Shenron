# SHENRON: Radar Simulation
Packaging shenron into minimal working code

To run the simulation, follow these steps:

1. Open the `simulator_config.yaml` file.
2. Add the file paths for the input and output files in the appropriate fields. For example:

   ```yaml
   PATHS :
     LIDAR_PATH : "/home/Kshitiz/"
     LIDAR_FOLDERS : ["semantic_lidar"]
     OUT_PATH : "/home/Kshitiz/semantic_lidar/"
3. Run the simulation using main file
   ```python
   python main.py
