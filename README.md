# CARLA_Localization_and_Mapping
Fusion of LiDAR and GPS data for 2 dimesnional pose estimation using a Kalman Filter.



State is defined as,  $X ={x,y,\theta,x^\prime,y^\prime,\theta^\prime}$

### 
![fusion](https://github.com/nsteve2407/CARLA_Localization_and_Mapping/blob/main/carla_demo.gif)  

### Dependencies
- Python 3.6+
- Open3D
### Useage

- Launch CARLA simulator in headless mode.
- Run main.py
- Log files are saved as csv files, containing state estimates and groundth truth values. 
