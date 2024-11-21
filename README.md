# NXP AIM 2024 Autonomous Driving Competition

This repository contains the code and assets developed for the NXP AIM 2024 competition. 

## Repository Structure  

### Components  

1. **b3rb_ros_edge_vectors**  
   Detects and defines edge boundaries of the track using OpenCV.  
   
2. **b3rb_ros_line_follower**  
   Implements obstacle avoidance and path tracking logic for navigating the track.  

3. **b3rb_ros_object_recog**  
   Performs recognition of stop signs using advanced computer vision models.  

### YOLOv8 Weights  

- **best(1)(1).pt** and **best(4).pt**  
   YOLOv8 model weights trained for recognizing critical objects on the track, including stop signs.  

### Configuration Data  

- **model_config.json**  
   Contains metadata and configuration details for all obstacles, including their precise locations on the map.  
