                         LiDAR + IMU
                              │
                    filtering / transforms
                              │
             ┌────────────────┴────────────────┐
             │                                 │
       OBSTACLE PIPELINE                 TERRAIN PIPELINE
             │                                 │
   ground_segmentation_ros2                 FAST-LIO2
             │                                 │
       obstacle_points                 registered cloud
             │                                 │
           STVL                       elevation_mapping_cupy
     (Nav2 costmap layer)                      │
             │                              GridMap
             │                                 │
             │                       terrain features
             │                     slope / roughness /
             │                       step / uncertainty
             │                                 │
             │                     traversability estimate
             │                                 │
             │                    custom Nav2 terrain layer    <-- (nav2_elevation_layer ?)
             │                                 │
             └───────────────┬─────────────────┘
                             ↓
                    Nav2 layered Costmap2D
                             │
                        Inflation layer
                             │
                    Planner / Controller