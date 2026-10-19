1. # Flipped Lidar and filtered out points reflected from the robot itelf:

        In sage-0.1.0/src/terrascout_description/urdf/scout.urdf.xacro

            <!-- <min_angle>-0.1222</min_angle>
            <max_angle>0.9076</max_angle> -->

            <!-- Vertical angles for the upsidedown mid360 lidar -->
            <min_angle>-0.9076</min_angle>
            <max_angle>0.1222</max_angle>


2. # adding uneven terrain in simulation

    Run createUnevenGround.py to get forest_terrain.obj in src/fleet_simulation/terrain.
    In src/fleet_simulation/worlds/forest.sdf we added a new segment (see comments in that file)




3. # To install STVL, this was changed in Dockerfile and Makefile.


    Added git link in third_party.repos. Also changed a few parameters related to STVL in nav2.yaml file

    # In Dockerfile:
        See comments in Dockerfile (line 33-50).

    # In Makefile:
    org. line 173 in makefile:
        -e CC="ccache gcc" -e CXX="ccache g++" \
    New line 173:
        -e CC=gcc -e CXX=g++ \

    org. line 177 in makefile:
        colcon build $(COLCON_INSTALL) --parallel-workers $$(nproc) $(COLCON_SKIP) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    New line 177 in makefile:
        colcon build $(COLCON_INSTALL) --parallel-workers $$(nproc) $(COLCON_SKIP) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
PHYSICAL_ONLY



4. # Implementing FAST-LIO2
    Added git link in third_party.repos

    # In Dockerfile:
        see comments on lines 53-64, and 193.

    # In Makefile:
        added:
            PHYSICAL_ONLY_PKGS := terrascout_hardware gscam2 fpsdk_common fpsdk_apps fpsdk_ros2 fixposition_driver_lib fixposition_driver_ros2 fixposition_driver_msgs rtcm_msgs
        on line 62 (and commented out line 61). This will make the livox drivers to be included in simulation aswell (needed for Fast-Lio2)

        added: 
            FAST_LIO_DIR := $(THIRD_PARTY_DIR)/FAST_LIO_ROS2
        on line 22 (see comment saying "New for FAST-LIO2)

        added:
            @git -C $(FAST_LIO_DIR) submodule update --init --recursive include/ikd-Tree \
	    	|| { echo "build failed: operation: initialize FAST-LIO2 ikd-Tree submodule; cause: reported above" >&2; exit 1; }
        on line 139-140

    # Additional changes
        added:
            new folder and file; patches/FAST_LIO_ROS2/imu_qos.patch

        added:
            new file; terrascout_navigation/config/fast_lio_sim.yaml

        added:
            new file src/terrascout_navigation/terrascout_navigation/fast_lio_odom.py

        added: 
            new FAST-LIO2 output topics to foxglove in foxglove_bridge.yaml (line 41-45) 

        added:
        in src/fleet_config/fleet_config/classes/terrascout.py this is added
            FAST_LIO = "fast_lio"
            FAST_LIO_SIM_CONFIG = SourceFile(NAVIGATION_PACKAGE,"config/fast_lio_sim.yaml")
            entries.append(fast_lio_simulation(unit))
            files["fast_lio_sim.yaml"] = FAST_LIO_SIM_CONFIG
            entries.append(fast_lio_odom_adapter(unit, [FAST_LIO]))

            and 

            function def fast_lio_simulation()
            function def fast_lio_odom_adapter()

        added: 
            terrascout_navigation/fast_lio_odom.py in CMakeLists.txt

        changed:
            commented out mapping for /odom gazebo bridge (using fast-lio instead)
            commented out a part in def _odom_cb in src/terrascout_navigation/terrascout_navigation/gnss_shim.py




        




