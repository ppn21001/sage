1. # Flipped Lidar and filtered out points reflected from the robot itelf:

        In sage-0.1.0/src/terrascout_description/urdf/scout.urdf.xacro

            <!-- <min_angle>-0.1222</min_angle>
            <max_angle>0.9076</max_angle> -->

            <!-- Vertical angles for the upsidedown mid360 lidar -->
            <min_angle>-0.9076</min_angle>
            <max_angle>0.1222</max_angle>


2. # adding uneven terrain in simulation

    Run createUnevenGround.py to get forest_terrain.obj in src/fleet_simulation/terrain
    In src/fleet_simulation/worlds/forest.sdf we added a new segment (see comments in that file)




3. # To install STVL, this was changed in Dockerfile and Makefile

    See comments in Dockerfile (line 33-50).

    In Makefile:
    # org. line 169 in makefile
    -e CC="ccache gcc" -e CXX="ccache g++" \
    # New line 169 
    -e CC=gcc -e CXX=g++ \
    # org. line 173 in makefile
    colcon build $(COLCON_INSTALL) --parallel-workers $$(nproc) $(COLCON_SKIP) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
    # New line 173 in makefile
    colcon build $(COLCON_INSTALL) --parallel-workers $$(nproc) $(COLCON_SKIP) --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \



