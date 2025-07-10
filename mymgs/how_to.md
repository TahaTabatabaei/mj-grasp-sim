# How to make grasp and visualize it for an unseen object:

1. Add a folder with the desired name in the YCB dataset folder
    1. Place your original .obj file there. Becareful that `mj-grasp-sim` treat v values with meter unit. if your object size is for example 5cm, you should write it like:\
    `v 0.05 0.05 0.05`
    
    2. Place the .mtl and it's respective .png texture in the same folder.

2. Using the [V-HACD tool](https://github.com/kmammou/v-hacd), generate v-hacd convex pieces for the object. Results will be placed in the same folder that you run the command.\
This is a sample:
    ```
    tahaos@tahaos-ROG-Strix-G531GT-G531GT:~/Code/Projects/mj-grasp-sim/asset/mj-objects/YCB/Cube_BodyPad$ ./../../../../../acrambly2/v-hacd/app/build/TestVHACD textured.obj
    ```
    Expected result:
    ```
    [COMPUTE_BOUNDS_OF_INPUT_MESH            ] : 0% : 100% : ComputingBounds
    [CREATE_RAYCAST_MESH                     ] : 20% : 100% : RaycastMesh completed
    [VOXELIZING_INPUT_MESH                   ] : 30% : 100% : Voxelization complete
    [BUILD_INITIAL_CONVEX_HULL               ] : 40% : 100% : Initial ConvexHull complete
    [INITIALIZING_CONVEX_HULLS_FOR_MERGING   ] : 60% : 100% : ConvexHull initialization complete
    [FINALIZING_RESULTS                      ] : 90% : 100% : Finalized results
    Convex Decomposition took 0.00000 seconds
    VHACD::Complete
    Computing Convex Decomposition took 0.54063 seconds
    Saving Convex Decomposition results of 1 convex hulls to 'decomp.obj'
    Saving convex hull results to a single file 'decomp.stl'
    ```

3. You also need a `composite.obj` file.\
    ***I did this manually just for test purposs, but I highly recommend    you to use tools like Blender. It is really time consuming and bug  prone to do it manually for complex objects***
    1. Make a new blank file with the .obj extention.
    2. From the `textured.obj`, copy everything and past it insidde the composite file.
    3. Inside the composite file, one line before\
    `usemtl texture_map`
    add this:
    `o visual`
    Now it should looks like this:
    ```
    mtllib textured.mtl

    o visual
    usemtl texture_map

    v 0.025000000 -0.000000 0.000000
    ...
    ```
    4. At the end of the file, add this in a new line:\
    `o collision`
    5. Now copy the contents of the `decompose.obj` generate by the from where is starts the `v` vaules till the end. Do not copy things like:
    ```
    mtllib decomp.mtl
    o textured000
    usemtl Material000
    ``` 

    6. This part is a little bit tricky. .obj files use global indexing for vertices (v), texture coordinates (vt), and normals (vn). After adding new v lines for the collision  (decompose.obj), your face definitions like:
    ```
    f 1 2 3
    f 4 5 6
    ...
    ```
    But we also have the f values from the visual mesh in the upper section. What you need to do is: **Renumber all vertex indices in the collision mesh section to start after the last vertex from the visual mesh.**
    So for example if:
    ```
    Visual mesh: has 8 vertices → indexed 1–8
    ```
    Then:
    ```
    Collision mesh: add 8 more → should start at index 9
    ```
    Now your face values looks like this:
    ```
    f 9 10 11
    f 12 13 14
    ...
    ```
   
4. Generate the info.yaml file.
    Look at the sample below to understand how to set vaules for each parameter.
    ```
    composite_file: composite.obj
    material_file: textured.mtl
    material_map: texture_map.png
    obj_disp_name: Cube_BodyPad  
    obj_name: Cube_BodyPad              → name of the folder   
    original_file: textured.obj
    submesh_files:
    - decomp.obj                        → if you have more than one collision mesh file, list them all here 
    submesh_props:
    - 1.0
    weight: 0.094
    ```

5. Now everything should be Ok. Run the `python -m mgs.cli.gen_grasps gripper={desired gripper name} object_id={desired object id (name of the object folder)}`.\
It should generate grasps for the desired object (your new unseen object in this case)

6. To visualize the graps, use the `mymgs/scripts/try_graps.py` script.
    1. Copy the .npz file generated from the `mymgs/MGS_OUTPUT_DIR` to the `mymgs/MGS_INPUT_DIR`.
    2. Run `python try_grasps.py gripper={desired gripper name} object_id={desired object id (name of the object folder)}`.\
    Names should match the names in step 5.





