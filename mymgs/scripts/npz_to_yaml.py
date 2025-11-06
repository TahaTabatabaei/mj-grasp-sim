import numpy as np
import matplotlib.pyplot as plt
import yaml


def main():

    data = np.load('/home/tahaos/Code/Projects/mj-grasp-sim/mymgs/MGS_OUTPUT_DIR/Cube_Pad_628e53fd7c9ddb43a6adb9a32d4991bf.npz')

    object_name = data['object_id']

    grasps = data['grasps']

    yaml_data = {'grasps':[]}

    for i, grasp in enumerate(grasps):
        grasp_dict = {
            'id': i,
            'orientation':{
                'w': float(grasp[0]),
                'x': float(grasp[1]),
                'y': float(grasp[2]),
                'z': float(grasp[3])
            },
            'position':{
                'x': float(grasp[4]),
                'y': float(grasp[5]),
                'z': float(grasp[6])
            }
        }
        yaml_data['grasps'].append(grasp_dict)

    destination_folder = "/home/tahaos/Code/Projects/mj-grasp-sim/mymgs/yamls"

    with open(f'{destination_folder}/{object_name}_grasps.yaml', 'w') as f:
        yaml.dump(yaml_data, f)

    print(f"Grasps saved to {destination_folder}/grasps.yaml")


if __name__ == "__main__":
    main()
