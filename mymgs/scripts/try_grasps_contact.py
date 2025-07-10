# Copyright (c) 2025 Robert Bosch GmbH
# Author: Roman Freiberg
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import hydra
import numpy as np
from omegaconf import DictConfig
from copy import deepcopy
import os

import time

import traceback



from mgs.env.gravityless_object_grasping import GravitylessObjectGrasping
from mgs.env.selector import get_env
from mgs.gripper.selector import get_gripper
from mgs.obj.marker import Marker
from mgs.util.geo.transforms import SE3Pose

from mgs.obj.ycb import ObjectYCB
from mgs.obj.selector import get_object




def get_grasps(gripper_name, obj_id, gripper_type=""):
    grasp_path = os.path.join(  # type: ignore
        os.getenv("MGS_INPUT_DIR"),  # type: ignore
        gripper_name + "-" + gripper_type,
    )
    all_files = os.listdir(grasp_path)
    files_with_obj_id = [file for file in all_files if file.startswith(f"{obj_id}_")]
    if len(files_with_obj_id) == 0:
        return None

    grasp_vecs = []
    for file in files_with_obj_id:
        current_file_path = os.path.join(grasp_path, file)
        grasp_dict = np.load(current_file_path)
        grasps_vec = grasp_dict["grasps"]
        grasp_vecs.append(grasps_vec)

    merged_grasps = np.concatenate(grasp_vecs, axis=0)
    return SE3Pose.from_vec(merged_grasps, layout="pq", type="wxyz")


def setup(cfg, gripper):

    myObj = get_object(cfg.object_id)
    
    grasps = get_grasps(
        gripper_name=cfg.gripper.id,
        gripper_type=cfg.gripper.get("grasp_type", ""),
        obj_id=myObj.object_id,
    )

    scene = GravitylessObjectGrasping(gripper, myObj)

    if cfg.grasp_id == "all":
        print("showing all grasps...")

        for i in range(len(grasps)):

            try:
                contacts, finger = scene.find_contacts(grasps[i])

                print(f"contacts= {contacts}")
                print(f"finger= {finger}")

            except (Exception, KeyboardInterrupt) as e:
                print(f"Error occurred while trying grasp {i}: {e}")
    else:
        print(f"showing grasp {cfg.grasp_id}...")
        try:
            grasp_id = int(cfg.grasp_id)
            scene.find_contacts(grasps[grasp_id], viewer=True)
        except (IndexError, Exception) as e:
            if isinstance(e, IndexError):
                print(f"Invalid grasp ID: {cfg.grasp_id}. Please provide a valid index between 0 and {len(grasps) - 1}.")
            else:
                raise
    
    time.sleep(10)


@hydra.main(config_path="config", config_name="try_grasps")
def main(cfg: DictConfig):
    gripper = get_gripper(cfg.gripper)
    setup(cfg, gripper)


if __name__ == "__main__":
    main()
