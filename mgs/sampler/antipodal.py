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

import numpy as np
import trimesh
from scipy.stats import vonmises_fisher

from mgs.obj.base import CollisionMeshObject
from mgs.sampler.base import GraspGenerator
from mgs.util.geo.transforms import SE3Pose


# AntipodalGraspGenerator generates antipodal grasps for a 3D object mesh.
# It samples contact points and computes valid 6-DoF grasp poses using surface normals and geometric reasoning.
class AntipodalGraspGenerator(GraspGenerator):
    def __init__(self, object: CollisionMeshObject):
        # Initialize with the target object mesh
        super().__init__(object)

    def denorm_grasp_pose(self, Hs: np.ndarray):
        # Convert grasp poses from normalized space back to original mesh scale and position
        position = Hs[..., :3, 3]  # Extract translation component
        position -= self.offset    # Undo centering offset
        position *= self.scale     # Undo scaling
        Hs[..., :3, 3] = position  # Update translation component
        return Hs

    def normalize_load(self):
        # Load and normalize the object mesh
        mesh = trimesh.load_mesh(self.mesh_file_path)
        if not isinstance(mesh, trimesh.Trimesh):
            raise TypeError("Loaded mesh is not a trimesh.Trimesh object")

        self.scale = float(mesh.scale)  # Store original scale
        mesh.apply_scale(1.0 / self.scale)  # Normalize to unit scale

        offset = np.eye(4)
        self.offset = -mesh.centroid  # Compute offset to center mesh
        offset[:3, 3] = self.offset
        mesh.apply_transform(offset)  # Apply centering
        self.mesh = mesh  # Save normalized mesh

    def generate_grasps(self, num, kappa=10):
        # Main function to generate 'num' antipodal grasp poses
        self.normalize_load()  # Load and normalize mesh
        contact_pairs_one = []  # First contact point list
        contact_pairs_two = []  # Second contact point list

        while len(contact_pairs_one) < num:
            # Sample twice the number of surface points needed
            surface_points, _ = trimesh.sample.sample_surface(self.mesh, 2 * num)  # type: ignore

            # Compute surface normals for sampled points
            _, distance, triange_id = trimesh.proximity.closest_point(
                self.mesh, surface_points
            )
            normals = self.mesh.face_normals[triange_id]

            # sample normals from mises_fisher distribution
            random_co_approaches = np.zeros((2 * num, 3))
            for i in range(2 * num):
                random_co_approaches[i, :] = vonmises_fisher.rvs(
                    mu=-normals[i, :], kappa=kappa, size=1
                )[0]

            # Duplicate surface points and stack into contact pairs
            contact_points = np.stack((surface_points, surface_points), axis=1)
            contact_points = contact_points.reshape(-1, 3)

            # Create mirrored approach vectors
            neg_random_co_approaches = -random_co_approaches
            co_approaches = np.stack(
                (random_co_approaches, neg_random_co_approaches), axis=1
            )
            co_approaches = co_approaches.reshape(-1, 3)

            # Raycast in both approach directions to find intersecting contact points
            for i in range(len(surface_points)):
                loc, _, _ = self.mesh.ray.intersects_location(
                    ray_origins=[
                        contact_points[2 * i, :],
                        contact_points[2 * i + 1, :],
                    ],
                    ray_directions=[
                        co_approaches[2 * i, :],
                        co_approaches[2 * i + 1, :],
                    ],
                )
                if len(loc) > 0:
                    # Choose the farthest intersection to ensure wide grasp
                    distance = np.linalg.norm(loc - surface_points[i], axis=1)
                    farthest_index = np.argmax(distance)
                    contact_pairs_one.append(surface_points[i])
                    contact_pairs_two.append(loc[farthest_index])
                if len(contact_pairs_one) >= num:
                    break

        # Compute 6-DoF gripper poses from contact point pairs
        Hs = AntipodalGraspGenerator.define_gripper_pose(
            np.array(contact_pairs_one), np.array(contact_pairs_two)
        )
        Hs = self.denorm_grasp_pose(Hs)  # Convert to original mesh space
        return SE3Pose.from_mat(Hs)  # Return poses as SE3Pose instances

    @classmethod
    def define_gripper_pose(cls, contact_one, contact_two):
        # Compute 6-DoF gripper poses given two lists of contact points
        assert len(contact_one) == len(contact_two)  # Ensure pairwise alignment

        center = (contact_two + contact_one) / 2.0  # Gripper center (midpoint)
        antipodal_direction = contact_two - contact_one  # Gripper jaw direction (X-axis)
        antipodal_direction /= np.linalg.norm(
            antipodal_direction, axis=1, keepdims=True
        )  # Normalize

        approach_direction = np.random.rand(len(contact_one), 3)  # Random Z-axis candidates

        # Clean up cases where approach and X-axis are parallel (invalid frame)
        cross_products = np.cross(antipodal_direction, approach_direction)
        mask = np.where(np.all(np.isclose(cross_products, 0), axis=1))[0]
        while len(mask) > 0:
            approach_direction[mask] = np.random.rand(len(mask), 3)
            cross_products[mask] = np.cross(
                antipodal_direction[mask], approach_direction[mask]
            )
            mask = np.where(np.all(np.isclose(cross_products, 0), axis=1))[0]

        approach_direction = cross_products  # Final Z-axis (approach)
        approach_direction /= np.linalg.norm(approach_direction, axis=1, keepdims=True)

        co_direction = np.cross(approach_direction, antipodal_direction)  # Y-axis

        # Assemble SE(3) homogeneous transformation matrices
        Hs = np.zeros((len(contact_one), 4, 4))
        Hs[..., :3, 3] = center              # Translation (gripper center)
        Hs[..., 3, 3] = 1.0                  # Homogeneous coordinate
        Hs[..., :3, 0] = antipodal_direction  # X-axis (gripper jaw direction)
        Hs[..., :3, 1] = co_direction         # Y-axis
        Hs[..., :3, 2] = approach_direction   # Z-axis (approach direction)
        return Hs
