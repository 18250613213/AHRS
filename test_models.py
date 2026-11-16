import unittest
import numpy as np
import trimesh
from main import ModelLoader


class ModelTests(unittest.TestCase):
    def test_small_material_and_scene_transform_are_preserved(self):
        scene = trimesh.Scene()
        scene.add_geometry(trimesh.creation.box(), transform=trimesh.transformations.translation_matrix([10, 0, 0]))
        scene.add_geometry(trimesh.creation.box(), transform=trimesh.transformations.translation_matrix([-10, 0, 0]))
        result = ModelLoader._process_scene(scene, {}, 'test')
        self.assertEqual(sum(len(s['faces']) for s in result['submeshes']), 24)
        centers = [s['vertices'].mean(axis=0)[0] for s in result['submeshes']]
        self.assertGreater(abs(centers[0] - centers[1]), 5)

    def test_detached_fragment_removed_but_attached_wing_retained(self):
        body = trimesh.creation.box(extents=[4, 1, 1])
        wing = trimesh.creation.box(extents=[1, 3, .1])
        debris = trimesh.creation.box(extents=[.2, .2, .2])
        debris.apply_translation([10, 10, 10])
        result = ModelLoader._process_scene(trimesh.Scene([body, wing, debris]),
                                           {'remove_detached': True}, 'test')
        self.assertEqual(sum(len(s['faces']) for s in result['submeshes']), 24)


if __name__ == '__main__':
    unittest.main()
