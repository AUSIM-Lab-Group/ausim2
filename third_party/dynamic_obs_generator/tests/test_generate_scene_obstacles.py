from __future__ import annotations

import sys
import unittest
import xml.etree.ElementTree as xml_et
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import generate_scene_obstacles as gso


class GenerateSceneObstaclesTest(unittest.TestCase):
    def test_inject_obstacles_wraps_collidable_geom_in_mocap_body(self) -> None:
        root = xml_et.Element("mujoco")
        worldbody = xml_et.SubElement(root, "worldbody")
        obstacle = gso.GeneratedObstacle(
            name="dynamic_obs_0",
            geom_type="box",
            size=[0.25, 0.25, 0.5],
            position=[1.0, 2.0, 0.5],
            half_extents=(0.25, 0.25, 0.5),
        )

        gso.inject_obstacles(worldbody, [obstacle], collision_enabled=True)

        self.assertEqual(1, len(worldbody))
        body = worldbody[0]
        self.assertEqual("body", body.tag)
        self.assertEqual("dynamic_obs_0", body.attrib.get("name"))
        self.assertEqual("true", body.attrib.get("mocap"))
        self.assertEqual("1 2 0.5", body.attrib.get("pos"))

        self.assertEqual(1, len(body))
        geom = body[0]
        self.assertEqual("geom", geom.tag)
        self.assertEqual("dynamic_obs_0_geom", geom.attrib.get("name"))
        self.assertEqual("box", geom.attrib.get("type"))
        self.assertEqual("0.25 0.25 0.5", geom.attrib.get("size"))
        self.assertEqual("1", geom.attrib.get("contype"))
        self.assertEqual("1", geom.attrib.get("conaffinity"))
        self.assertNotIn("pos", geom.attrib)

    def test_remove_dynamic_obstacles_removes_generated_mocap_bodies(self) -> None:
        root = xml_et.Element("mujoco")
        worldbody = xml_et.SubElement(root, "worldbody")
        body = xml_et.SubElement(worldbody, "body", name="dynamic_obs_0", mocap="true", pos="0 0 0")
        xml_et.SubElement(body, "geom", name="dynamic_obs_0_geom", type="box", size="0.1 0.1 0.1")
        xml_et.SubElement(worldbody, "geom", name="ground", type="plane", size="1 1 0.1")

        removed = gso.remove_dynamic_obstacles(root)

        self.assertEqual(1, removed)
        self.assertEqual(1, len(worldbody))
        self.assertEqual("ground", worldbody[0].attrib.get("name"))


if __name__ == "__main__":
    unittest.main()
