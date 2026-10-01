import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class ShelfScene(Node):
    def __init__(self):
        super().__init__("shelf_scene")
        p = {
            "frame_id": "world",
            "x": 0.0,
            "y": 0.5,
            "yaw": 0.0,
            "width": 1.0,
            "depth": 0.45,
            "board_heights": [0.0, 0.23, 0.46, 0.69],
            "board_thickness": 0.03,
            "top_height": 0.92,
            "bottom_height": -0.8,
            "post_size": 0.04,
            "back_wall": False,
            "back_wall_thickness": 0.01,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in p}
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.busy = False
        self.warned = False
        # move_group can reset its scene while starting, so keep checking and re-add when missing
        self.timer = self.create_timer(2.0, self.check)

    def boxes(self):
        """Boxes as (size_x, size_y, size_z, cx, cy, cz) in the shelf-local frame."""
        p = self.p
        w, d, s = p["width"], p["depth"], p["post_size"]
        z0, z1 = p["bottom_height"], p["top_height"]
        out = []
        for z in list(p["board_heights"]) + [z1]:
            out.append((w, d, p["board_thickness"], 0.0, d / 2, z))
        hz = z1 - z0
        for px in (-w / 2 + s / 2, w / 2 - s / 2):
            for py in (s / 2, d - s / 2):
                out.append((s, s, hz, px, py, z0 + hz / 2))
        if p["back_wall"]:
            t = p["back_wall_thickness"]
            out.append((w, t, hz, 0.0, d - t / 2, z0 + hz / 2))
        return out

    def collision_object(self):
        p = self.p
        c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
        obj = CollisionObject()
        obj.header.frame_id = p["frame_id"]
        obj.id = "shelf"
        obj.operation = CollisionObject.ADD
        obj.pose.orientation.w = 1.0
        for sx, sy, sz, lx, ly, lz in self.boxes():
            prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, sz])
            pose = Pose()
            pose.position.x = p["x"] + c * lx - s * ly
            pose.position.y = p["y"] + s * lx + c * ly
            pose.position.z = lz
            pose.orientation.z = math.sin(p["yaw"] / 2)
            pose.orientation.w = math.cos(p["yaw"] / 2)
            obj.primitives.append(prim)
            obj.primitive_poses.append(pose)
        return obj

    def check(self):
        if self.busy:
            return
        if not (self.client.service_is_ready() and self.scene_client.service_is_ready()):
            if not self.warned:
                self.get_logger().info("waiting for move_group ...")
                self.warned = True
            return
        self.busy = True
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        self.scene_client.call_async(req).add_done_callback(self.on_scene)

    def on_scene(self, fut):
        res = fut.result()
        if res is not None and any(o.id == "shelf" for o in res.scene.world.collision_objects):
            self.busy = False
            return
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(self.collision_object())
        self.client.call_async(ApplyPlanningScene.Request(scene=scene)).add_done_callback(self.on_applied)

    def on_applied(self, fut):
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info("shelf added to planning scene" if ok else "failed to add shelf")
        self.busy = False


def main():
    rclpy.init()
    node = ShelfScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    rclpy.try_shutdown()
