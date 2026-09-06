import unittest

from physics_engine import Ball, PhysicsWorld, Vector2


class PhysicsWorldTests(unittest.TestCase):
    def test_gravity_accelerates_ball(self) -> None:
        world = PhysicsWorld(gravity=Vector2(0, 10))
        ball = world.add_ball(Ball(position=Vector2(0, 0)))

        world.step(0.5)

        self.assertAlmostEqual(ball.velocity.y, 5)
        self.assertAlmostEqual(ball.position.y, 2.5)

    def test_restitution_controls_floor_bounce(self) -> None:
        world = PhysicsWorld(
            gravity=Vector2(0, 0),
            bounds=(0, 0, 10, 10),
        )
        ball = world.add_ball(
            Ball(
                position=Vector2(5, 8),
                radius=1,
                velocity=Vector2(0, 4),
                restitution=0.25,
            )
        )

        world.step(1)

        self.assertAlmostEqual(ball.position.y, 9)
        self.assertAlmostEqual(ball.velocity.y, -1)

    def test_world_restitution_overrides_ball_restitution(self) -> None:
        world = PhysicsWorld(
            gravity=Vector2(0, 0),
            bounds=(0, 0, 10, 10),
            restitution=1,
        )
        ball = world.add_ball(
            Ball(
                position=Vector2(5, 8),
                radius=1,
                velocity=Vector2(0, 4),
                restitution=0,
            )
        )

        world.step(1)

        self.assertAlmostEqual(ball.velocity.y, -4)

    def test_horizontal_boundaries_are_resolved(self) -> None:
        world = PhysicsWorld(
            gravity=Vector2(),
            bounds=(0, 0, 10, 10),
        )
        ball = world.add_ball(
            Ball(
                position=Vector2(2, 5),
                radius=1,
                velocity=Vector2(-3, 0),
                restitution=1,
            )
        )

        world.step(1)

        self.assertAlmostEqual(ball.position.x, 1)
        self.assertAlmostEqual(ball.velocity.x, 3)

    def test_invalid_configuration_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            Ball(position=Vector2(), radius=0)
        with self.assertRaises(ValueError):
            PhysicsWorld(restitution=1.1)
        with self.assertRaises(ValueError):
            PhysicsWorld().step(0)


if __name__ == "__main__":
    unittest.main()
