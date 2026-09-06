# Basic physics engine

This repository contains a dependency-free 2D physics engine for basic ball
mechanics. It uses semi-implicit Euler integration, configurable gravity,
optional rectangular world bounds, and restitution (bounciness) from `0`
(no bounce) to `1` (perfectly elastic bounce).

```python
from physics_engine import Ball, PhysicsWorld, Vector2

world = PhysicsWorld(
    gravity=Vector2(0, 9.81),
    bounds=(0, 0, 800, 600),
)
ball = world.add_ball(
    Ball(
        position=Vector2(100, 100),
        radius=16,
        velocity=Vector2(80, 0),
        restitution=0.75,
    )
)

# Call once per frame; dt is elapsed time in seconds.
world.step(1 / 60)
print(ball.position, ball.velocity)
```

The `restitution` passed to `PhysicsWorld` is an optional global override. If
it is omitted, each ball uses its own restitution value.

Run the tests with:

```sh
python -m unittest discover -s tests
```
