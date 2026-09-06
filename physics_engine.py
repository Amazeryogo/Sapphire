"""A small 2D physics engine for spherical bodies.

The engine intentionally models only the mechanics needed for a simple ball:
constant gravity, Euler integration, and perfectly positional boundary
collisions with configurable restitution.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass
class Vector2:
    """A two-dimensional vector."""

    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: Vector2) -> Vector2:
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2) -> Vector2:
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vector2:
        return Vector2(self.x * scalar, self.y * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> Vector2:
        if scalar == 0:
            raise ZeroDivisionError("cannot divide a vector by zero")
        return Vector2(self.x / scalar, self.y / scalar)


@dataclass
class Ball:
    """A circular rigid body with a configurable bounce response."""

    position: Vector2
    radius: float = 0.5
    mass: float = 1.0
    velocity: Vector2 | None = None
    restitution: float = 0.8

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("radius must be greater than zero")
        if self.mass <= 0:
            raise ValueError("mass must be greater than zero")
        if not 0 <= self.restitution <= 1:
            raise ValueError("restitution must be between 0 and 1")
        if self.velocity is None:
            self.velocity = Vector2()


class PhysicsWorld:
    """Advances balls through time under gravity and optional world bounds.

    Coordinates use the conventional screen-space layout: positive x points
    right and positive y points down. Bounds are expressed as (left, top,
    right, bottom), and represent the inside edge of the world.
    """

    def __init__(
        self,
        gravity: Vector2 | None = None,
        bounds: tuple[float, float, float, float] | None = None,
        restitution: float | None = None,
    ) -> None:
        if restitution is not None and not 0 <= restitution <= 1:
            raise ValueError("restitution must be between 0 and 1")
        if bounds is not None:
            left, top, right, bottom = bounds
            if left >= right or top >= bottom:
                raise ValueError("bounds must have positive width and height")

        self.gravity = gravity if gravity is not None else Vector2(0.0, 9.81)
        self.bounds = bounds
        self.restitution = restitution
        self.balls: list[Ball] = []

    def add_ball(self, ball: Ball) -> Ball:
        """Add a ball to the world and return it for convenient setup."""
        self.balls.append(ball)
        return ball

    def add_balls(self, balls: Iterable[Ball]) -> None:
        for ball in balls:
            self.add_ball(ball)

    def step(self, dt: float) -> None:
        """Advance every ball by ``dt`` seconds.

        A non-positive or non-finite timestep is rejected so simulation bugs
        cannot silently produce invalid positions.
        """
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be a finite value greater than zero")

        for ball in self.balls:
            # Semi-implicit Euler integration is stable for this simple model:
            # update velocity first, then use it to update position.
            ball.velocity = ball.velocity + self.gravity * dt
            ball.position = ball.position + ball.velocity * dt
            self._resolve_bounds(ball)

    def _resolve_bounds(self, ball: Ball) -> None:
        if self.bounds is None:
            return

        left, top, right, bottom = self.bounds
        coefficient = (
            self.restitution
            if self.restitution is not None
            else ball.restitution
        )

        min_x, max_x = left + ball.radius, right - ball.radius
        min_y, max_y = top + ball.radius, bottom - ball.radius

        if ball.position.x < min_x:
            ball.position.x = min_x
            if ball.velocity.x < 0:
                ball.velocity.x = -ball.velocity.x * coefficient
        elif ball.position.x > max_x:
            ball.position.x = max_x
            if ball.velocity.x > 0:
                ball.velocity.x = -ball.velocity.x * coefficient

        if ball.position.y < min_y:
            ball.position.y = min_y
            if ball.velocity.y < 0:
                ball.velocity.y = -ball.velocity.y * coefficient
        elif ball.position.y > max_y:
            ball.position.y = max_y
            if ball.velocity.y > 0:
                ball.velocity.y = -ball.velocity.y * coefficient
