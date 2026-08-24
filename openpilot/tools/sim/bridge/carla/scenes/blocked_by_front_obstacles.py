"""Short CARLA scene with a barrier row directly ahead of the ego vehicle."""

OBSTACLE_DISTANCE = 12.0
OBSTACLE_OFFSETS = tuple(-1.8 + 0.45 * index for index in range(9))


def spawn(world, vehicle, carla):
  """Spawn a static barrier row across the ego lane."""
  transform = vehicle.get_transform()
  forward = transform.get_forward_vector()
  right = transform.get_right_vector()
  origin = transform.location
  blueprint = world.get_blueprint_library().find("static.prop.streetbarrier")
  actors = []

  try:
    for lateral_offset in OBSTACLE_OFFSETS:
      location = carla.Location(
        x=origin.x + forward.x * OBSTACLE_DISTANCE + right.x * lateral_offset,
        y=origin.y + forward.y * OBSTACLE_DISTANCE + right.y * lateral_offset,
        z=origin.z + 0.05,
      )
      obstacle_transform = carla.Transform(
        location,
        carla.Rotation(yaw=transform.rotation.yaw),
      )
      obstacle = world.try_spawn_actor(blueprint, obstacle_transform)
      if obstacle is None:
        raise RuntimeError("could not spawn the blocked_by_front_obstacles scene")
      actors.append(obstacle)
  except Exception:
    for actor in reversed(actors):
      if actor.is_alive:
        actor.destroy()
    raise

  return actors