from importlib import import_module


def spawn_scene(scene_name, world, vehicle, carla):
  if scene_name is None:
    return []
  if not scene_name.isidentifier():
    raise ValueError(f"invalid CARLA scene name: {scene_name}")

  module_name = f"{__name__}.{scene_name}"
  try:
    scene = import_module(module_name)
  except ModuleNotFoundError as exc:
    if exc.name != module_name:
      raise
    raise ValueError(f"unknown CARLA scene: {scene_name}") from exc
  if not hasattr(scene, "spawn"):
    raise ValueError(f"CARLA scene has no spawn function: {scene_name}")
  return scene.spawn(world, vehicle, carla)