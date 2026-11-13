import sys, time, queue, math
import carla
xodr = open(sys.argv[1], encoding='utf-8').read()
out = sys.argv[2]
c = carla.Client('127.0.0.1', 2100); c.set_timeout(180.0)
print('server', c.get_server_version(), 'client', c.get_client_version())
t = time.time()
p = carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=0.6,
                                        smooth_junctions=True, enable_mesh_visibility=True)
w = c.generate_opendrive_world(xodr, p)
print('generate seconds', round(time.time() - t, 1), 'map', w.get_map().name)
m = w.get_map()
sps = m.get_spawn_points()
topo = m.get_topology()
print('spawn points', len(sps), 'topology edges', len(topo))
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; w.apply_settings(s)
w.set_weather(carla.WeatherParameters.ClearNoon)
bl = w.get_blueprint_library()
# longest straight-ish stretch: pick the spawn point whose waypoint can go furthest without a junction
def run(wp):
    d = 0
    while d < 400:
        nxt = wp.next(5.0)
        if not nxt or nxt[0].is_junction: break
        wp = nxt[0]; d += 5
    return d
best = max(sps, key=lambda sp: run(m.get_waypoint(sp.location)))
wp = m.get_waypoint(best.location)
print('best spawn', best.location, 'free run m', run(wp), 'lane width', wp.lane_width, 'lane id', wp.lane_id,
      'left mark', wp.left_lane_marking.type, wp.left_lane_marking.color, 'right mark', wp.right_lane_marking.type)
def shot(name, tf, fov, wd=1928, ht=1208):
    bp = bl.find('sensor.camera.rgb'); bp.set_attribute('image_size_x', str(wd)); bp.set_attribute('image_size_y', str(ht)); bp.set_attribute('fov', str(fov))
    q = queue.Queue(); cam = w.spawn_actor(bp, tf); cam.listen(q.put)
    img = None
    for _ in range(12):
        w.tick()
        try:
            img = q.get(timeout=5.0)
        except queue.Empty:
            pass
    img.save_to_disk(f'{out}/{name}.png'); cam.stop(); cam.destroy(); print('saved', name)
tf = wp.transform
fwd = tf.get_forward_vector()
shot('driver_fov40', carla.Transform(tf.location + carla.Location(z=1.13), tf.rotation), 40)
shot('chase', carla.Transform(tf.location + carla.Location(x=-8 * fwd.x, y=-8 * fwd.y, z=4), carla.Rotation(pitch=-12, yaw=tf.rotation.yaw)), 90, 1280, 720)
shot('top_300m', carla.Transform(tf.location + carla.Location(z=300), carla.Rotation(pitch=-90)), 90, 1280, 1280)
# a junction view
jwp = wp
for _ in range(200):
    n = jwp.next(5.0)
    if not n: break
    jwp = n[0]
    if jwp.is_junction: break
jt = jwp.transform
shot('junction_top_60m', carla.Transform(jt.location + carla.Location(z=60), carla.Rotation(pitch=-90)), 90, 1280, 1280)
veh = w.try_spawn_actor(bl.find('vehicle.tesla.model3'), carla.Transform(tf.location + carla.Location(z=0.5), tf.rotation))
for _ in range(40): w.tick()
print('vehicle spawned', veh is not None, 'z after settle', round(veh.get_location().z, 2) if veh else None)
if veh: veh.destroy()
s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
