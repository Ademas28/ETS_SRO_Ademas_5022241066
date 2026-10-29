import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

TARGET_PATH = '/target'
BASE_PATH   = '/Quadcopter/base'
SENSOR_PATH = '/Quadcopter/fast3DLaserScanner/sensor'
MAX_RANGE   = 5.0
FLY_Z       = 1.5
AREA_X      = (-2.0, 2.0)
AREA_Y      = (-2.0, 2.0)
LANE_GAP    = 1.0
SPEED       = 0.4
REACH_TOL   = 0.15
VOXEL       = 0.05

# (xmin, xmax, ymin, ymax, tinggi_puncak)
OBSTACLES = [
    (-1.0, 0.0, -1.0, 0.0, 1.0),
    (1.0, 1.6, 0.0, 1.0, 2.0),
]
SAFE_MARGIN = 0.6
CLEARANCE   = 0.6
RAMP        = 1.0
MAX_LEAD    = 0.5


def lawnmower(xr, yr, z, gap):
    pts, flip = [], False
    for x in np.arange(xr[0], xr[1] + 1e-6, gap):
        ys = (yr[1], yr[0]) if flip else (yr[0], yr[1])
        pts += [[x, ys[0], z], [x, ys[1], z]]
        flip = not flip
    return np.array(pts)


def plan_path(wps, step=0.2):
    dense = [np.array(wps[0], dtype=float)]
    for a, b in zip(wps[:-1], wps[1:]):
        n = max(int(np.linalg.norm(b - a) / step), 1)
        for k in range(1, n + 1):
            dense.append(a + (b - a) * k / n)
    dense = np.array(dense)

    z_need = dense[:, 2].copy()
    for i, p in enumerate(dense):
        for (x0, x1, y0, y1, top) in OBSTACLES:
            if (x0 - SAFE_MARGIN <= p[0] <= x1 + SAFE_MARGIN and
                    y0 - SAFE_MARGIN <= p[1] <= y1 + SAFE_MARGIN):
                z_need[i] = max(z_need[i], top + CLEARANCE)

    k = int(np.ceil(RAMP / step))
    n = len(z_need)
    dense[:, 2] = [z_need[max(0, i - k):i + k + 1].max() for i in range(n)]
    return dense


def voxelize(p, size):
    if len(p) == 0:
        return p
    keys = np.floor(p / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return p[idx]


def read_cloud(sim, sensor):
    out = sim.readVisionSensor(sensor)
    rest = list(out[1:])

    packets = []
    if (len(rest) >= 2 and isinstance(rest[0], (list, tuple))
            and isinstance(rest[1], (list, tuple)) and len(rest[1]) > 0
            and all(float(c).is_integer() for c in rest[1])
            and int(sum(rest[1])) == len(rest[0])):
        i = 0
        for c in rest[1]:
            packets.append(rest[0][i:i + int(c)])
            i += int(c)
    else:
        packets = [r for r in rest if isinstance(r, (list, tuple))]

    for p in packets:
        if len(p) > 2:
            w, h = int(p[0]), int(p[1])
            if w > 0 and h > 0 and len(p) == 2 + 4 * w * h:
                arr = np.asarray(p[2:], dtype=float).reshape(-1, 4)
                valid = arr[:, 3] < MAX_RANGE * 0.9999
                return arr[valid, :3]
    return None


def get_T(sim, obj, rel_to):
    M = np.array(sim.getObjectMatrix(obj, rel_to)).reshape(3, 4)
    return np.vstack([M, [0, 0, 0, 1]])


def transform(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def sensor_to_world(sim, sensor, base, pts):
    T_base_sensor = get_T(sim, sensor, base)
    T_world_base = get_T(sim, base, sim.handle_world)
    return transform(T_world_base, transform(T_base_sensor, pts))


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    client.setStepping(True)

    target = sim.getObject(TARGET_PATH)
    base = sim.getObject(BASE_PATH)
    sensor = sim.getObject(SENSOR_PATH)

    waypoints = lawnmower(AREA_X, AREA_Y, FLY_Z, LANE_GAP)
    waypoints = np.vstack([[waypoints[0][0], waypoints[0][1], FLY_Z], waypoints])
    waypoints = plan_path(waypoints)

    cloud = np.empty((0, 3))
    buffer = []
    path = []

    sim.startSimulation()
    try:
        dt = sim.getSimulationTimeStep()
        wp_i, step = 0, 0
        tpos = np.array(sim.getObjectPosition(target, sim.handle_world))

        while wp_i < len(waypoints):
            goal = waypoints[wp_i]
            drone_now = np.array(sim.getObjectPosition(base, sim.handle_world))
            if np.linalg.norm(tpos - drone_now) < MAX_LEAD:
                vec = goal - tpos
                dist = np.linalg.norm(vec)
                tpos = goal.copy() if dist < SPEED * dt else tpos + vec / dist * SPEED * dt
                sim.setObjectPosition(target, sim.handle_world, tpos.tolist())

            client.step()
            step += 1

            dpos = np.array(sim.getObjectPosition(base, sim.handle_world))
            path.append(dpos)

            if step > 3:
                pts = read_cloud(sim, sensor)
                if pts is not None and len(pts) > 0:
                    buffer.append(sensor_to_world(sim, sensor, base, pts))

            if step % 100 == 0 and buffer:
                cloud = voxelize(np.vstack([cloud] + buffer), VOXEL)
                buffer = []
                print(f'step {step}  wp {wp_i + 1}/{len(waypoints)}  titik: {len(cloud)}')

            if np.linalg.norm(dpos - goal) < REACH_TOL:
                wp_i += 1

        if buffer:
            cloud = voxelize(np.vstack([cloud] + buffer), VOXEL)
    finally:
        sim.stopSimulation()

    path = np.array(path)
    np.savez('map_result.npz', cloud=cloud, path=path)
    print('Total titik peta:', len(cloud))
    plot_map(cloud, path)


def plot_map(cloud, path):
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    if len(cloud):
        sc = ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2],
                        c=cloud[:, 2], cmap='RdYlGn_r', s=2)
        fig.colorbar(sc, ax=ax, shrink=0.6, label='Ketinggian Z (m)')
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', linewidth=2, label='Path drone')

    allp = np.vstack([cloud, path]) if len(cloud) else path
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(hi - lo)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Map')
    ax.legend()

    ax.mouse_init(rotate_btn=1, pan_btn=[], zoom_btn=[])
    plt.show()


if __name__ == '__main__':
    main()