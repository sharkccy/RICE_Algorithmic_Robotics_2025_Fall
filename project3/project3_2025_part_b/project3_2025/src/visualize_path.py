"""visualize_path.py

Reads obstacles.csv, path_point.csv and path_box.csv (if present) and plots them
using matplotlib. Saves a PNG `planner_result.png` and shows the figure.
"""
import csv
import os
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon


def read_obstacles(file_name='obstacles.csv'):
    obs = []
    if not os.path.exists(file_name):
        return obs
    with open(file_name, newline='') as f:
        rdr = csv.reader(f)
        for row in rdr:
            if not row:
                continue
            x, y, w, h = map(float, row[:4])
            obs.append((x, y, w, h))
    return obs


def read_point(file_name='path_point.csv'):
    pts = []
    if not os.path.exists(file_name):
        return pts
    with open(file_name, newline='') as f:
        rdr = csv.reader(f)
        for row in rdr:
            if not row:
                continue
            x, y = map(float, row[:2])
            pts.append((x, y))
    return pts


def read_box(file_name='path_box.csv'):
    pts = []
    if not os.path.exists(file_name):
        return pts
    with open(file_name, newline='') as f:
        rdr = csv.reader(f)
        for row in rdr:
            if not row:
                continue
            x, y, yaw = map(float, row[:3])
            pts.append((x, y, yaw))
    return pts


def draw(obstacles, point_path=None, box_path=None, side_len=5.0, save_as='planner_result.png'):
    fig, ax = plt.subplots(figsize=(6,6))

    # draw obstacles
    for (x,y,w,h) in obstacles:
        rect = Rectangle((x,y), w, h, facecolor='gray', edgecolor='black', alpha=0.8)
        ax.add_patch(rect)

    # point path
    if point_path and len(point_path) > 0:
        xs = [p[0] for p in point_path]
        ys = [p[1] for p in point_path]
        ax.plot(xs, ys, '-o', color='blue', label='point path')
        # draw directional arrows between consecutive points
        if len(point_path) >= 2:
            ux = []
            uy = []
            bx = []
            by = []
            for i in range(len(point_path)-1):
                x0,y0 = point_path[i]
                x1,y1 = point_path[i+1]
                dx = x1 - x0
                dy = y1 - y0
                mag = math.hypot(dx, dy)
                if mag == 0:
                    continue
                # place arrow at 40% along the segment and make arrow length 40% of segment
                bx.append(x0 + 0.4 * dx)
                by.append(y0 + 0.4 * dy)
                ux.append(0.4 * dx)
                uy.append(0.4 * dy)
            if bx:
                # scale=1 and scale_units='xy' means U,V are in data units
                ax.quiver(bx, by, ux, uy, angles='xy', scale_units='xy', scale=1.0,
                          color='blue', width=0.01, headwidth=3, zorder=5)

    # box path
    if box_path and len(box_path) > 0:
        xs = [p[0] for p in box_path]
        ys = [p[1] for p in box_path]
        ax.plot(xs, ys, '-o', color='red', label='box path')
        half = side_len / 2.0
        for (x,y,yaw) in box_path:
            corners = [(-half,-half),(half,-half),(half,half),(-half,half)]
            R = [[math.cos(yaw), -math.sin(yaw)],[math.sin(yaw), math.cos(yaw)]]
            pts = [ (x + R[0][0]*cx + R[0][1]*cy, y + R[1][0]*cx + R[1][1]*cy) for (cx,cy) in corners ]
            poly = Polygon(pts, closed=True, fill=False, edgecolor='red', linewidth=1)
            ax.add_patch(poly)
        # draw orientation arrows based on yaw (small arrows centered at each box)
        ox = []
        oy = []
        ou = []
        ov = []
        # choose arrow length as a fraction of the box side length so it's visible
        orient_len = 0.6 * side_len
        for (x,y,yaw) in box_path:
            ox.append(x)
            oy.append(y)
            ou.append(orient_len * math.cos(yaw))
            ov.append(orient_len * math.sin(yaw))
        if ox:
            ax.quiver(ox, oy, ou, ov, angles='xy', scale_units='xy', scale=1.0,
                      color='red', width=0.01, headwidth=3, zorder=6)

    # draw start/goal (assumes default start/goal used in Project3Exercise2)
    # ax.scatter([10],[10], c='green', s=80, label='start')
    # ax.scatter([90],[90], c='purple', s=80, label='goal')

    # ax.scatter([90],[10], c='green', s=80, label='start')
    # ax.scatter([10],[80], c='purple', s=80, label='goal')

    ax.scatter([50],[10], c='green', s=80, label='start')
    ax.scatter([10],[90], c='purple', s=80, label='goal')

    ax.set_xlim(0,100)
    ax.set_ylim(0,100)
    ax.set_aspect('equal', 'box')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.tight_layout()
    plt.savefig(save_as, dpi=200)
    plt.show()


if __name__ == '__main__':
    obs = read_obstacles()
    pp = read_point()
    pb = read_box()
    print(f'Loaded obstacles={len(obs)}, point_path={len(pp)}, box_path={len(pb)}')
    draw(obs, pp if pp else None, pb if pb else None)
