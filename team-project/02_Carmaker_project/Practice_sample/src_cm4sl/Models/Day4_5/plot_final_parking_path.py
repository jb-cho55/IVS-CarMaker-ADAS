from __future__ import annotations

import heapq
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


N = 200
RES = 0.5
X_MIN = 0.0
Y_MAX = 0.0

EGO_W = 1.9
SAFETY_MARGIN = 0.8
BOUNDARY_INFLATE = EGO_W * 0.5 + SAFETY_MARGIN

TRAFFIC_W = 1.97
TRAFFIC_L = 4.47
OBSTACLE_MARGIN = EGO_W * 0.5 + SAFETY_MARGIN

START = (5.5, -36.5)
GOAL = (35.0, -30.0)
GOAL_YAW = 2.0 * math.pi / 3.0

MAP_BOUNDARY = np.array(
    [
        [4.0, -4.0],
        [4.0, -46.8],
        [48.0, -4.0],
        [48.0, -46.8],
    ],
    dtype=float,
)

# Current Day4_5 add_obstacle_ readers after expanding /Subsystem to T00..T20.
OBSTACLES = [
    (7.3, -28.7, -math.pi / 2.0, "T00"),
    (12.8, -6.8, -math.pi / 2.0, "T01"),
    (21.3, -6.6, -math.pi / 2.0, "T02"),
    (30.0, -6.5, -math.pi / 2.0, "T03"),
    (41.7, -6.3, -math.pi / 2.0, "T04"),
    (6.9, -6.9, -math.pi / 2.0, "T05"),
    (7.0, -21.8, math.pi / 2.0, "T06"),
    (18.6, -21.8, math.pi / 2.0, "T07"),
    (24.3, -21.9, math.pi / 2.0, "T08"),
    (36.0, -21.9, math.pi / 2.0, "T09"),
    (38.8, -21.8, math.pi / 2.0, "T10"),
    (41.8, -21.8, math.pi / 2.0, "T11"),
    (12.6, -28.8, -math.pi / 2.0, "T12"),
    (24.3, -28.9, -math.pi / 2.0, "T13"),
    (41.6, -28.9, -math.pi / 2.0, "T14"),
    (9.9, -44.4, math.pi / 2.0, "T15"),
    (21.4, -44.3, math.pi / 2.0, "T16"),
    (24.5, -44.4, math.pi / 2.0, "T17"),
    (36.0, -44.4, math.pi / 2.0, "T18"),
    (41.8, -44.5, math.pi / 2.0, "T19"),
    (44.6, -44.4, math.pi / 2.0, "T20"),
]


def sort_polygon(points: np.ndarray) -> np.ndarray:
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)]


def point_in_polygon(x: float, y: float, poly: np.ndarray) -> bool:
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_at = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            if x < x_at:
                inside = not inside
        j = i
    return inside


def point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    v2 = vx * vx + vy * vy
    if v2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / v2))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def dist_to_polygon_edge(x: float, y: float, poly: np.ndarray) -> float:
    best = 1e18
    j = len(poly) - 1
    for i in range(len(poly)):
        ax, ay = poly[j]
        bx, by = poly[i]
        best = min(best, point_to_segment(x, y, ax, ay, bx, by))
        j = i
    return best


def world_to_cell(x: float, y: float) -> tuple[int, int]:
    col = int(math.floor((x - X_MIN) / RES))
    row = int(math.floor((Y_MAX - y) / RES))
    return row, col


def cell_to_world(row: int, col: int) -> tuple[float, float]:
    x = X_MIN + (col + 0.5) * RES
    y = Y_MAX - (row + 0.5) * RES
    return x, y


def make_map() -> np.ndarray:
    poly = sort_polygon(MAP_BOUNDARY)
    occ = np.zeros((N, N), dtype=np.uint8)
    for row in range(N):
        for col in range(N):
            x, y = cell_to_world(row, col)
            if (not point_in_polygon(x, y, poly)) or dist_to_polygon_edge(x, y, poly) < BOUNDARY_INFLATE:
                occ[row, col] = 1
    return occ


def mark_obstacle(occ: np.ndarray, rear_x: float, rear_y: float, yaw: float) -> None:
    c = math.cos(yaw)
    s = math.sin(yaw)
    half_w = TRAFFIC_W * 0.5 + OBSTACLE_MARGIN
    local_xs = [-OBSTACLE_MARGIN, -OBSTACLE_MARGIN, TRAFFIC_L + OBSTACLE_MARGIN, TRAFFIC_L + OBSTACLE_MARGIN]
    local_ys = [-half_w, half_w, half_w, -half_w]

    corners = []
    for lx, ly in zip(local_xs, local_ys):
        corners.append((rear_x + lx * c - ly * s, rear_y + lx * s + ly * c))

    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    c0 = max(0, int(math.floor((min(xs) - X_MIN) / RES)) - 1)
    c1 = min(N - 1, int(math.floor((max(xs) - X_MIN) / RES)) + 1)
    r0 = max(0, int(math.floor((Y_MAX - max(ys)) / RES)) - 1)
    r1 = min(N - 1, int(math.floor((Y_MAX - min(ys)) / RES)) + 1)

    for row in range(r0, r1 + 1):
        for col in range(c0, c1 + 1):
            x, y = cell_to_world(row, col)
            dx = x - rear_x
            dy = y - rear_y
            local_x = dx * c + dy * s
            local_y = -dx * s + dy * c
            if -OBSTACLE_MARGIN <= local_x <= TRAFFIC_L + OBSTACLE_MARGIN and -half_w <= local_y <= half_w:
                occ[row, col] = 1


def nearest_free(occ: np.ndarray, rc: tuple[int, int]) -> tuple[int, int]:
    r, c = rc
    if 0 <= r < N and 0 <= c < N and occ[r, c] == 0:
        return rc
    for radius in range(1, 40):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if abs(dr) != radius and abs(dc) != radius:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < N and 0 <= nc < N and occ[nr, nc] == 0:
                    return nr, nc
    raise RuntimeError(f"No free cell near {rc}")


def astar(occ: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    moves = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    def h(a: tuple[int, int]) -> float:
        return math.hypot(a[0] - goal[0], a[1] - goal[1])

    open_heap: list[tuple[float, tuple[int, int]]] = [(h(start), start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]

        cr, cc = cur
        for dr, dc, cost in moves:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < N and 0 <= nc < N):
                continue
            if occ[nr, nc] != 0:
                continue
            nxt = (nr, nc)
            tentative = g_score[cur] + cost
            if tentative < g_score.get(nxt, 1e18):
                came_from[nxt] = cur
                g_score[nxt] = tentative
                heapq.heappush(open_heap, (tentative + h(nxt), nxt))

    return []


def px(x: float, y: float, scale: int, x0: float, y0: float) -> tuple[int, int]:
    return int(round((x - x0) * scale)), int(round((y0 - y) * scale))


def draw_vehicle(draw: ImageDraw.ImageDraw, rear_x: float, rear_y: float, yaw: float, label: str, scale: int, x0: float, y0: float) -> None:
    c = math.cos(yaw)
    s = math.sin(yaw)
    half_w = TRAFFIC_W * 0.5
    corners = []
    for lx, ly in [(0, -half_w), (0, half_w), (TRAFFIC_L, half_w), (TRAFFIC_L, -half_w)]:
        corners.append(px(rear_x + lx * c - ly * s, rear_y + lx * s + ly * c, scale, x0, y0))
    draw.polygon(corners, fill=(80, 145, 210), outline=(28, 82, 128))
    tx, ty = px(rear_x, rear_y, scale, x0, y0)
    draw.text((tx + 2, ty + 2), label, fill=(20, 55, 90))


def render(occ: np.ndarray, path: list[tuple[int, int]]) -> Path:
    x0, x1 = 0.0, 52.0
    y0, y1 = 0.0, -50.0
    scale = 18
    margin = 36
    width = int((x1 - x0) * scale) + margin * 2
    height = int((y0 - y1) * scale) + margin * 2
    img = Image.new("RGB", (width, height), (244, 246, 241))
    draw = ImageDraw.Draw(img)

    def p(x: float, y: float) -> tuple[int, int]:
        a, b = px(x, y, scale, x0, y0)
        return a + margin, b + margin

    # Occupancy cells.
    for row in range(N):
        for col in range(N):
            x, y = cell_to_world(row, col)
            if not (x0 <= x <= x1 and y1 <= y <= y0):
                continue
            if occ[row, col]:
                x_a, y_a = p(x - RES / 2, y + RES / 2)
                x_b, y_b = p(x + RES / 2, y - RES / 2)
                draw.rectangle([x_a, y_a, x_b, y_b], fill=(198, 203, 197))

    # Boundary.
    poly = sort_polygon(MAP_BOUNDARY)
    draw.line([p(x, y) for x, y in poly] + [p(poly[0, 0], poly[0, 1])], fill=(70, 70, 70), width=3)

    # Uninflated vehicles.
    for ox, oy, oyaw, label in OBSTACLES:
        draw_vehicle(draw, ox, oy, oyaw, label, scale, x0 - margin / scale, y0 + margin / scale)

    # Path.
    if path:
        world_path = [cell_to_world(r, c) for r, c in path]
        draw.line([p(x, y) for x, y in world_path], fill=(230, 70, 45), width=5)

    sx, sy = START
    gx, gy = GOAL
    draw.ellipse([p(sx - 0.45, sy + 0.45), p(sx + 0.45, sy - 0.45)], fill=(34, 150, 90), outline=(15, 90, 50), width=2)
    draw.text(p(sx + 0.6, sy), "START", fill=(15, 90, 50))
    draw.ellipse([p(gx - 0.45, gy + 0.45), p(gx + 0.45, gy - 0.45)], fill=(235, 185, 40), outline=(130, 95, 10), width=2)
    draw.text(p(gx + 0.6, gy), "GOAL", fill=(120, 85, 10))

    title = "Day4_5 final parking path preview (grid A*)"
    draw.text((margin, 10), title, fill=(20, 20, 20))
    draw.text((margin, height - 26), f"path cells: {len(path)} | start {START} -> goal {GOAL} | goal yaw {GOAL_YAW:.2f} rad", fill=(20, 20, 20))

    out = Path(__file__).with_name("final_parking_path_preview.png")
    img.save(out)
    return out


def main() -> None:
    occ = make_map()
    for ox, oy, oyaw, _ in OBSTACLES:
        mark_obstacle(occ, ox, oy, oyaw)

    start_cell = nearest_free(occ, world_to_cell(*START))
    goal_cell = nearest_free(occ, world_to_cell(*GOAL))
    path = astar(occ, start_cell, goal_cell)
    if not path:
        raise RuntimeError("No grid path found. Start/goal or obstacle inflation may be blocking the lot.")

    out = render(occ, path)
    print(f"start cell: {start_cell}, goal cell: {goal_cell}")
    print(f"path cells: {len(path)}")
    print(out)


if __name__ == "__main__":
    main()
