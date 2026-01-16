# planner_core.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple, Optional, List, Dict

import math
import numpy as np


# =======================
# Geometry info for the voxel grid
# =======================

@dataclass
class VoxelGridInfo:
    """
    Describes the geometry of a 3D voxel grid.
    It does NOT contain data, only resolution, origin and sizes.
    """
    resolution: float      # meters per voxel (x, y, z)
    origin_x: float        # world coordinate of voxel index (0, 0, 0)
    origin_y: float
    origin_z: float
    size_x: int            # number of voxels in x
    size_y: int            # number of voxels in y
    size_z: int            # number of voxels in z


class VoxelGrid:
    """
    Provides coordinate conversion between world coordinates and voxel indices.
    Data (ESDF / cost) live in separate numpy arrays with shape (Z, Y, X).
    """

    def __init__(self, info: VoxelGridInfo):
        self.info = info

    def world_to_voxel_index(
        self,
        x: float,
        y: float,
        z: float
    ) -> Optional[Tuple[int, int, int]]:
        """
        Convert world coordinates (meters) to voxel indices (ix, iy, iz).
        Returns None if outside the grid.
        """
        ix = int((x - self.info.origin_x) / self.info.resolution)
        iy = int((y - self.info.origin_y) / self.info.resolution)
        iz = int((z - self.info.origin_z) / self.info.resolution)

        if (0 <= ix < self.info.size_x and
            0 <= iy < self.info.size_y and
            0 <= iz < self.info.size_z):
            return ix, iy, iz
        return None

    def voxel_index_to_world_center(
        self,
        ix: int,
        iy: int,
        iz: int
    ) -> Tuple[float, float, float]:
        """
        Convert voxel indices (ix, iy, iz) to the world coordinates
        of the voxel center.
        """
        x = self.info.origin_x + (ix + 0.5) * self.info.resolution
        y = self.info.origin_y + (iy + 0.5) * self.info.resolution
        z = self.info.origin_z + (iz + 0.5) * self.info.resolution
        return x, y, z


# =======================
# ESDF -> cost grid
# =======================

def esdf_to_cost_grid(
    esdf_grid: np.ndarray,
    safety_radius: float,
    inflation_radius: float,
    free_cost: float = 1.0,
    max_cost: float = 5.0,
    unknown_threshold: float = -999.0,
    unknown_is_free: bool = True,
    unknown_cost: float = 2.0,
) -> np.ndarray:
    """
    ESDF (Z,Y,X) -> cost grid (Z,Y,X)

    unknown_threshold:
      Values <= unknown_threshold are treated as "unknown/unobserved" (e.g. -1000 from nvblox)

    unknown_is_free:
      True  -> unknown is traversable (cost=unknown_cost)
      False -> unknown is blocked (cost=np.inf)
    """
    if inflation_radius <= safety_radius:
        raise ValueError("inflation_radius must be > safety_radius")

    dist = esdf_grid.astype(np.float32)
    cost_grid = np.full_like(dist, free_cost, dtype=np.float32)

    # --- 0) unknown mask ---
    unknown_mask = dist <= unknown_threshold
    if unknown_is_free:
        cost_grid[unknown_mask] = float(unknown_cost)
    else:
        cost_grid[unknown_mask] = np.inf

    # --- 1) Forbidden region (ONLY where distance is known) ---
    known_mask = ~unknown_mask
    forbidden_mask = (dist < safety_radius) & known_mask
    cost_grid[forbidden_mask] = np.inf

    # --- 2) Inflation corridor (ONLY where distance is known) ---
    corridor_mask = (dist >= safety_radius) & (dist < inflation_radius) & known_mask
    if np.any(corridor_mask):
        d_corr = dist[corridor_mask]
        ratio = (d_corr - safety_radius) / (inflation_radius - safety_radius)
        cost_grid[corridor_mask] = max_cost - (max_cost - free_cost) * ratio

    return cost_grid



# =======================
# 3D A* planner
# =======================

class AStar3DPlanner:
    """
    Simple 3D A* planner on a cost grid.

    - voxels with cost_grid[z, y, x] = np.inf are considered forbidden
    - other voxels have a step cost factor >= free_cost

    The planner searches for a path that minimizes the sum of
    (step_length * cell_cost) over all steps.
    """

    def __init__(self, grid: VoxelGrid, cost_grid: np.ndarray):
        assert cost_grid.ndim == 3, "cost_grid must be 3D (Z, Y, X)"

        info = grid.info
        assert cost_grid.shape == (info.size_z, info.size_y, info.size_x), \
            "cost_grid shape must match VoxelGridInfo"

        self.grid = grid
        self.cost_grid = cost_grid

        # 26-connected neighborhood (including diagonals)
        self.neighbor_offsets: List[Tuple[int, int, int]] = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

    @staticmethod
    def estimate_distance_to_goal(
        ix: int,
        iy: int,
        iz: int,
        gx: int,
        gy: int,
        gz: int
    ) -> float:
        """
        Heuristic for A*:
        straight-line distance in grid index space to the goal.
        """
        return math.sqrt((ix - gx) ** 2 + (iy - gy) ** 2 + (iz - gz) ** 2)

    def plan_path(
        self,
        start_xyz: Tuple[float, float, float],
        goal_xyz: Tuple[float, float, float],
    ) -> List[Tuple[float, float, float]]:
        """
        Run A* from start_xyz to goal_xyz.

        Args:
            start_xyz: start position in world coordinates (meters).
            goal_xyz: goal position in world coordinates (meters).

        Returns:
            List of world coordinates (x, y, z) along the path.
            Empty list if no path was found.
        """
        start_index = self.grid.world_to_voxel_index(*start_xyz)
        goal_index = self.grid.world_to_voxel_index(*goal_xyz)

        if start_index is None or goal_index is None:
            return []

        sx, sy, sz = start_index
        gx, gy, gz = goal_index

        # Priority queue (min-heap) of (f_cost, (ix, iy, iz))
        frontier: List[Tuple[float, Tuple[int, int, int]]] = []
        import heapq
        heapq.heappush(frontier, (0.0, (sx, sy, sz)))

        # cost_from_start[(ix, iy, iz)] = accumulated cost g
        cost_from_start: Dict[Tuple[int, int, int], float] = {(sx, sy, sz): 0.0}

        # best_predecessor[(ix, iy, iz)] = previous voxel on the best path so far
        best_predecessor: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}

        while frontier:
            f_cost, (cx, cy, cz) = heapq.heappop(frontier)

            # Goal reached -> reconstruct path
            if (cx, cy, cz) == (gx, gy, gz):
                return self._reconstruct_world_path(best_predecessor, (cx, cy, cz))

            for dx, dy, dz in self.neighbor_offsets:
                nx, ny, nz = cx + dx, cy + dy, cz + dz

                # Bounds check
                if not (0 <= nx < self.grid.info.size_x and
                        0 <= ny < self.grid.info.size_y and
                        0 <= nz < self.grid.info.size_z):
                    continue

                cell_cost = float(self.cost_grid[nz, ny, nx])
                if not math.isfinite(cell_cost):
                    # Forbidden cell (np.inf or NaN)
                    continue

                # Geometric step length (1 for axis-aligned, sqrt(2) or sqrt(3) for diagonals)
                step_length = math.sqrt(dx * dx + dy * dy + dz * dz)
                step_cost = step_length * cell_cost

                current_g = cost_from_start[(cx, cy, cz)]
                tentative_g = current_g + step_cost

                neighbor_key = (nx, ny, nz)

                if neighbor_key not in cost_from_start or tentative_g < cost_from_start[neighbor_key]:
                    # Found a better path to this neighbor
                    cost_from_start[neighbor_key] = tentative_g
                    best_predecessor[neighbor_key] = (cx, cy, cz)

                    h = self.estimate_distance_to_goal(nx, ny, nz, gx, gy, gz)
                    total_f = tentative_g + h

                    heapq.heappush(frontier, (total_f, neighbor_key))

        # No path found
        return []

    def _reconstruct_world_path(
        self,
        best_predecessor: Dict[Tuple[int, int, int], Tuple[int, int, int]],
        goal_index: Tuple[int, int, int],
    ) -> List[Tuple[float, float, float]]:
        """
        Backtrack from the goal voxel to the start voxel using
        best_predecessor, then convert all voxel indices to world
        coordinates (centers).
        """
        path_indices: List[Tuple[int, int, int]] = [goal_index]
        current = goal_index

        while current in best_predecessor:
            current = best_predecessor[current]
            path_indices.append(current)

        path_indices.reverse()

        path_world: List[Tuple[float, float, float]] = [
            self.grid.voxel_index_to_world_center(ix, iy, iz)
            for (ix, iy, iz) in path_indices
        ]
        return path_world
