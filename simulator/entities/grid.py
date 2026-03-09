"""Lightweight 3D spatial index for neighbor queries."""

import threading
from collections import defaultdict
from collections.abc import Iterable
from math import floor

from simulator.helpers.coordinates import ENU
from simulator.helpers.rid import RIDData

CellKey = tuple[int, int, int]
Cell = set[int]


class Grid:
    """Lightweight 3D spatial index for neighbor queries."""

    def __init__(self, cell_size: float) -> None:
        assert cell_size > 0
        self.cell_size = cell_size
        self._cell: dict[CellKey, Cell] = defaultdict(set)
        self._rid: dict[int, RIDData] = {}
        self._key: dict[int, CellKey] = {}
        self._lock = threading.RLock()  # single structure lock
        self._pending_rid: defaultdict[int, bool] = defaultdict(lambda: False)
        # track if RID has been retransmitted

    # === Core methods ===
    def _idx(self, coor: float) -> int:
        return int(floor(coor / self.cell_size))

    def _pos2key(self, pos: ENU) -> CellKey:
        return (self._idx(pos.x), self._idx(pos.y), self._idx(pos.z))

    def rid(self, sysid: int) -> RIDData:
        """Return RID object only if it exists and has pending data."""
        with self._lock:
            return self._rid[sysid]

    def pop_rid(self, sysid: int) -> RIDData | None:
        """Pop RID object if it has pending data, else return None."""
        with self._lock:
            if self._pending_rid.get(sysid):
                rid = self.rid(sysid)
                self._pending_rid[sysid] = False
                return rid
            return None

    # === Sysid management ===
    def add_rid(self, sysid: int, rid: RIDData) -> None:
        """
        Insert a new UAV at the given position.
        It assumes sysid is not already present.
        """
        pos = rid.enu_pos
        k = self._pos2key(pos)
        with self._lock:
            if sysid in self._rid:
                raise ValueError(f"sysid {sysid} already exists in grid")
            self._cell[k].add(sysid)
            self._key[sysid] = k
            self._rid[sysid] = rid
            self._pending_rid[sysid] = True

    def remove_sysid(self, sysid: int) -> None:
        """Completely remove a UAV from the grid."""
        with self._lock:
            k = self._key.pop(sysid, None)
            self._rid.pop(sysid, None)
            self._pending_rid.pop(sysid, None)
            if k is None:
                return
            cell = self._cell.get(k)
            if cell:
                cell.discard(sysid)
                if not cell:
                    self._cell.pop(k, None)

    def update(self, sysid: int, rid: RIDData) -> None:
        """Incrementally move sysid between cells if needed."""
        pos = rid.enu_pos
        k_new = self._pos2key(pos)
        with self._lock:
            k_old = self._key.get(sysid)
            if k_old != k_new:
                # Remove from old cell (if any), avoiding accidental creation
                if k_old is not None:
                    old_cell = self._cell.get(k_old)  # get avoids new cell
                    if old_cell:
                        old_cell.discard(sysid)
                        if not old_cell:
                            self._cell.pop(k_old, None)
                # Add to new cell
                self._cell[k_new].add(sysid)
                self._key[sysid] = k_new
            self._rid[sysid] = rid
            self._pending_rid[sysid] = True

    # === Neighbor queries ===
    def _iter_neighbor_keys(self, pos: ENU) -> Iterable[CellKey]:
        cx, cy, cz = self._pos2key(pos)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    key = (cx + dx, cy + dy, cz + dz)
                    if key in self._cell:
                        yield key

    def iter_neighbor_sysids_snapshot(self, pos: ENU) -> list[int]:
        """Return a snapshot of sysids in 3x3x3 neighborhood (safe, no locks held)."""
        with self._lock:
            cell: list[int] = []
            for key in self._iter_neighbor_keys(pos):
                cell.extend(self._cell[key])
            return cell

    def iter_neighbors_within(
        self, sysid: int, pos: ENU, radius: float | None = None
    ) -> Iterable[int]:
        """
        Yield neighbor sysids within optional Euclidean radius.
        Assumes radius = None or 0<radius<=cell_size for correctness.
        """
        r2 = None if radius is None else radius * radius
        neighbor_ids = self.iter_neighbor_sysids_snapshot(pos)
        for o_sysid in neighbor_ids:
            if o_sysid == sysid:
                continue
            with self._lock:
                o_rid = self._rid.get(o_sysid)
            if o_rid is None:
                continue
            if r2 is not None and ENU.distance_squared(pos, o_rid.enu_pos) > r2:
                continue
            yield o_sysid
