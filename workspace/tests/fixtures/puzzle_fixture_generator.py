"""原创、程序生成的拼图 fixture；不含外部图片或真值进入生产接口。"""

from __future__ import annotations

from dataclasses import dataclass
import random


@dataclass(frozen=True)
class FixturePiece:
    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    yaw_deg: float = 0.0
    timestamp_ns: int = 1
    equivalent_yaws_deg: tuple[float, ...] = ()


@dataclass(frozen=True)
class PuzzleFixture:
    family: str
    seed: int
    target_size_mm: tuple[float, float]
    pieces: tuple[FixturePiece, ...]
    ground_truth: tuple[tuple[str, tuple[tuple[float, float], ...]], ...]


def _area(polygon: tuple[tuple[float, float], ...]) -> float:
    return (
        abs(
            sum(
                point[0] * polygon[(index + 1) % len(polygon)][1]
                - point[1] * polygon[(index + 1) % len(polygon)][0]
                for index, point in enumerate(polygon)
            )
        )
        / 2
    )


def _edge_lengths(polygon: tuple[tuple[float, float], ...]) -> tuple[float, ...]:
    return tuple(
        (
            (polygon[(index + 1) % len(polygon)][0] - point[0]) ** 2
            + (polygon[(index + 1) % len(polygon)][1] - point[1]) ** 2
        )
        ** 0.5
        for index, point in enumerate(polygon)
    )


def _piece(identifier: str, polygon: tuple[tuple[float, float], ...]) -> FixturePiece:
    center_x = sum(point[0] for point in polygon) / len(polygon)
    center_y = sum(point[1] for point in polygon) / len(polygon)
    return FixturePiece(identifier, polygon, (center_x + 15.0, center_y + 180.0))


def _validate(
    family: str,
    polygons: tuple[tuple[tuple[float, float], ...], ...],
    width: float,
    height: float,
) -> None:
    if not 1 <= len(polygons) <= 4:
        raise ValueError("fixture 只允许 1--4 片")
    if abs(sum(_area(polygon) for polygon in polygons) - width * height) > 1e-6:
        raise ValueError("fixture 不覆盖目标矩形")
    for polygon in polygons:
        if not 3 <= len(polygon) <= 5 or _area(polygon) <= 0:
            raise ValueError("fixture 边数或面积非法")
        if min(_edge_lengths(polygon)) < 20.0 - 1e-8:
            raise ValueError("fixture 存在小于题面下限的边")
        if not any(
            abs(x) < 1e-8
            or abs(x - width) < 1e-8
            or abs(y) < 1e-8
            or abs(y - height) < 1e-8
            for x, y in polygon
        ):
            raise ValueError("每片必须具有目标外边")


def generate_fixture(family: str, seed: int = 1) -> PuzzleFixture:
    """由显式 ``random.Random(seed)`` 产生可复现的合法小型语料。"""
    random_source = random.Random(seed)
    width, height = 100.0, 60.0
    # 仅以 seed 决定 piece ID 的无语义置换；几何真值永不传入生产 solver。
    if family == "rectangle-1":
        polygons = (((0, 0), (width, 0), (width, height), (0, height)),)
    elif family in {"stripes-2", "corner-2"}:
        polygons = (
            ((0, 0), (50, 0), (50, height), (0, height)),
            ((50, 0), (width, 0), (width, height), (50, height)),
        )
    elif family == "stripes-3":
        polygons = (
            ((0, 0), (30, 0), (30, height), (0, height)),
            ((30, 0), (70, 0), (70, height), (30, height)),
            ((70, 0), (width, 0), (width, height), (70, height)),
        )
    elif family == "stripes-4":
        polygons = (
            ((0, 0), (25, 0), (25, height), (0, height)),
            ((25, 0), (50, 0), (50, height), (25, height)),
            ((50, 0), (75, 0), (75, height), (50, height)),
            ((75, 0), (width, 0), (width, height), (75, height)),
        )
    elif family in {"grid-4", "corner-4"}:
        polygons = (
            ((0, 0), (50, 0), (50, 30), (0, 30)),
            ((50, 0), (100, 0), (100, 30), (50, 30)),
            ((0, 30), (50, 30), (50, 60), (0, 60)),
            ((50, 30), (100, 30), (100, 60), (50, 60)),
        )
    elif family == "t-offset-3":
        # 右方短边在左片 60 mm 长边的内部，形成可解释 T junction。
        polygons = (
            ((0, 0), (40, 0), (40, 60), (0, 60)),
            ((40, 0), (100, 0), (100, 30), (40, 30)),
            ((40, 30), (100, 30), (100, 60), (40, 60)),
        )
    elif family == "concave-2":
        polygons = (
            ((0, 0), (100, 0), (100, 60), (50, 30), (0, 60)),
            ((0, 60), (50, 30), (100, 60)),
        )
    else:
        raise ValueError(f"未知 fixture family: {family}")
    _validate(family, polygons, width, height)
    indices = list(range(len(polygons)))
    random_source.shuffle(indices)
    pieces = tuple(_piece(f"fixture-{index}", polygons[index]) for index in indices)
    truth = tuple(sorted((piece.piece_id, piece.polygon_table_mm) for piece in pieces))
    return PuzzleFixture(family, seed, (width, height), pieces, truth)


FAMILIES = (
    "rectangle-1",
    "stripes-2",
    "stripes-3",
    "stripes-4",
    "grid-4",
    "corner-2",
    "corner-4",
    "t-offset-3",
    "concave-2",
)
