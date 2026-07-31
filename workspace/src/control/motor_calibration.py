"""无需目测刻度的 STS3215 两端点居中计算。"""

from __future__ import annotations

from dataclasses import dataclass

ENCODER_MIN = 0
ENCODER_MAX = 4095
HALF_TURN = 2047


def _raw_count(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} 必须是整数 raw count")
    if not ENCODER_MIN <= value <= ENCODER_MAX:
        raise ValueError(f"{name} 必须在 0..4095")
    return value


@dataclass(frozen=True)
class CenteredMotorRange:
    endpoint_min_raw: int
    endpoint_max_raw: int
    span_counts: int
    wraps_encoder_zero: bool
    midpoint_raw: int
    homing_offset: int
    range_min: int
    range_max: int


def center_motor_range(
    endpoint_a_raw: int,
    endpoint_b_raw: int,
    minimum_span_counts: int = 500,
    maximum_span_counts: int = 3500,
) -> CenteredMotorRange:
    """由两个机械端点计算 LeRobot/Feetech 居中标定。

    SO-101 的 J1–J4 有效行程均大于半圈；因此在 12-bit 环形编码器
    的两条候选弧中选择长弧。该处理覆盖端点跨越 ``4095 -> 0`` 的情况。
    先在 Homing_Offset=0 时记录端点，再令长弧中点映射到 ``2047``。
    调用方不需要目测关节中位，也不需要外部角度刻度。
    """

    endpoint_a = _raw_count(endpoint_a_raw, "endpoint_a_raw")
    endpoint_b = _raw_count(endpoint_b_raw, "endpoint_b_raw")
    if isinstance(minimum_span_counts, bool) or not isinstance(
        minimum_span_counts, int
    ):
        raise ValueError("minimum_span_counts 必须是正整数")
    if minimum_span_counts <= 0:
        raise ValueError("minimum_span_counts 必须是正整数")
    if isinstance(maximum_span_counts, bool) or not isinstance(
        maximum_span_counts, int
    ):
        raise ValueError("maximum_span_counts 必须是正整数")
    if not minimum_span_counts <= maximum_span_counts <= ENCODER_MAX:
        raise ValueError(
            "maximum_span_counts 必须不小于 minimum_span_counts 且不超过 4095"
        )

    encoder_counts = ENCODER_MAX + 1
    arc_a_to_b = (endpoint_b - endpoint_a) % encoder_counts
    arc_b_to_a = (endpoint_a - endpoint_b) % encoder_counts
    if arc_a_to_b >= arc_b_to_a:
        arc_start = endpoint_a
        span = arc_a_to_b
    else:
        arc_start = endpoint_b
        span = arc_b_to_a
    if not minimum_span_counts <= span <= maximum_span_counts:
        raise ValueError(
            "机械端点长弧跨度超出要求："
            f"{span} counts 不在 {minimum_span_counts}..{maximum_span_counts}"
        )

    midpoint = (arc_start + span // 2) % encoder_counts
    homing_offset = midpoint - HALF_TURN
    range_min, range_max = sorted(
        (
            (endpoint_a - homing_offset) % encoder_counts,
            (endpoint_b - homing_offset) % encoder_counts,
        )
    )
    if range_max - range_min != span:
        raise ValueError(
            f"端点无法映射为连续长弧范围：raw=({endpoint_a}, {endpoint_b}), span={span}"
        )
    return CenteredMotorRange(
        endpoint_min_raw=min(endpoint_a, endpoint_b),
        endpoint_max_raw=max(endpoint_a, endpoint_b),
        span_counts=span,
        wraps_encoder_zero=arc_start + span > ENCODER_MAX,
        midpoint_raw=midpoint,
        homing_offset=homing_offset,
        range_min=range_min,
        range_max=range_max,
    )
