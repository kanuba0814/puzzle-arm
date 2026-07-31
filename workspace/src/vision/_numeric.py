"""视觉模块内部的受保护数值转换。

集中处理 numpy/cv2 标量到 Python 数值的转换，使非有限值在单一位置显式失败，
而不是让每个调用点各自包裹 try/except。
"""

from __future__ import annotations

import math


def finite_float(value: object, field: str = "value") -> float:
    """把标量转为有限 Python float；非数值或非有限值显式失败。"""
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{field} 必须是有限数值，实际为 {value!r}") from error
    if not math.isfinite(result):
        raise ValueError(f"{field} 必须是有限数值，实际为 {result!r}")
    return result


def positive_float(value: object, field: str = "value") -> float:
    """把标量转为正有限 float。"""
    result = finite_float(value, field)
    if result <= 0.0:
        raise ValueError(f"{field} 必须为正数，实际为 {result!r}")
    return result


def rounded_int(value: object, field: str = "value") -> int:
    """把标量四舍五入为 int；非有限值显式失败。"""
    return int(round(finite_float(value, field)))
