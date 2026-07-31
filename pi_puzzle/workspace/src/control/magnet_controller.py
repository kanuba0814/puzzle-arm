"""GPIO17 继电器电磁铁控制器。

该模块只暴露布尔状态接口，不允许调用方直接操作 GPIO。默认断电，单次通电
超过配置上限会自动关闭；急停和关闭路径都会先关闭输出。
"""

from __future__ import annotations

import importlib
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class MagnetConfig:
    """电磁铁 GPIO 与安全限制。line_offset 使用 BCM GPIO 编号。"""

    chip_path: str = "/dev/gpiochip0"
    line_offset: int = 17
    active_high: bool = False
    max_on_seconds: float = 10.0
    consumer: str = "epuzzle-magnet"

    def __post_init__(self) -> None:
        if self.line_offset < 0:
            raise ValueError("GPIO line offset 必须非负")
        if self.max_on_seconds <= 0:
            raise ValueError("电磁铁通电超时必须大于 0 秒")


@dataclass(frozen=True)
class MagnetStatus:
    enabled: bool
    reason: str
    monotonic_ns: int
    line_offset: int


class DigitalOutput(Protocol):
    def set_active(self, active: bool) -> None:
        """设置逻辑激活状态。"""

    def close(self) -> None:
        """释放输出资源。"""


class TimerHandle(Protocol):
    daemon: bool

    def start(self) -> None:
        """启动定时器。"""

    def cancel(self) -> None:
        """取消定时器。"""


OutputFactory = Callable[[MagnetConfig], DigitalOutput]
TimerFactory = Callable[[float, Callable[[], None]], TimerHandle]
StatusCallback = Callable[[MagnetStatus], None]


class _GpiodOutput:
    def __init__(self, config: MagnetConfig) -> None:
        try:
            gpiod = importlib.import_module("gpiod")
        except ImportError as error:
            raise RuntimeError("缺少 python3-gpiod，禁止控制电磁铁") from error

        settings = gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT,
            active_low=not config.active_high,
            output_value=gpiod.line.Value.INACTIVE,
        )
        self._gpiod = gpiod
        self._offset = config.line_offset
        self._request = gpiod.request_lines(
            config.chip_path,
            consumer=config.consumer,
            config={config.line_offset: settings},
        )

    def set_active(self, active: bool) -> None:
        value = (
            self._gpiod.line.Value.ACTIVE if active else self._gpiod.line.Value.INACTIVE
        )
        self._request.set_value(self._offset, value)

    def close(self) -> None:
        self._request.release()


def _default_output_factory(config: MagnetConfig) -> DigitalOutput:
    return _GpiodOutput(config)


class MagnetController:
    """线程安全的电磁铁控制器；Supervisor/GUI 后端只调用 set_enabled。"""

    def __init__(
        self,
        config: MagnetConfig | None = None,
        *,
        output_factory: OutputFactory = _default_output_factory,
        timer_factory: TimerFactory = threading.Timer,
        on_status: StatusCallback | None = None,
    ) -> None:
        self.config = config or MagnetConfig()
        self._output_factory = output_factory
        self._timer_factory = timer_factory
        self._on_status = on_status or (lambda status: None)
        self._lock = threading.RLock()
        self._output: DigitalOutput | None = None
        self._timer: TimerHandle | None = None
        self._status = MagnetStatus(
            enabled=False,
            reason="not_connected",
            monotonic_ns=time.monotonic_ns(),
            line_offset=self.config.line_offset,
        )

    def connect(self) -> MagnetStatus:
        """独占请求 GPIO，并显式写入安全关闭状态。"""
        with self._lock:
            if self._output is not None:
                return self._status
            output = self._output_factory(self.config)
            try:
                output.set_active(False)
            except Exception:
                output.close()
                raise
            self._output = output
            status = self._record_locked(False, "connected_safe_off")
        self._on_status(status)
        return status

    def set_enabled(self, enabled: bool) -> MagnetStatus:
        """唯一控制接口：True=吸合，False=释放。"""
        with self._lock:
            output = self._require_output_locked()
            self._cancel_timer_locked()
            try:
                output.set_active(bool(enabled))
            except Exception:
                try:
                    output.set_active(False)
                finally:
                    self._record_locked(False, "gpio_error")
                raise
            reason = "command_on" if enabled else "command_off"
            status = self._record_locked(bool(enabled), reason)
            if enabled:
                timer = self._timer_factory(
                    self.config.max_on_seconds, self._timeout_off
                )
                timer.daemon = True
                self._timer = timer
                timer.start()
        self._on_status(status)
        return status

    def emergency_off(self) -> MagnetStatus:
        """急停路径：取消定时器并立即写入关闭状态。"""
        with self._lock:
            output = self._require_output_locked()
            self._cancel_timer_locked()
            output.set_active(False)
            status = self._record_locked(False, "emergency_off")
        self._on_status(status)
        return status

    def get_status(self) -> MagnetStatus:
        with self._lock:
            return self._status

    def close(self) -> None:
        """先断电再释放 line；可重复调用。"""
        status = None
        with self._lock:
            self._cancel_timer_locked()
            output = self._output
            if output is None:
                return
            try:
                output.set_active(False)
                status = self._record_locked(False, "closed_safe_off")
            finally:
                output.close()
                self._output = None
        if status is not None:
            self._on_status(status)

    def _timeout_off(self) -> None:
        with self._lock:
            output = self._output
            if output is None or not self._status.enabled:
                return
            output.set_active(False)
            self._timer = None
            status = self._record_locked(False, "timeout")
        self._on_status(status)

    def _require_output_locked(self) -> DigitalOutput:
        if self._output is None:
            raise RuntimeError("MagnetController 尚未连接 GPIO")
        return self._output

    def _cancel_timer_locked(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _record_locked(self, enabled: bool, reason: str) -> MagnetStatus:
        self._status = MagnetStatus(
            enabled=enabled,
            reason=reason,
            monotonic_ns=time.monotonic_ns(),
            line_offset=self.config.line_offset,
        )
        return self._status
