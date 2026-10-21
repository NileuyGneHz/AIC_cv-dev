#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""PLCnext RSC 链路：把一张「标签表」映射到 PLC 变量上。

为什么单独一层
--------------
`plc_gateway.py` 里的状态机只认识语义标签（`CAM_DATA` / `POS_SET` / `DI_INPOS`…），
不关心这些标签是躺在 Modbus 保持寄存器里，还是躺在 PLCnext 的 GDS 变量树里。
本模块负责后者：读 YAML 标签表 → 批量 `Read`/`Write` → 返回/写入一个纯 Python 字典。
换 PLC 工程只需要改 YAML，不动一行状态机代码。

RSC 是什么
----------
PLCnext 的 Remote Service Call：TCP 41100（TLS），按 `Arp.Plc.Gds.Services.IDataAccessService`
服务读写变量，变量名形如 ``Arp.Plc.Eclr/Main.Camera.Request``。
官方 Python 客户端是 `PyPlcnextRsc`（Phoenix Contact 出品，MIT）。

**只支持 Python 3.7–3.10**：该库 `from enum import _decompose`，3.11 起 CPython 删了
这个私有符号，导入即报 `ImportError`。ROS 2 Humble 自带 python3.10，正好可用；
本机若 `python3` 指向 3.13（WorkBuddy 托管运行时就是），必须显式用 `/usr/bin/python3`。

SDK 从哪来（按优先级）
----------------------
1. 环境变量 ``PLCNEXT_RSC_WHEEL`` 指向 .whl 或已解包目录；
2. 包内 ``third_party/PyPlcnextRsc-*.whl``（纯 Python wheel，可直接加进 ``sys.path`` 当 zip 导入）；
3. 系统里 ``pip install PyPlcnextRsc`` 装好的版本。

命令行自检（不需要 PLC，--dry-run 用内存假设备）
----------------------------------------------
    python3 rsc_link.py --map ../config/plc_rsc_map.yaml --dry-run --watch
    python3 rsc_link.py --host 192.168.3.99 --read CAM_DATA --write CAM_REQ=1
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

LOG = logging.getLogger("rsc_link")

# 方向：PLC 写、网关读 / 网关写、PLC 读 / 双向
DIR_PLC2GW = "plc2gw"
DIR_GW2PLC = "gw2plc"
DIR_BOTH = "both"
DIRECTIONS = (DIR_PLC2GW, DIR_GW2PLC, DIR_BOTH)

# IEC 61131-3 标量类型（PLCnext 侧声明的类型，必须与 PLC 工程一致）
SCALAR_TYPES = (
    "BOOL", "SINT", "INT", "DINT", "LINT",
    "USINT", "UINT", "UDINT", "ULINT",
    "BYTE", "WORD", "DWORD", "LWORD",
    "REAL", "LREAL", "STRING",
)


# --------------------------------------------------------------------------- SDK
class Sdk:
    """PyPlcnextRsc 的最小门面，避免全项目到处 import 私有路径。"""

    def __init__(self) -> None:
        self.Device = None
        self.RscVariant = None
        self.RscType = None
        self.IecType = None
        self.DataTypeStore = None
        self.IDataAccessService = None
        self.WriteItem = None
        self.DataAccessError = None
        self.ExtraConfigure = None

    @property
    def ready(self) -> bool:
        return self.Device is not None


_SDK: Optional[Sdk] = None


def _wheel_candidates() -> List[Path]:
    here = Path(__file__).resolve().parent
    # 源码树：scripts/rsc_link.py -> ../third_party
    # 安装后：lib/aic_plc_gateway/rsc_link.py -> ../../share/aic_plc_gateway/third_party
    return [
        here.parent / "third_party",
        here.parent.parent / "share" / "aic_plc_gateway" / "third_party",
        here.parent.parent.parent / "share" / "aic_plc_gateway" / "third_party",
    ]


def _inject_sdk_path() -> Optional[str]:
    """把 third_party 里的 wheel（或解包目录）塞进 sys.path。返回注入的路径。"""
    env = os.environ.get("PLCNEXT_RSC_WHEEL", "").strip()
    candidates: List[Path] = [Path(env)] if env else []
    for d in _wheel_candidates():
        if d.is_dir():
            candidates.extend(sorted(d.glob("PyPlcnextRsc-*.whl")))
            candidates.append(d)          # 解包目录也行
    for p in candidates:
        if p.exists():
            sp = str(p)
            if sp not in sys.path:
                sys.path.insert(0, sp)
            return sp
    return None


def load_sdk() -> Sdk:
    """导入 PyPlcnextRsc；失败时抛出带可操作提示的 RuntimeError。"""
    global _SDK
    if _SDK is not None and _SDK.ready:
        return _SDK

    if sys.version_info >= (3, 11):
        raise RuntimeError(
            "PyPlcnextRsc 0.1.12 只支持 Python 3.7–3.10（用了 3.11 已删除的 enum._decompose）。"
            "当前是 %d.%d；请用 /usr/bin/python3（Ubuntu 22.04 自带 3.10）运行本节点。"
            % sys.version_info[:2])

    injected = None
    try:
        from PyPlcnextRsc import Device, RscVariant  # noqa: F401
    except ImportError:
        injected = _inject_sdk_path()
        try:
            from PyPlcnextRsc import Device, RscVariant  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                "找不到 PyPlcnextRsc。把官方 wheel 放到 %s，或 pip install PyPlcnextRsc。"
                "（原始错误：%s）" % (_wheel_candidates()[0], e))

    import warnings
    warnings.filterwarnings("ignore", category=FutureWarning)   # Device() 自带内部测试警告

    from PyPlcnextRsc.Arp.Plc.Gds.Services import (  # noqa: E402
        DataAccessError, IDataAccessService, WriteItem,
    )
    from PyPlcnextRsc.common.device import ExtraConfigure  # noqa: E402
    from PyPlcnextRsc.common.tag_type import IecType, RscType  # noqa: E402
    from PyPlcnextRsc.tools import DataTypeStore  # noqa: E402

    sdk = Sdk()
    sdk.Device = Device
    sdk.RscVariant = RscVariant
    sdk.RscType = RscType
    sdk.IecType = IecType
    sdk.DataTypeStore = DataTypeStore
    sdk.IDataAccessService = IDataAccessService
    sdk.WriteItem = WriteItem
    sdk.DataAccessError = DataAccessError
    sdk.ExtraConfigure = ExtraConfigure
    _SDK = sdk
    if injected:
        LOG.info("PyPlcnextRsc 从 %s 加载", injected)
    return sdk


# --------------------------------------------------------------------------- 标签表
@dataclass
class TagSpec:
    name: str
    path: str
    iec: str
    direction: str = DIR_BOTH
    note: str = ""
    # 工程量缩放：PLC 值 = 工程值 / scale（暂无使用者，留给脉冲/丝杠换算）
    scale: float = 1.0

    def __post_init__(self) -> None:
        self.iec = str(self.iec).upper()
        if self.iec not in SCALAR_TYPES:
            raise ValueError("标签 %s 的 type=%s 不是 IEC 标量（可用：%s）"
                             % (self.name, self.iec, ", ".join(SCALAR_TYPES)))
        if self.direction not in DIRECTIONS:
            raise ValueError("标签 %s 的 dir=%s 非法（可用：%s）"
                             % (self.name, self.direction, ", ".join(DIRECTIONS)))


@dataclass
class StructSpec:
    """一个 UDT 结构体变量，字段分别映射到标签。

    RSC 也能按 ``Main.Camera.Request`` 逐字段读写；但用 UDT 一次性读整块，
    在 12 位点巡视里能把 4 次往返压成 1 次，且与官方 `Camera` 契约天然对齐。
    """

    udt: str
    path: str
    schema: str
    fields: Dict[str, str]          # 字段名 -> 标签名
    direction: str = DIR_BOTH
    note: str = ""


@dataclass
class RscMap:
    tags: Dict[str, TagSpec] = field(default_factory=dict)
    structs: List[StructSpec] = field(default_factory=list)
    program: str = "Main"
    prefix: str = "Arp.Plc.Eclr/"

    @staticmethod
    def load(path: str | os.PathLike) -> "RscMap":
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}

        m = RscMap(program=str(raw.get("program", "Main")),
                   prefix=str(raw.get("prefix", "Arp.Plc.Eclr/")))
        for item in raw.get("tags", []) or []:
            spec = TagSpec(
                name=str(item["tag"]).strip(),
                path=_resolve_path(str(item["path"]), m.prefix, m.program),
                iec=str(item.get("type", "INT")),
                direction=str(item.get("dir", DIR_BOTH)).lower(),
                note=str(item.get("note", "")),
                scale=float(item.get("scale", 1.0) or 1.0),
            )
            if spec.name in m.tags:
                raise ValueError("标签名重复：%s" % spec.name)
            m.tags[spec.name] = spec

        for item in raw.get("structs", []) or []:
            st = StructSpec(
                udt=str(item["udt"]).strip(),
                path=_resolve_path(str(item["path"]), m.prefix, m.program),
                schema=str(item["schema"]),
                fields={str(k): str(v) for k, v in (item.get("fields") or {}).items()},
                direction=str(item.get("dir", DIR_BOTH)).lower(),
                note=str(item.get("note", "")),
            )
            for field_name, tag in st.fields.items():
                if tag not in m.tags:
                    raise ValueError("结构体 %s 的字段 %s 指向未定义标签 %s"
                                     % (st.udt, field_name, tag))
            if st.direction == DIR_BOTH:
                # 整块写会把 PLC 正在写的字段一起覆盖回去（典型：Camera.Request 被算法
                # 用上一周期的 FALSE 冲掉）。要么整块归算法所有，要么只读整块 + 逐字段写。
                bad = [t for t in st.fields.values()
                       if m.tags[t].direction != DIR_GW2PLC]
                if bad:
                    raise ValueError(
                        "结构体 %s 声明 dir=both，但字段标签 %s 里含非 gw2plc 方向；"
                        "整块写会覆盖 PLC 侧的写入。请把结构体改成 dir=plc2gw（整块读、"
                        "逐字段写），或把这些标签都改成 gw2plc。" % (st.udt, bad))
            m.structs.append(st)

        if not m.tags:
            raise ValueError("%s 里没有任何 tags" % path)
        return m

    def describe(self) -> str:
        n_r = sum(1 for t in self.tags.values() if t.direction in (DIR_PLC2GW, DIR_BOTH))
        n_w = sum(1 for t in self.tags.values() if t.direction in (DIR_GW2PLC, DIR_BOTH))
        return ("program=%s  标签 %d（读 %d / 写 %d）  结构体 %d  前缀 %s"
                % (self.program, len(self.tags), n_r, n_w, len(self.structs), self.prefix))


def _resolve_path(path: str, prefix: str, program: str) -> str:
    path = path.strip()
    if path.startswith("Arp."):
        return path
    path = path.replace("{program}", program)
    return prefix + path.lstrip("/")


def _coerce(value: Any, iec: str) -> Any:
    if iec == "BOOL":
        return bool(value)
    if iec in ("REAL", "LREAL"):
        return float(value)
    if iec == "STRING":
        return "" if value is None else str(value)
    return int(value)


# --------------------------------------------------------------------------- 链路
RECONNECT_MIN_S = 2.0
RECONNECT_MAX_S = 10.0


class RscLink:
    """一条到 PLCnext 的 RSC 连接 + 一张标签表。

    线程模型：``read_all`` / ``write_all`` 是阻塞的（TCP 往返，毫秒级），
    由调用方放在自己的工作线程里；本类不加锁，也不起线程。
    """

    def __init__(self, mapping: RscMap, host: str, port: int = 41100,
                 user: Optional[str] = None, password: Optional[str] = None,
                 timeout_s: float = 5.0, dry_run: bool = False,
                 device: Optional[Any] = None) -> None:
        self.map = mapping
        self.host = host
        self.port = int(port)
        self.user = user
        self.password = password
        self.timeout_s = float(timeout_s)
        self.dry_run = bool(dry_run) or device is not None

        self.sdk = load_sdk()
        self._device: Optional[Any] = None
        self._svc: Optional[Any] = None
        self._schema: Optional[Any] = None
        # 只有「结构体负责写」的那些字段才禁止再逐字段写；只读整块的结构体
        # （dir=plc2gw）仍允许按字段把算法的结果写回去。
        self._struct_by_tag: Dict[str, Tuple[StructSpec, str]] = {}
        for st in mapping.structs:
            if st.direction in (DIR_GW2PLC, DIR_BOTH):
                for f_name, tag in st.fields.items():
                    self._struct_by_tag[tag] = (st, f_name)

        self._last_written: Dict[str, Any] = {}
        self._failed: Dict[str, int] = {}        # 标签 -> 最近一次错误码（只报一次）
        # 断链重连：PLC 重启 / 网线拔插后不必重启网关。退避从 2 s 翻倍到 10 s，
        # 否则每个 tick 都去 connect（不可达时会阻塞到 timeout_s），把 50 Hz 轮询拖死。
        self._backoff = RECONNECT_MIN_S
        self._next_try = 0.0
        self.reconnects = 0
        self.connected = False
        self.reads = 0
        self.writes = 0
        self.last_error: Optional[str] = None
        self.connected_at: Optional[float] = None

    # ---- 生命周期 ----
    def connect(self) -> None:
        if self._svc is not None:
            return
        if self.dry_run:
            self._init_schema()
            self._svc = _DryRunService(self)
            self.connected = True
            self.connected_at = time.time()
            LOG.info("RSC 干跑模式（内存假 PLC），标签表：%s", self.map.describe())
            return

        supplier = None
        if self.user:
            user, pwd = self.user, self.password or ""
            supplier = lambda: (user, pwd)          # noqa: E731

        cfg = self.sdk.ExtraConfigure()
        cfg.timeout = max(1, int(round(self.timeout_s)))
        dev = self.sdk.Device(self.host, port=self.port, secureInfoSupplier=supplier,
                              config=cfg)
        dev.connect()
        self._device = dev
        self._svc = self.sdk.IDataAccessService(dev)
        self._init_schema()
        self.connected = True
        self.connected_at = time.time()
        self._last_written.clear()
        LOG.info("RSC 已连接 %s:%d（%s）", self.host, self.port, self.map.describe())

    def _ensure_connected(self) -> None:
        """没连接就尝试重连（带退避）。失败抛 RuntimeError，调用方按「本轮跳过」处理。"""
        if self._svc is not None:
            return
        # 用单调钟：现场校时 / NTP 跳变不该让退避窗口失效
        now = time.monotonic()
        if now < self._next_try:
            raise RuntimeError(
                "RSC 未连接（%.0f s 后重试；上次错误：%s）"
                % (self._next_try - now, self.last_error or "无"))
        # 首次连接不算「重连」：connected_at 为空才是第一次
        first = self.connected_at is None
        try:
            self.connect()
        except Exception as e:      # noqa: BLE001 - 连不上不是致命错误
            self.last_error = str(e)
            self._next_try = now + self._backoff
            wait = self._backoff
            self._backoff = min(self._backoff * 2, RECONNECT_MAX_S)
            LOG.warning("RSC 重连失败：%s；%.0f s 后重试", e, wait)
            raise RuntimeError("RSC 重连失败：%s" % e) from e
        if not first:
            self.reconnects += 1
            LOG.info("RSC 已恢复连接（累计重连 %d 次）", self.reconnects)
        self._backoff = RECONNECT_MIN_S
        self.last_error = None
        self._next_try = 0.0

    def _init_schema(self) -> None:
        if not self.map.structs:
            return
        text = "\n".join(s.schema for s in self.map.structs)
        self._schema = self.sdk.DataTypeStore.fromString(text)

    def close(self) -> None:
        self.connected = False
        self._svc = None
        if self._device is not None:
            try:
                self._device.dispose()
            except Exception as e:      # noqa: BLE001 - 断链时任何异常都不该抛给上层
                LOG.debug("dispose 失败：%s", e)
            self._device = None

    # ---- 读 ----
    def read_all(self, include_writes: bool = False) -> Dict[str, Any]:
        """读 PLC→网关 的标签，返回 {标签: python 值}。单个变量不存在不致命。

        include_writes=True 时把 gw2plc 标签也读上来。网关状态机不用它（只需输入），
        但手工调试要能「写下去再读回来」，否则 --write X=1 --read X 永远是 None。
        """
        self._ensure_connected()
        dirs = (DIR_PLC2GW, DIR_BOTH) + ((DIR_GW2PLC,) if include_writes else ())
        reads = [t for t in self.map.tags.values() if t.direction in dirs]
        structs = [s for s in self.map.structs if s.direction in dirs]
        if not reads and not structs:
            return {}

        names = [t.path for t in reads] + [s.path for s in structs]
        try:
            items = self._svc.Read(names)
        except Exception as e:      # noqa: BLE001
            # close() 会把 _svc 置 None，下一次调用才会走重连；
            # 只把 connected 置 False 的话 _svc 还在，会拿着坏掉的句柄反复失败。
            self.close()
            raise RuntimeError("RSC 读失败：%s" % e) from e

        out: Dict[str, Any] = {}
        for spec, item in zip(reads, items[:len(reads)]):
            if self._check(spec.name, item.Error):
                value = _coerce(item.Value.GetValue(), spec.iec)
                out[spec.name] = value * spec.scale if spec.scale != 1.0 else value
        for st, item in zip(structs, items[len(reads):]):
            if not self._check(st.udt, item.Error):
                continue
            try:
                obj = self._schema.ReceiveAsSchemaInstance(st.udt, item.Value)
            except Exception as e:      # noqa: BLE001
                self._warn_once(st.udt, "UDT 解码失败（PLC 侧类型与 YAML schema 是否一致？）：%s" % e)
                continue
            for f_name, tag in st.fields.items():
                if self.map.tags[tag].direction not in dirs:
                    continue
                try:
                    raw = getattr(obj, f_name)
                except AttributeError:
                    self._warn_once(tag, "UDT %s 没有字段 %s" % (st.udt, f_name))
                    continue
                out[tag] = _coerce(raw, self.map.tags[tag].iec)
        self.reads += 1
        return out

    # ---- 写 ----
    def write_all(self, values: Dict[str, Any], force: bool = False) -> int:
        """把网关→PLC 的标签写下去；只写变化过的（force=True 则全写）。

        返回成功写入的标签数。写失败只记一次日志并把该标签拉黑到下次重连，
        避免「变量名拼错」时每 50 ms 刷一行错误。
        """
        self._ensure_connected()

        items: List[Any] = []
        pending: Dict[str, Any] = {}
        for name, spec in self.map.tags.items():
            if spec.direction not in (DIR_GW2PLC, DIR_BOTH):
                continue
            if name in self._struct_by_tag or name not in values or name in self._failed:
                continue
            raw = _coerce(values[name], spec.iec)
            if spec.scale != 1.0:
                raw = int(round(raw / spec.scale))
            if not force and self._last_written.get(name) == raw:
                continue
            items.append(self.sdk.WriteItem(spec.path, self.sdk.RscVariant(raw, self._type_of(spec.iec))))
            pending[name] = raw

        struct_pending: Dict[str, Dict[str, Any]] = {}
        for st in self.map.structs:
            if st.direction not in (DIR_GW2PLC, DIR_BOTH):
                continue
            if any(t not in values for t in st.fields.values()):
                continue
            snapshot = {f: _coerce(values[t], self.map.tags[t].iec) for f, t in st.fields.items()}
            if not force and self._last_written.get(st.path) == snapshot:
                continue
            try:
                inst = self._schema.NewSchemaInstance(st.udt)
                for f_name, v in snapshot.items():
                    setattr(inst, f_name, v)
                items.append(self.sdk.WriteItem(st.path, self.sdk.RscVariant.of(inst)))
            except Exception as e:      # noqa: BLE001
                self._warn_once(st.udt, "UDT 组装失败：%s" % e)
                continue
            struct_pending[st.path] = snapshot

        if not items:
            return 0

        try:
            errs = self._svc.Write(tuple(items))
        except Exception as e:      # noqa: BLE001
            self.close()
            raise RuntimeError("RSC 写失败：%s" % e) from e

        ok = 0
        for i, err in enumerate(errs):
            code = _err_code(err)
            if code == 0:
                ok += 1
                continue
            name = list(pending)[i] if i < len(pending) else "struct[%d]" % i
            self._failed[name] = code
            LOG.warning("RSC 写入 %s 被拒：%s（变量名/类型是否与 PLC 工程一致？）",
                        name, _err_name(self.sdk, code))

        for name, v in pending.items():
            if name not in self._failed:
                self._last_written[name] = v
        for path, snap in struct_pending.items():
            self._last_written[path] = snap
        self.writes += 1
        return ok

    def _type_of(self, iec: str) -> Any:
        try:
            return getattr(self.sdk.IecType, iec)
        except AttributeError as e:
            raise ValueError("不支持的 IEC 类型：%s" % iec) from e

    # ---- 诊断 ----
    def _check(self, name: str, err: Any) -> bool:
        code = _err_code(err)
        if code == 0:
            self._failed.pop(name, None)
            return True
        self._warn_once(name, "RSC 读 %s 失败：%s" % (name, _err_name(self.sdk, code)))
        return False

    def _warn_once(self, key: str, text: str) -> None:
        if key not in self._failed:
            self._failed[key] = -1
            LOG.warning(text)

    def health(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "host": self.host,
            "port": self.port,
            "reads": self.reads,
            "writes": self.writes,
            "failed_tags": sorted(self._failed),
            "dry_run": self.dry_run,
        }


def _err_code(err: Any) -> int:
    if err is None:
        return 0
    try:
        return int(getattr(err, "value", err))
    except (TypeError, ValueError):
        return 255


def _err_name(sdk: Sdk, code: int) -> str:
    try:
        return sdk.DataAccessError(code).name
    except Exception:       # noqa: BLE001
        return "code=%d" % code


# --------------------------------------------------------------------------- 干跑
class _DryRunService:
    """内存假 PLC：接口与 IDataAccessService 的 Read/Write 对齐，用来验证标签表本身。

    它不模拟 PLC 程序逻辑，只保证「写进去的值能原样读出来」，
    因此能抓出：变量名笔误之外的映射错误、类型不匹配、UDT 字段名写错。
    """

    def __init__(self, link: RscLink) -> None:
        self.link = link
        self.store: Dict[str, Any] = {}

    def Read(self, names: Iterable[str]) -> Tuple[Any, ...]:
        out = []
        for name in names:
            variant = self._get(name)
            out.append(_FakeReadItem(0, variant))
        time.sleep(0.002)               # 让干跑也有真实往返的量级
        return tuple(out)

    def Write(self, items: Iterable[Any]) -> Tuple[int, ...]:
        errs = []
        for it in items:
            self.store[it.PortName] = it.Value
            errs.append(0)
        time.sleep(0.002)
        return tuple(errs)

    def _get(self, path: str) -> Any:
        sdk = self.link.sdk
        if path in self.store:
            return self.store[path]
        for st in self.link.map.structs:
            if st.path == path:
                inst = self.link._schema.NewSchemaInstance(st.udt)
                return sdk.RscVariant.of(inst)
        spec = next((t for t in self.link.map.tags.values() if t.path == path), None)
        iec = spec.iec if spec else "INT"
        zero = False if iec == "BOOL" else (0.0 if iec in ("REAL", "LREAL") else ("" if iec == "STRING" else 0))
        return sdk.RscVariant(zero, self.link._type_of(iec))


@dataclass
class _FakeReadItem:
    Error: int
    Value: Any


# --------------------------------------------------------------------------- CLI
def _default_map() -> Path:
    here = Path(__file__).resolve().parent
    for c in (here.parent / "config" / "plc_rsc_map.yaml",
              here.parent.parent / "share" / "aic_plc_gateway" / "config" / "plc_rsc_map.yaml"):
        if c.exists():
            return c
    return here.parent / "config" / "plc_rsc_map.yaml"


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="PLCnext RSC 标签表自检 / 手动读写")
    ap.add_argument("--map", default=str(_default_map()), help="标签表 YAML")
    ap.add_argument("--host", default=os.environ.get("PLCNEXT_HOST", "192.168.3.99"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("PLCNEXT_PORT", "41100")))
    ap.add_argument("--user", default=os.environ.get("PLCNEXT_USER", "admin"))
    ap.add_argument("--password-env", default="PLCNEXT_PASSWORD")
    ap.add_argument("--dry-run", action="store_true", help="不连 PLC，用内存假设备验证标签表")
    ap.add_argument("--read", action="append", default=[], help="读一个标签，可重复")
    ap.add_argument("--write", action="append", default=[], help="写 TAG=VALUE，可重复")
    ap.add_argument("--watch", action="store_true", help="每秒打印一次全部标签")
    ap.add_argument("--describe", action="store_true", help="只打印标签表")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")
    mapping = RscMap.load(args.map)
    print(mapping.describe())

    if args.describe:
        for t in mapping.tags.values():
            print("  %-12s %-9s %-6s %s" % (t.name, t.iec, t.direction, t.path))
        for s in mapping.structs:
            print("  %-12s UDT      %-6s %s" % (s.udt, s.direction, s.path))
        return 0

    pwd = os.environ.get(args.password_env)
    link = RscLink(mapping, args.host, args.port, args.user, pwd, dry_run=args.dry_run)
    link.connect()
    try:
        for w in args.write:
            name, _, val = w.partition("=")
            v: Any = val
            if val.lower() in ("true", "false"):
                v = val.lower() == "true"
            elif "." not in val:
                v = int(val)
            link.write_all({name.strip(): v}, force=True)
            print("写 %s = %r" % (name.strip(), v))

        for _ in range(2 if args.watch else 1):
            vals = link.read_all(include_writes=bool(args.read))
            if args.read:
                for name in args.read:
                    print("  %-12s = %r" % (name, vals.get(name)))
            else:
                print("  " + "  ".join("%s=%r" % (k, v) for k, v in sorted(vals.items())))
            if args.watch:
                time.sleep(1.0)
        print("健康状态：%s" % link.health())
        return 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
