#!/usr/bin/env python3
"""Single public entry for SLAM 2D interface acceptance checks.

The tool intentionally does not switch systemd services. Run each mode only
after the corresponding mapping or localization stack is ready.

Functional test IDs are scoped by mode:
  MAP-A-xx: mapping automated checks
  LOC-A-xx: localization automated checks
Safety gates use MAP-G-xx or LOC-G-xx and are not functional test cases.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
import random
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable


DEFAULT_MAPS_ROOT = Path(
    os.environ.get("SLAM_2D_MAPS_ROOT", "/home/dexforce/.dexe/data/maps")
)
DEFAULT_RESULT_ROOT = Path(
    os.environ.get(
        "SLAM_2D_TEST_RESULT_ROOT",
        str(Path.home() / ".dexe/localization/slam_2d_interface_test"),
    )
)
DEFAULT_MANUAL_RESULT_ROOT = Path(
    os.environ.get(
        "SLAM_2D_MANUAL_RESULT_ROOT",
        str(Path.home() / ".dexe/localization/slam_2d_manual"),
    )
)
DEFAULT_LOCALIZATION_LOG = Path(
    os.environ.get(
        "SLAM_2D_LOCALIZATION_LOG",
        "/var/log/syslog",
    )
)
DEFAULT_LOCALIZATION_LOG_TAG = os.environ.get(
    "SLAM_2D_LOCALIZATION_LOG_TAG", "dexe_localization"
)
ALLOWED_TEST_PREFIXES = ("slam2d_test_", "mm_test_", "test_")
PASS = "PASS"
FAIL = "FAIL"
ERROR = "ERROR"
PENDING_ENV = "PENDING_ENV"
RELOCALIZATION_TRIALS = 10
RELOCALIZING_STATE_OBSERVATION_TIMEOUT_S = 5.0
STATE_CAPTURE_BASELINE_TIMEOUT_S = 15.0
MAP_RESTORE_ATTEMPTS = 2
MAP_RESTORE_STATE_TIMEOUT_S = 30.0
SLAM_MAP_FILES = ("map.pbstream", "map.png", "map.yaml")
MODEL_C_OWNER_CANDIDATES = (
    (
        "/command_board",
        "/command_board/backup_unified",
        "/command_board/rollback_map",
    ),
    (
        "/command_board_manager",
        "/command_board_manager/backup_unified",
        "/command_board_manager/rollback_map",
    ),
    (
        "/nav2_command_board",
        "/nav2_command_board/backup_unified",
        "/nav2_command_board/rollback_map",
    ),
)
TEST_ACTION_FLOWS = {
    "MAP-A-01": "读取 ROS2 node/service/topic 图，识别当前栈，并确认建图与定位业务栈没有同时运行。",
    "MAP-A-02": "读取建图输入 Topic、输出 Topic 和控制 Service 的名称与类型，逐项对照接口基线并汇总判定。",
    "MAP-A-03": "扫描旧服务、旧 Topic 和旧离线节点名，确认整理前接口均不存在。",
    "MAP-A-04": "创建受管测试图哨兵，调用 start_mapping(map_name, overwrite=true)，并轮询 MappingState 到 MAPPING。",
    "MAP-A-05": "检查同名 start_mapping 后旧哨兵和旧目录均被清除，且只重建空 slam 目录。",
    "MAP-A-06": "调用 cancel_mapping，轮询 MappingState 回到 IDLE，并按受管状态清理测试图。",
    "LOC-A-01": "读取 ROS2 node/service/topic 图，识别当前栈，并确认定位与建图业务栈没有同时运行。",
    "LOC-A-02": "读取定位输入 Topic、输出 Topic 和业务 Service 的名称与类型，逐项对照接口基线；另核对 set_pose 发布/订阅端点。",
    "LOC-A-03": "扫描旧服务、旧 Topic 和旧离线节点名，确认整理前接口均不存在。",
    "LOC-A-04": "以 transient-local/reliable QoS 单次读取 MapState，解析 folder、name 和 epoch。",
    "LOC-A-05": "单次读取 LocalizationState，解析状态码与 score，并执行范围检查。",
    "LOC-A-06": "读取 UKF frequency 参数，核对配置的滤波频率为 25 Hz。",
    "LOC-A-07": "在固定采样窗口执行 ros2 topic hz，计算 filtered odometry 实际频率并对照 23~27 Hz。",
    "LOC-A-08": "定位实际安装的 ukf_node，读取 build.sh 写入的 cap_sys_nice file capability，并用 ldd 核对 librl_lib.so 来自同一 install；再定位运行 PID，用 chrt 和 taskset 读取实时优先级与 CPU 亲和性。",
    "LOC-A-09": "记录原 MapState，调用测试图 load_map，并轮询同名地图 epoch 精确增加 1。",
    "LOC-A-10": "地图加载后持续采样 LocalizationState，确认最终进入 status=4。",
    "LOC-A-11": "先订阅 LocalizationState 并记录定位日志字节偏移，再调用全局重定位服务；优先确认 3→4，快速切换漏采状态 3 时以本轮成功日志和新状态 4 闭环，并从日志提取耗时与得分。",
    "LOC-A-12": "调用 localization save_map，解析 success、map_path 和 backup_ts，并核对保存路径。",
    "LOC-A-13": "连续执行全局重定位 10 次；每次先建立状态订阅和本轮日志边界，再调用服务，从新增日志提取真实定位耗时/最终得分，并结合状态闭环汇总成功率。",
    "LOC-A-14": "读取当前全局位姿，随机加入较大位置/角度扰动；逐轮打印扰动距离、方向、角度以及基准/请求位姿；每次建立状态订阅和本轮日志边界，再调用给定位姿重定位；优先读取 Pose relocalization took 算法耗时，并分别记录观察器基线、服务调用、status 3 出现和 3→4 恢复耗时后统计成功率。",
    "LOC-A-15": "从合法 command board 候选中发现唯一完整 Model C owner，核对服务类型并保存测试前哈希与资产语义摘要；执行统一备份、指定历史版本回滚和重新定位，最后回滚到新建基线备份并复核 epoch、文件、资产与定位状态。",
    "MAP-G-01": "检查是否显式提供 --confirm-mutations；缺失时 fail-closed，不执行地图变更。",
    "MAP-G-02": "变更测试图前读取 MappingState，只有 IDLE 才继续。",
    "MAP-G-03": "记录受管测试图路径与创建者，结束时只清理本轮受管对象。",
    "LOC-G-01": "检查是否显式提供 --confirm-mutations；缺失时 fail-closed，不执行地图变更。",
    "LOC-G-02": "校验测试地图名属于安全前缀并记录测试前当前地图，失败时不继续。",
    "LOC-G-03": "在 finally 路径读取恢复前 MapState，以当前 epoch+1 为本轮精确期望；最多两次调用 load_map 恢复测试前地图，逐次打印服务响应、期望/实际地图名和 epoch。",
}
_UNORDERED_ASSET_ARRAY_PATHS = {
    ("points",),
    ("routes",),
    ("areas",),
    ("route_graph", "junctions"),
    ("route_graph", "edges"),
    ("dsg", "layers", "ROOMS"),
    ("dsg", "layers", "PLACES"),
    ("dsg", "layers", "FLOORS"),
    ("dsg", "layers", "OBJECTS"),
    ("dsg", "layers", "BUILDINGS"),
    ("dsg", "edges"),
}
_UNORDERED_ASSET_MEMBER_KEYS = {
    "additional_area_names",
    "manual_anchor_point_names",
    "manual_linked_point_names",
    "related_junction_ids",
    "related_nav_point_names",
}


@dataclass
class CommandRecord:
    command: str
    returncode: int
    duration_s: float
    stdout: str
    stderr: str
    timed_out: bool = False
    operation_id: int = 0


@dataclass
class TestResult:
    tc: str
    name: str
    status: str
    detail: str
    action_flow: str
    interfaces: list["InterfaceCheck"] = field(default_factory=list)


@dataclass(frozen=True)
class InterfaceCheck:
    kind: str
    name: str
    expected: str
    actual: str
    normal: bool


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def timestamp() -> str:
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def decode(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


class Reporter:
    def __init__(
        self, command: str, artifact_dir: Path, *, verbose: bool = False
    ) -> None:
        self.command = command
        self.artifact_dir = artifact_dir
        self.verbose = verbose
        self.started_at = now_iso()
        self.results: list[TestResult] = []
        self.commands: list[CommandRecord] = []
        self.metadata: dict[str, Any] = {}
        self._next_operation_id = 1
        self._next_script_step_id = 1
        self.script_steps: list[dict[str, Any]] = []
        artifact_dir.mkdir(parents=True, exist_ok=False)

    def script_step(self, scope: str, action: str, detail: str) -> None:
        step_id = self._next_script_step_id
        self._next_script_step_id += 1
        step = {
            "step_id": step_id,
            "scope": scope,
            "action": action,
            "detail": detail,
            "at": now_iso(),
        }
        self.script_steps.append(step)
        if self.verbose:
            print(
                f"  [STEP {step_id:03d}] {scope} | {action}"
                + (f" — {detail}" if detail else "")
            )

    def operation_start(self, command: Iterable[str], detail: str) -> int:
        operation_id = self._next_operation_id
        self._next_operation_id += 1
        if self.verbose:
            print(
                f"  [OP {operation_id:03d} START] "
                f"{shlex.join(str(v) for v in command)}"
            )
            print(f"  [OP {operation_id:03d} INFO ] {detail}")
        return operation_id

    @staticmethod
    def _print_operation_stream(
        operation_id: int, stream_name: str, value: str
    ) -> None:
        if not value:
            print(f"  [OP {operation_id:03d} {stream_name:<6}] <empty>")
            return
        print(f"  [OP {operation_id:03d} {stream_name:<6}]")
        for line in value.splitlines():
            print(f"    │ {line}")

    def operation_finish(self, record: CommandRecord) -> None:
        self.commands.append(record)
        if not self.verbose:
            return
        result = "PASS" if record.returncode == 0 else "FAIL"
        print(
            f"  [OP {record.operation_id:03d} END  ] {result} "
            f"rc={record.returncode} duration={record.duration_s:.3f}s "
            f"timed_out={record.timed_out}"
        )
        self._print_operation_stream(
            record.operation_id, "STDOUT", record.stdout
        )
        self._print_operation_stream(
            record.operation_id, "STDERR", record.stderr
        )

    def add(
        self,
        tc: str,
        name: str,
        status: str,
        detail: str,
        interfaces: Iterable[InterfaceCheck] = (),
    ) -> None:
        checked_interfaces = list(interfaces)
        if not checked_interfaces:
            checked_interfaces.append(
                InterfaceCheck(
                    kind="oracle",
                    name=tc,
                    expected=PASS,
                    actual=status,
                    normal=status == PASS,
                )
            )
        self.results.append(
            TestResult(
                tc,
                name,
                status,
                detail,
                TEST_ACTION_FLOWS.get(tc, "执行该检查并按期望/实际结果判定。"),
                checked_interfaces,
            )
        )
        symbol = {PASS: "✓", FAIL: "✗", ERROR: "!", PENDING_ENV: "…"}.get(
            status, "-"
        )
        print(f"\n[TEST] {tc} {name}")
        print(f"  [ACTION  ] {TEST_ACTION_FLOWS.get(tc, '执行该检查并记录证据。')}")
        for interface in checked_interfaces:
            interface_status = "NORMAL" if interface.normal else "ABNORMAL"
            print(
                f"  [{interface_status:<8}] {interface.kind} "
                f"{interface.name} | expected={interface.expected} "
                f"| actual={interface.actual}"
            )
        print(f"[{status:<11}] {symbol} {tc}: {detail}")

    def run(
        self,
        args: Iterable[str],
        *,
        timeout_s: float = 15.0,
    ) -> CommandRecord:
        command = [str(value) for value in args]
        operation_id = self.operation_start(
            command, f"同步执行；timeout={timeout_s:.1f}s"
        )
        started = time.monotonic()
        timed_out = False
        try:
            completed = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout_s,
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
            )
            returncode = completed.returncode
            stdout = completed.stdout
            stderr = completed.stderr
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            returncode = 124
            stdout = decode(exc.stdout)
            stderr = decode(exc.stderr)
        except OSError as exc:
            returncode = 127
            stdout = ""
            stderr = str(exc)
        record = CommandRecord(
            command=shlex.join(command),
            returncode=returncode,
            duration_s=round(time.monotonic() - started, 3),
            stdout=stdout.strip(),
            stderr=stderr.strip(),
            timed_out=timed_out,
            operation_id=operation_id,
        )
        self.operation_finish(record)
        return record

    def run_for(self, args: Iterable[str], duration_s: float) -> CommandRecord:
        command = [str(value) for value in args]
        operation_id = self.operation_start(
            command, f"限时采样；duration={duration_s:.1f}s"
        )
        started = time.monotonic()
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
            )
            try:
                stdout, stderr = process.communicate(timeout=duration_s)
                returncode = process.returncode
                timed_out = False
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    stdout, stderr = process.communicate(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    stdout, stderr = process.communicate()
                returncode = 124
                timed_out = True
        except OSError as exc:
            stdout, stderr = "", str(exc)
            returncode = 127
            timed_out = False
        record = CommandRecord(
            command=shlex.join(command),
            returncode=returncode,
            duration_s=round(time.monotonic() - started, 3),
            stdout=stdout.strip(),
            stderr=stderr.strip(),
            timed_out=timed_out,
            operation_id=operation_id,
        )
        self.operation_finish(record)
        return record

    def finalize(self) -> int:
        counts = {
            status: sum(result.status == status for result in self.results)
            for status in (PASS, FAIL, ERROR, PENDING_ENV)
        }
        if counts[FAIL] or counts[ERROR]:
            overall = FAIL
            exit_code = 1
        elif counts[PENDING_ENV]:
            overall = "INCOMPLETE"
            exit_code = 2
        else:
            overall = PASS
            exit_code = 0

        payload = {
            "schema_version": 4,
            "command": self.command,
            "console_verbosity": "verbose" if self.verbose else "summary",
            "started_at": self.started_at,
            "finished_at": now_iso(),
            "overall": overall,
            "counts": counts,
            "metadata": self.metadata,
            "script_steps": self.script_steps,
            "tests": [asdict(result) for result in self.results],
            "commands": [asdict(command) for command in self.commands],
        }
        atomic_json(self.artifact_dir / "summary.json", payload)
        atomic_json(
            self.artifact_dir / "commands.json",
            {"commands": payload["commands"]},
        )
        operation_lines = [
            "# 自动测试具体操作记录",
            "",
            "> 每个 OP 对应脚本实际执行的一条命令；保留完整命令、返回码、耗时及原始输出。",
            "",
            "## 脚本内部步骤",
            "",
            "| 步骤 | 范围 | 具体操作 | 详细 |",
            "|---:|---|---|---|",
        ]
        for step in self.script_steps:
            values = (
                step["step_id"],
                step["scope"],
                step["action"],
                step["detail"],
            )
            escaped = [str(value).replace("|", "\\|") for value in values]
            operation_lines.append(f"| {' | '.join(escaped)} |")
        operation_lines.extend(["", "## 实际命令与原始输出", ""])
        for command in self.commands:
            operation_lines.extend(
                [
                    f"## OP {command.operation_id:03d}",
                    "",
                    f"- 命令：`{command.command}`",
                    f"- 返回码：`{command.returncode}`",
                    f"- 耗时：`{command.duration_s:.3f}s`",
                    f"- 超时：`{command.timed_out}`",
                    "",
                    "### STDOUT",
                    "",
                    "```text",
                    command.stdout,
                    "```",
                    "",
                    "### STDERR",
                    "",
                    "```text",
                    command.stderr,
                    "```",
                    "",
                ]
            )
        (self.artifact_dir / "operations.md").write_text(
            "\n".join(operation_lines), encoding="utf-8"
        )
        comparison_rows = []
        for result in self.results:
            if result.tc not in {"MAP-A-02", "LOC-A-02"}:
                continue
            for interface in result.interfaces:
                if interface.kind not in {
                    "input-topic",
                    "output-topic",
                    "service",
                }:
                    continue
                missing = interface.actual == "missing"
                comparison_rows.append(
                    {
                        "test_id": result.tc,
                        "category": interface.kind,
                        "expected_interface": interface.name,
                        "expected_type": interface.expected,
                        "actual_interface": (
                            "未发现" if missing else interface.name
                        ),
                        "actual_type": (
                            "未发现" if missing else interface.actual
                        ),
                        "status": PASS if interface.normal else FAIL,
                    }
                )
        comparison_overall = (
            "NOT_APPLICABLE"
            if not comparison_rows
            else (
                PASS
                if all(row["status"] == PASS for row in comparison_rows)
                else FAIL
            )
        )
        comparison_payload = {
            "schema_version": 1,
            "overall": comparison_overall,
            "rows": comparison_rows,
        }
        atomic_json(
            self.artifact_dir / "interface_comparison.json",
            comparison_payload,
        )
        comparison_lines = [
            "# 输入/输出/服务接口期望—实际对比",
            "",
            f"- 总体结论：**{comparison_overall}**",
            f"- 对比项数：{len(comparison_rows)}",
            "",
            "| 测试项 | 类别 | 期望接口 | 期望类型 | 实际接口 | 实际类型 | 是否通过 |",
            "|---|---|---|---|---|---|:---:|",
        ]
        for row in comparison_rows:
            escaped = [
                str(row[key]).replace("|", "\\|").replace("\n", " ")
                for key in (
                    "test_id",
                    "category",
                    "expected_interface",
                    "expected_type",
                    "actual_interface",
                    "actual_type",
                    "status",
                )
            ]
            comparison_lines.append(f"| {' | '.join(escaped)} |")
        comparison_lines.extend(
            [
                "",
                "> PASS 要求接口名称存在且实际 ROS 类型与基线完全一致。",
                "",
            ]
        )
        (self.artifact_dir / "interface_comparison.md").write_text(
            "\n".join(comparison_lines), encoding="utf-8"
        )

        lines = [
            "# SLAM 2D 接口自动化测试结果",
            "",
            f"- 执行模式：`{self.command}`",
            f"- 总体结论：**{overall}**",
            f"- 开始：{self.started_at}",
            f"- 结束：{payload['finished_at']}",
            (
                f"- 统计：PASS {counts[PASS]} / FAIL {counts[FAIL]} / "
                f"ERROR {counts[ERROR]} / PENDING_ENV {counts[PENDING_ENV]}"
            ),
            "",
            "| 测试项 ID | 测试项 | 脚本动作流程 | 状态 | 详情 |",
            "|---|---|---|:---:|---|",
        ]
        for result in self.results:
            detail = result.detail.replace("|", "\\|").replace("\n", " ")
            action_flow = (
                result.action_flow.replace("|", "\\|").replace("\n", " ")
            )
            lines.append(
                f"| {result.tc} | {result.name} | {action_flow} | "
                f"{result.status} | {detail} |"
            )
        lines.extend(["", "## 接口检查明细", ""])
        for result in self.results:
            lines.extend(
                [
                    f"### {result.tc} {result.name}",
                    "",
                    "| 状态 | 类型 | 接口/检查对象 | 期望 | 实际 |",
                    "|:---:|---|---|---|---|",
                ]
            )
            for interface in result.interfaces:
                values = (
                    "正常" if interface.normal else "异常",
                    interface.kind,
                    interface.name,
                    interface.expected,
                    interface.actual,
                )
                escaped = [
                    str(value).replace("|", "\\|").replace("\n", " ")
                    for value in values
                ]
                lines.append(f"| {' | '.join(escaped)} |")
            lines.append("")
        lines.extend(
            [
                "",
                "> `PENDING_ENV` 表示运行环境不完整，不能记为测试通过。",
                "",
            ]
        )
        (self.artifact_dir / "summary.md").write_text(
            "\n".join(lines), encoding="utf-8"
        )
        print(f"overall: {overall}")
        print(f"result: {self.artifact_dir}")
        return exit_code


@dataclass
class GraphSnapshot:
    nodes: set[str]
    services: set[str]
    topics: set[str]


@dataclass(frozen=True)
class TopicEndpoint:
    node: str
    endpoint_type: str
    topic_type: str


def lines(value: str) -> set[str]:
    return {
        line.strip()
        for line in value.splitlines()
        if line.strip().startswith("/")
    }


def graph_snapshot(
    reporter: Reporter, unavailable_id: str
) -> GraphSnapshot | None:
    if shutil.which("ros2") is None:
        reporter.add(
            unavailable_id,
            "ROS2 图可访问",
            PENDING_ENV,
            "找不到 ros2；请先 source ROS2 与待测工作空间",
            [
                interface_check(
                    "executable", "ros2", "available", "not found", False
                )
            ],
        )
        return None
    node_result = reporter.run(["ros2", "node", "list"], timeout_s=20)
    service_result = reporter.run(["ros2", "service", "list"], timeout_s=20)
    topic_result = reporter.run(["ros2", "topic", "list"], timeout_s=20)
    snapshot = GraphSnapshot(
        lines(node_result.stdout),
        lines(service_result.stdout),
        lines(topic_result.stdout),
    )
    atomic_json(
        reporter.artifact_dir / "ros_graph.json",
        {
            "nodes": sorted(snapshot.nodes),
            "services": sorted(snapshot.services),
            "topics": sorted(snapshot.topics),
        },
    )
    if not snapshot.nodes:
        reporter.add(
            unavailable_id,
            "ROS2 图可访问",
            PENDING_ENV,
            "ros2 node list 为空；对应业务栈尚未启动",
            [
                interface_check(
                    "ros-graph",
                    "ros2 node list",
                    "non-empty",
                    "empty",
                    False,
                )
            ],
        )
        return None
    return snapshot


def has_node(nodes: set[str], name: str) -> bool:
    return any(node == f"/{name}" or node.endswith(f"/{name}") for node in nodes)


def missing_nodes(nodes: set[str], required: Iterable[str]) -> list[str]:
    return [name for name in required if not has_node(nodes, name)]


def missing_names(existing: set[str], required: Iterable[str]) -> list[str]:
    return sorted(set(required) - existing)


def interface_check(
    kind: str,
    name: str,
    expected: Any,
    actual: Any,
    normal: bool,
) -> InterfaceCheck:
    return InterfaceCheck(
        kind=kind,
        name=name,
        expected=str(expected),
        actual=str(actual),
        normal=normal,
    )


MAPPING_INPUT_TOPICS = {
    "/laser_scan/front": "sensor_msgs/msg/LaserScan",
    "/laser_scan/rear": "sensor_msgs/msg/LaserScan",
    "/imu/chassis": "sensor_msgs/msg/Imu",
    "/feedback/odom": "nav_msgs/msg/Odometry",
}
MAPPING_OUTPUT_TOPICS = {
    "/mapping_node/state": "nav_interfaces/msg/MappingState",
    "/map": "nav_msgs/msg/OccupancyGrid",
}
MAPPING_SERVICES = {
    "/mapping_node/start_mapping": "nav_interfaces/srv/StartMapping",
    "/mapping_node/cancel_mapping": "std_srvs/srv/Trigger",
    "/mapping_node/finish_mapping": "std_srvs/srv/Trigger",
}
LOCALIZATION_INPUT_TOPICS = {
    "/laser_scan/front": "sensor_msgs/msg/LaserScan",
    "/laser_scan/rear": "sensor_msgs/msg/LaserScan",
    "/imu/chassis/refined": "sensor_msgs/msg/Imu",
    "/feedback/odom": "nav_msgs/msg/Odometry",
    "/map_manager/map_state": "nav_interfaces/msg/MapState",
    "/initialpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
}
LOCALIZATION_OUTPUT_TOPICS = {
    "/localization_node/loc_pose": (
        "geometry_msgs/msg/PoseWithCovarianceStamped"
    ),
    "/localization_node/state": "nav_interfaces/msg/LocalizationState",
    "/localization_node/tracked_path": "nav_msgs/msg/Path",
    "/robot_localization/odometry/filtered": "nav_msgs/msg/Odometry",
    "/robot_localization/set_pose": (
        "geometry_msgs/msg/PoseWithCovarianceStamped"
    ),
    "/map": "nav_msgs/msg/OccupancyGrid",
}
LOCALIZATION_SERVICES = {
    "/map_manager/load_map": "nav_interfaces/srv/LoadMap",
    "/map_manager/get_current_map_info": "nav_interfaces/srv/GetMapInfo",
    "/map_manager/get_map_list": "nav_interfaces/srv/GetMapList",
    "/localization_node/start_localization": "std_srvs/srv/Trigger",
    "/localization_node/stop_localization": "std_srvs/srv/Trigger",
    "/localization_node/reset_localization": "std_srvs/srv/Trigger",
    "/localization_node/save_map": "nav_interfaces/srv/SaveMap",
    "/localization_node/reset_map_origin": "std_srvs/srv/Trigger",
    "/localization_node/relocalization": "nav_interfaces/srv/SetPose",
    "/localization_node/trigger_global_relocalization": (
        "std_srvs/srv/Trigger"
    ),
    "/map_asset_manager/save_assets": "nav_interfaces/srv/SaveAssets",
    "/map_asset_manager/reload_data": "std_srvs/srv/Trigger",
    "/map_asset_manager/transform_assets": (
        "nav_interfaces/srv/TransformAssets"
    ),
    "/robot_localization/set_pose": "robot_localization/srv/SetPose",
    "/robot_localization/start_filter": "std_srvs/srv/Trigger",
    "/robot_localization/stop_filter": "std_srvs/srv/Trigger",
}


def queried_interface_type(
    reporter: Reporter, kind: str, name: str
) -> str:
    result = reporter.run(
        ["ros2", kind, "type", name],
        timeout_s=10.0,
    )
    direct_type = " | ".join(
        line.strip() for line in result.stdout.splitlines() if line.strip()
    )
    if result.returncode == 0 and direct_type:
        return direct_type

    # `ros2 service type` has no --no-daemon option in Humble and can race a
    # stale daemon even when the service name is already visible.  Rebuild a
    # fresh graph and read the exact name/type pair before declaring failure.
    fallback_command = [
        "ros2",
        kind,
        "list",
        "-t",
        "--no-daemon",
        "--spin-time",
        "3.0",
    ]
    fallback = reporter.run(fallback_command, timeout_s=10.0)
    if fallback.returncode == 0:
        for line in fallback.stdout.splitlines():
            match = re.match(r"^\s*(/\S+)\s+\[([^\]]+)\]\s*$", line)
            if match and match.group(1) == name:
                return " | ".join(
                    value.strip()
                    for value in match.group(2).split(",")
                    if value.strip()
                )

    direct_error = (
        result.stderr.strip()
        or result.stdout.strip()
        or f"exit {result.returncode}"
    )
    fallback_error = (
        fallback.stderr.strip()
        or fallback.stdout.strip()
        or f"exit {fallback.returncode}"
    )
    return (
        f"type query failed: {direct_error}; "
        f"fresh {kind} list -t unresolved: {fallback_error}"
    )


def contract_interface_checks(
    reporter: Reporter,
    graph: GraphSnapshot,
    kind: str,
    expected: dict[str, str],
) -> list[InterfaceCheck]:
    existing = graph.services if kind == "service" else graph.topics
    query_kind = "service" if kind == "service" else "topic"
    checks = []
    for name, expected_type in sorted(expected.items()):
        if name not in existing:
            actual_type = "missing"
        else:
            actual_type = queried_interface_type(
                reporter, query_kind, name
            )
        checks.append(
            interface_check(
                kind,
                name,
                expected_type,
                actual_type,
                actual_type == expected_type,
            )
        )
    return checks


def presence_checks(
    kind: str,
    existing: set[str],
    names: Iterable[str],
    *,
    expected_present: bool,
) -> list[InterfaceCheck]:
    checks = []
    for name in sorted(names):
        present = (
            has_node(existing, name.lstrip("/"))
            if kind == "node"
            else name in existing
        )
        checks.append(
            interface_check(
                kind,
                name if name.startswith("/") else f"/{name}",
                "present" if expected_present else "absent",
                "present" if present else "absent",
                present == expected_present,
            )
        )
    return checks


def parse_scalar(text: str, field: str) -> str:
    patterns = (
        rf"(?m)^\s*{re.escape(field)}:\s*['\"]?([^'\"\n]+)",
        rf"\b{re.escape(field)}=(?:'([^']*)'|\"([^\"]*)\"|([^,\)\s]+))",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return next(
                (group.strip() for group in match.groups() if group is not None),
                "",
            )
    return ""


def parse_topic_endpoints(text: str) -> list[TopicEndpoint]:
    endpoints: list[TopicEndpoint] = []
    blocks = re.split(r"(?=^\s*Node name:\s*)", text, flags=re.MULTILINE)
    for block in blocks:
        node_name = parse_scalar(block, "Node name")
        endpoint_type = parse_scalar(block, "Endpoint type").upper()
        topic_type = parse_scalar(block, "Topic type")
        if not node_name or endpoint_type not in {"PUBLISHER", "SUBSCRIPTION"}:
            continue
        namespace = parse_scalar(block, "Node namespace") or "/"
        node = f"{namespace.rstrip('/')}/{node_name.lstrip('/')}"
        if not node.startswith("/"):
            node = f"/{node}"
        endpoints.append(
            TopicEndpoint(
                node=node,
                endpoint_type=endpoint_type,
                topic_type=topic_type,
            )
        )
    return endpoints


_NODE_INFO_SECTIONS = {
    "Subscribers",
    "Subscriptions",
    "Publishers",
    "Service Servers",
    "Service Clients",
    "Action Servers",
    "Action Clients",
}


def parse_node_info_topics(text: str, sections: set[str]) -> dict[str, str]:
    """Parse topic/type pairs from selected `ros2 node info` sections."""
    active = False
    topics: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        heading = stripped[:-1] if stripped.endswith(":") else stripped
        if heading in _NODE_INFO_SECTIONS:
            active = heading in sections
            continue
        if not active:
            continue
        match = re.match(r"^\s*(/[^:]+):\s*(\S+)\s*$", line)
        if match:
            topics[match.group(1)] = match.group(2)
    return topics


def check_set_pose_endpoints(
    reporter: Reporter,
) -> tuple[list[str], list[InterfaceCheck]]:
    topic_result = reporter.run(
        [
            "ros2",
            "topic",
            "info",
            "/robot_localization/set_pose",
            "--verbose",
            "--no-daemon",
            "--spin-time",
            "3.0",
        ],
        timeout_s=10,
    )
    localization_info = reporter.run(
        [
            "ros2",
            "node",
            "info",
            "/localization_node",
            "--no-daemon",
            "--spin-time",
            "3.0",
        ],
        timeout_s=10,
    )
    filter_info = reporter.run(
        [
            "ros2",
            "node",
            "info",
            "/robot_localization",
            "--no-daemon",
            "--spin-time",
            "3.0",
        ],
        timeout_s=10,
    )

    expected_type = "geometry_msgs/msg/PoseWithCovarianceStamped"
    endpoints = (
        parse_topic_endpoints(topic_result.stdout)
        if topic_result.returncode == 0
        else []
    )
    all_publisher_endpoints = [
        endpoint
        for endpoint in endpoints
        if endpoint.endpoint_type == "PUBLISHER"
    ]
    all_subscription_endpoints = [
        endpoint
        for endpoint in endpoints
        if endpoint.endpoint_type == "SUBSCRIPTION"
    ]
    publisher_endpoints = [
        endpoint
        for endpoint in all_publisher_endpoints
        if endpoint.topic_type == expected_type
    ]
    subscription_endpoints = [
        endpoint
        for endpoint in all_subscription_endpoints
        if endpoint.topic_type == expected_type
    ]
    publishers = {endpoint.node for endpoint in publisher_endpoints}
    subscriptions = {endpoint.node for endpoint in subscription_endpoints}
    node_publishers = parse_node_info_topics(
        localization_info.stdout, {"Publishers"}
    )
    node_subscriptions = parse_node_info_topics(
        filter_info.stdout, {"Subscribers", "Subscriptions"}
    )
    missing: list[str] = []
    unknown_node = "/_NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_"
    publisher_conflict = any(
        endpoint.topic_type != expected_type
        or endpoint.node
        not in {"/localization_node", unknown_node}
        for endpoint in all_publisher_endpoints
    ) or len(all_publisher_endpoints) > 1
    subscriber_conflict = any(
        endpoint.topic_type != expected_type
        or endpoint.node
        not in {"/robot_localization", unknown_node}
        for endpoint in all_subscription_endpoints
    ) or len(all_subscription_endpoints) > 1
    publisher_owner_ok = (
        has_node(publishers, "localization_node")
        or node_publishers.get("/robot_localization/set_pose") == expected_type
    ) and not publisher_conflict
    subscriber_owner_ok = (
        has_node(subscriptions, "robot_localization")
        or node_subscriptions.get("/robot_localization/set_pose") == expected_type
    ) and not subscriber_conflict
    anonymous_publisher_ok = bool(
        not publisher_owner_ok
        and localization_info.returncode == 0
        and len(all_publisher_endpoints) == 1
        and all_publisher_endpoints[0].node == unknown_node
        and all_publisher_endpoints[0].topic_type == expected_type
    )
    anonymous_subscriber_ok = bool(
        not subscriber_owner_ok
        and filter_info.returncode == 0
        and len(all_subscription_endpoints) == 1
        and all_subscription_endpoints[0].node == unknown_node
        and all_subscription_endpoints[0].topic_type == expected_type
    )
    publisher_ok = publisher_owner_ok or anonymous_publisher_ok
    subscriber_ok = subscriber_owner_ok or anonymous_subscriber_ok
    publisher_actual = (
        expected_type
        if publisher_owner_ok
        else (
            f"{expected_type}; owner=RMW_UNKNOWN; "
            "evidence=single correct-type publisher + /localization_node present"
        )
        if anonymous_publisher_ok
        else (
            f"topic_info_publishers={sorted(publishers)}; "
            f"node_info_type={node_publishers.get('/robot_localization/set_pose', 'missing')}"
        )
    )
    subscriber_actual = (
        expected_type
        if subscriber_owner_ok
        else (
            f"{expected_type}; owner=RMW_UNKNOWN; "
            "evidence=single correct-type subscription + /robot_localization present"
        )
        if anonymous_subscriber_ok
        else (
            f"topic_info_subscriptions={sorted(subscriptions)}; "
            f"node_info_type={node_subscriptions.get('/robot_localization/set_pose', 'missing')}"
        )
    )
    if not publisher_ok:
        missing.append(
            "缺 localization_node 的 PoseWithCovarianceStamped publisher；"
            "若源码已包含该 publisher，请重新编译、source install 并重启定位栈"
        )
    if not subscriber_ok:
        missing.append(
            "缺 robot_localization 的 PoseWithCovarianceStamped subscription"
        )
    checks = [
        interface_check(
            "topic-publisher",
            "/robot_localization/set_pose <- /localization_node",
            expected_type,
            publisher_actual,
            publisher_ok,
        ),
        interface_check(
            "topic-subscription",
            "/robot_localization/set_pose -> /robot_localization",
            expected_type,
            subscriber_actual,
            subscriber_ok,
        ),
    ]
    return missing, checks


class LocalizationStateCapture:
    """Keep a state subscription alive across a service call to catch short state 3."""

    def __init__(self, reporter: Reporter) -> None:
        self.reporter = reporter
        self.command = [
            "ros2",
            "topic",
            "echo",
            "/localization_node/state",
            "nav_interfaces/msg/LocalizationState",
        ]
        self.process: subprocess.Popen[str] | None = None
        self.started = 0.0
        self.stdout_lines: list[str] = []
        self.stderr_lines: list[str] = []
        self.samples: list[dict[str, Any]] = []
        self.condition = threading.Condition()
        self.threads: list[threading.Thread] = []
        self.start_error = ""
        self.operation_id = 0

    def _append_sample(self, block: list[str]) -> None:
        text = "".join(block)
        status_text = parse_scalar(text, "status")
        score_text = parse_scalar(text, "score")
        try:
            status = int(status_text)
            score = float(score_text)
        except ValueError:
            return
        with self.condition:
            self.samples.append(
                {
                    "captured_at": time.monotonic(),
                    "status": status,
                    "score": score,
                }
            )
            self.condition.notify_all()

    def _drain_stdout(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        block: list[str] = []
        for line in self.process.stdout:
            self.stdout_lines.append(line)
            if line.strip() == "---":
                self._append_sample(block)
                block = []
            else:
                block.append(line)
        if block:
            self._append_sample(block)
        with self.condition:
            self.condition.notify_all()

    def _drain_stderr(self) -> None:
        assert self.process is not None and self.process.stderr is not None
        for line in self.process.stderr:
            self.stderr_lines.append(line)

    def start(self) -> bool:
        self.operation_id = self.reporter.operation_start(
            self.command,
            "持续订阅 LocalizationState；先确认基线，再跨服务调用捕获短暂状态 3",
        )
        self.started = time.monotonic()
        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env={
                    **os.environ,
                    "PYTHONDONTWRITEBYTECODE": "1",
                    "PYTHONUNBUFFERED": "1",
                },
            )
        except OSError as exc:
            self.start_error = str(exc)
            return False
        self.threads = [
            threading.Thread(target=self._drain_stdout, daemon=True),
            threading.Thread(target=self._drain_stderr, daemon=True),
        ]
        for thread in self.threads:
            thread.start()
        return True

    def wait_for_samples(self, minimum: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        with self.condition:
            while len(self.samples) < minimum:
                if self.process is not None and self.process.poll() is not None:
                    return False
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self.condition.wait(timeout=min(0.1, remaining))
            return True

    def snapshot(self) -> list[dict[str, Any]]:
        with self.condition:
            return [dict(sample) for sample in self.samples]

    def close(self) -> None:
        if self.process is None:
            self.reporter.operation_finish(
                CommandRecord(
                    command=shlex.join(self.command),
                    returncode=127,
                    duration_s=round(time.monotonic() - self.started, 3),
                    stdout="",
                    stderr=self.start_error,
                    operation_id=self.operation_id,
                )
            )
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        for thread in self.threads:
            thread.join(timeout=1.0)
        raw_returncode = self.process.returncode or 0
        self.reporter.operation_finish(
            CommandRecord(
                command=shlex.join(self.command),
                returncode=0 if raw_returncode in {0, -15} else raw_returncode,
                duration_s=round(time.monotonic() - self.started, 3),
                stdout="".join(self.stdout_lines).strip(),
                stderr="".join(self.stderr_lines).strip(),
                operation_id=self.operation_id,
            )
        )


def bool_success(text: str) -> bool:
    return bool(
        re.search(r"\bsuccess\s*(?:=|:)\s*(?:True|true)\b", text)
    )


def resolve_localization_log_file(log_path: Path) -> Path:
    """Accept either a syslog directory or an explicit log file."""
    return log_path / "syslog" if log_path.is_dir() else log_path


def _read_log_bytes(path: Path) -> bytes:
    if path.suffix == ".gz":
        with gzip.open(path, "rb") as stream:
            return stream.read()
    return path.read_bytes()


def _syslog_family(active_path: Path) -> list[Path]:
    try:
        return sorted(
            (
                path
                for path in active_path.parent.glob(f"{active_path.name}*")
                if path.is_file()
            ),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
    except OSError:
        return []


SYSLOG_LINE_RE = re.compile(
    r"^(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec)\s+"
    r"\d{1,2}\s+\d{2}:\d{2}:\d{2}\s+\S+\s+"
    r"(?P<tag>[^\s\[]+)(?:\[\d+\])?:\s"
)


def filter_syslog_tag(text: str, log_tag: str) -> str:
    """Keep one logger(1) tag in a shared syslog; retain legacy plain logs."""
    if not log_tag:
        return text
    selected: list[str] = []
    syslog_lines = 0
    include_continuation = False
    for line in text.splitlines(keepends=True):
        match = SYSLOG_LINE_RE.match(line)
        if match:
            syslog_lines += 1
            include_continuation = match.group("tag") == log_tag
        if include_continuation:
            selected.append(line)
    return "".join(selected) if syslog_lines else text


def capture_log_cursor(
    reporter: Reporter,
    scope: str,
    log_path: Path,
    log_tag: str = DEFAULT_LOCALIZATION_LOG_TAG,
) -> dict[str, Any]:
    """Record an exact byte boundary so one trial cannot reuse old log lines."""
    active_path = resolve_localization_log_file(log_path)
    try:
        stat = active_path.stat()
        anchor_size = min(4096, stat.st_size)
        with active_path.open("rb") as stream:
            stream.seek(stat.st_size - anchor_size)
            anchor = stream.read(anchor_size)
        cursor = {
            "available": True,
            "device": stat.st_dev,
            "inode": stat.st_ino,
            "offset": stat.st_size,
            "active_path": str(active_path),
            "anchor": anchor,
            "captured_at_ns": time.time_ns(),
            "error": "",
        }
        reporter.script_step(
            scope,
            "记录本轮定位日志起点",
            (
                f"source={log_path} active_file={active_path} tag={log_tag} "
                f"inode={stat.st_ino} byte_offset={stat.st_size}；"
                "后续只读取该边界之后且属于定位 tag 的新增日志"
            ),
        )
        return cursor
    except OSError as exc:
        reporter.script_step(
            scope,
            "定位日志不可读，降级为状态闭环",
            f"source={log_path} active_file={active_path} error={exc!r}",
        )
        return {
            "available": False,
            "device": None,
            "inode": None,
            "offset": 0,
            "active_path": str(active_path),
            "anchor": b"",
            "captured_at_ns": time.time_ns(),
            "error": repr(exc),
        }


def read_log_since_cursor(
    log_path: Path,
    cursor: dict[str, Any],
    log_tag: str = DEFAULT_LOCALIZATION_LOG_TAG,
) -> tuple[str, str]:
    if not cursor["available"]:
        return "", str(cursor["error"])
    active_path = resolve_localization_log_file(log_path)
    try:
        stat = active_path.stat()
        same_file = (
            stat.st_dev == cursor["device"]
            and stat.st_ino == cursor["inode"]
            and stat.st_size >= int(cursor["offset"])
        )
        if same_file and cursor.get("anchor"):
            anchor = bytes(cursor["anchor"])
            with active_path.open("rb") as stream:
                stream.seek(int(cursor["offset"]) - len(anchor))
                same_file = stream.read(len(anchor)) == anchor
        if same_file:
            with active_path.open("rb") as stream:
                stream.seek(int(cursor["offset"]))
                raw = stream.read().decode("utf-8", errors="replace")
            return filter_syslog_tag(raw, log_tag), ""

        rotation_prefix = cursor.get("rotation_prefix")
        rotation_error = str(cursor.get("rotation_error", ""))
        if rotation_prefix is None:
            old_tail = b""
            old_source = ""
            candidates = [
                path for path in _syslog_family(active_path) if path != active_path
            ]
            for candidate in candidates:
                try:
                    candidate_stat = candidate.stat()
                    if (
                        candidate_stat.st_dev == cursor["device"]
                        and candidate_stat.st_ino == cursor["inode"]
                    ):
                        candidate_bytes = _read_log_bytes(candidate)
                        old_tail = candidate_bytes[int(cursor["offset"]):]
                        old_source = str(candidate)
                        break
                except (OSError, EOFError):
                    continue
            if not old_source and cursor.get("anchor"):
                anchor = bytes(cursor["anchor"])
                for candidate in candidates[:4]:
                    try:
                        candidate_bytes = _read_log_bytes(candidate)
                    except (OSError, EOFError):
                        continue
                    anchor_at = candidate_bytes.rfind(anchor)
                    if anchor_at >= 0:
                        old_tail = candidate_bytes[anchor_at + len(anchor):]
                        old_source = str(candidate)
                        break
            if old_source:
                rotation_prefix = old_tail.decode(
                    "utf-8", errors="replace"
                )
                rotation_error = ""
            else:
                rotation_prefix = ""
                rotation_error = (
                    "syslog rotated after the trial boundary but the previous "
                    "file tail could not be recovered; read the new active file"
                )
            cursor["rotation_prefix"] = rotation_prefix
            cursor["rotation_error"] = rotation_error

        current_text = active_path.read_bytes().decode(
            "utf-8", errors="replace"
        )
        combined = f"{rotation_prefix}{current_text}"
        return filter_syslog_tag(combined, log_tag), rotation_error
    except (OSError, EOFError) as exc:
        return "", repr(exc)


def last_regex_group(pattern: str, text: str) -> str:
    matches = re.findall(pattern, text, flags=re.IGNORECASE)
    return str(matches[-1]) if matches else ""


def parse_relocalization_log(text: str) -> dict[str, Any]:
    float_pattern = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
    total_text = last_regex_group(
        rf"Global relocalization took\s+({float_pattern})\s*ms", text
    )
    pose_total_text = last_regex_group(
        rf"Pose relocalization took\s+({float_pattern})\s*ms", text
    )
    pose_starts = [
        (match.start(), float(match.group(1)))
        for match in re.finditer(
            rf"\[({float_pattern})\][^\r\n]*"
            r"PoseInitializer relocalization thread started",
            text,
        )
    ]
    pose_successes = [
        (match.start(), float(match.group(1)))
        for match in re.finditer(
            rf"\[({float_pattern})\][^\r\n]*Relocalization succeeded!",
            text,
        )
    ]
    pose_total_ms: float | None = None
    pose_started_at_ros_s: float | None = None
    pose_succeeded_at_ros_s: float | None = None
    for success_position, success_timestamp in reversed(pose_successes):
        matching_starts = [
            (position, timestamp)
            for position, timestamp in pose_starts
            if position < success_position and timestamp <= success_timestamp
        ]
        if matching_starts:
            _, start_timestamp = matching_starts[-1]
            pose_started_at_ros_s = start_timestamp
            pose_succeeded_at_ros_s = success_timestamp
            pose_total_ms = round(
                (success_timestamp - start_timestamp) * 1000.0, 3
            )
            break
    global_total_ms = float(total_text) if total_text else None
    direct_pose_total_ms = float(pose_total_text) if pose_total_text else None
    total_ms = (
        global_total_ms
        if global_total_ms is not None
        else (
            direct_pose_total_ms
            if direct_pose_total_ms is not None
            else pose_total_ms
        )
    )
    timing_source = (
        "Global relocalization took"
        if global_total_ms is not None
        else (
            "Pose relocalization took"
            if direct_pose_total_ms is not None
            else (
                "PoseInitializer start->Relocalization succeeded"
                if pose_total_ms is not None
                else "unavailable"
            )
        )
    )
    final_score_text = last_regex_group(
        rf"Relocalization validation passed:\s*final_score=({float_pattern})",
        text,
    )
    if not final_score_text:
        final_score_text = last_regex_group(
            rf"Global localization score:\s*({float_pattern})", text
        )
    info_lines = re.findall(r"Global relocalization info:[^\r\n]*", text)
    info_line = info_lines[-1] if info_lines else ""
    timing_fields: dict[str, float | int] = {}
    for key, value in re.findall(
        rf"\b(call_index|t_[a-z_]+_ms)=({float_pattern})", info_line
    ):
        timing_fields[key] = int(float(value)) if key == "call_index" else float(value)
    failure_patterns = (
        r"Global relocalization failed[^\r\n]*",
        r"Global relocalization validation failed[^\r\n]*",
        r"Validation gate[123] failed[^\r\n]*",
    )
    failure_lines = [
        match
        for pattern in failure_patterns
        for match in re.findall(pattern, text)
    ]
    return {
        "complete": total_ms is not None,
        "success": (
            "Global relocalization succeeded" in text
            or (
                direct_pose_total_ms is not None
                and "Relocalization validation passed" in text
            )
            or pose_total_ms is not None
        ),
        "total_ms": total_ms,
        "timing_source": timing_source,
        "pose_started_at_ros_s": pose_started_at_ros_s,
        "pose_succeeded_at_ros_s": pose_succeeded_at_ros_s,
        "final_score": float(final_score_text) if final_score_text else None,
        "timing_fields": timing_fields,
        "failure": failure_lines[-1] if failure_lines else "",
    }


def read_topic_once(
    reporter: Reporter,
    topic: str,
    message_type: str,
    *,
    timeout_s: float = 10.0,
    transient_local: bool = False,
) -> CommandRecord:
    command = ["ros2", "topic", "echo", topic, message_type]
    if transient_local:
        command.extend(
            [
                "--qos-durability",
                "transient_local",
                "--qos-reliability",
                "reliable",
            ]
        )
    command.append("--once")
    return reporter.run(command, timeout_s=timeout_s)


def read_map_state(reporter: Reporter, timeout_s: float = 10.0) -> dict[str, Any] | None:
    result = read_topic_once(
        reporter,
        "/map_manager/map_state",
        "nav_interfaces/msg/MapState",
        timeout_s=timeout_s,
        transient_local=True,
    )
    text = f"{result.stdout}\n{result.stderr}"
    try:
        epoch = int(parse_scalar(text, "epoch"))
    except ValueError:
        return None
    return {
        "map_folder": parse_scalar(text, "map_folder"),
        "map_name": parse_scalar(text, "map_name"),
        "epoch": epoch,
        "raw": result.stdout,
    }


def read_localization_state(
    reporter: Reporter, timeout_s: float = 6.0
) -> dict[str, Any] | None:
    result = read_topic_once(
        reporter,
        "/localization_node/state",
        "nav_interfaces/msg/LocalizationState",
        timeout_s=timeout_s,
    )
    text = f"{result.stdout}\n{result.stderr}"
    try:
        status = int(parse_scalar(text, "status"))
        score = float(parse_scalar(text, "score"))
    except ValueError:
        return None
    return {
        "status": status,
        "score": score,
        "raw": result.stdout,
    }


def read_localization_status(
    reporter: Reporter, timeout_s: float = 6.0
) -> int | None:
    state = read_localization_state(reporter, timeout_s=timeout_s)
    return state["status"] if state else None


def read_localization_pose(
    reporter: Reporter, timeout_s: float = 6.0
) -> dict[str, float | str] | None:
    result = read_topic_once(
        reporter,
        "/localization_node/loc_pose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        timeout_s=timeout_s,
    )
    text = f"{result.stdout}\n{result.stderr}"
    position = re.search(
        r"(?ms)^\s*position:\s*\n(?P<body>.*?)(?=^\s*orientation:\s*$)",
        text,
    )
    orientation = re.search(
        r"(?ms)^\s*orientation:\s*\n(?P<body>.*?)(?=^\s*covariance:\s*)",
        text,
    )
    if not position or not orientation:
        return None
    try:
        x = float(parse_scalar(position.group("body"), "x"))
        y = float(parse_scalar(position.group("body"), "y"))
        z = float(parse_scalar(position.group("body"), "z"))
        qx = float(parse_scalar(orientation.group("body"), "x"))
        qy = float(parse_scalar(orientation.group("body"), "y"))
        qz = float(parse_scalar(orientation.group("body"), "z"))
        qw = float(parse_scalar(orientation.group("body"), "w"))
    except ValueError:
        return None
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return {
        "frame_id": parse_scalar(text, "frame_id"),
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
    }


def read_mapping_state(
    reporter: Reporter, timeout_s: float = 6.0
) -> tuple[int, str] | None:
    result = read_topic_once(
        reporter,
        "/mapping_node/state",
        "nav_interfaces/msg/MappingState",
        timeout_s=timeout_s,
    )
    text = f"{result.stdout}\n{result.stderr}"
    try:
        return int(parse_scalar(text, "status")), parse_scalar(text, "map_name")
    except ValueError:
        return None


def wait_map_state(
    reporter: Reporter,
    *,
    min_epoch: int,
    map_name: str,
    timeout_s: float,
) -> dict[str, Any] | None:
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        latest = read_map_state(reporter, timeout_s=min(5.0, timeout_s))
        if (
            latest
            and latest["epoch"] >= min_epoch
            and latest["map_name"] == map_name
        ):
            return latest
        time.sleep(0.5)
    return latest


def wait_localization(
    reporter: Reporter,
    *,
    timeout_s: float,
    require_relocalizing: bool = False,
) -> tuple[bool, list[int]]:
    deadline = time.monotonic() + timeout_s
    statuses: list[int] = []
    while time.monotonic() < deadline:
        status = read_localization_status(reporter, timeout_s=5.0)
        if status is not None:
            statuses.append(status)
            if status in (-2, -1):
                return False, statuses
            if status == 4 and (
                not require_relocalizing or 3 in statuses
            ):
                return True, statuses
        time.sleep(0.5)
    return False, statuses


def wait_mapping_status(
    reporter: Reporter, wanted: int, timeout_s: float
) -> tuple[bool, list[int]]:
    deadline = time.monotonic() + timeout_s
    statuses: list[int] = []
    while time.monotonic() < deadline:
        state = read_mapping_state(reporter, timeout_s=5.0)
        if state is not None:
            statuses.append(state[0])
            if state[0] == wanted:
                return True, statuses
            if state[0] == -1:
                return False, statuses
        time.sleep(0.5)
    return False, statuses


def check_legacy_interfaces(
    reporter: Reporter, graph: GraphSnapshot, test_id: str
) -> None:
    forbidden_services = {
        "/localization_node/set_map_changed",
        "/localization_node/rollback_map_to",
    }
    forbidden_topics = {
        "/map_manager/map_folder",
        "/map_manager/map_name",
        "/localization_state",
    }
    forbidden_nodes = {"cartographer_offline_node", "loc_offline_node"}
    present = sorted(forbidden_services & graph.services)
    topic_checks: list[InterfaceCheck] = []
    for topic in sorted(forbidden_topics):
        if topic not in graph.topics:
            topic_checks.append(
                interface_check(
                    "topic-publisher", topic, "no publisher", "no graph endpoint", True
                )
            )
            continue

        result = reporter.run(
            [
                "ros2",
                "topic",
                "info",
                topic,
                "--verbose",
                "--no-daemon",
                "--spin-time",
                "3.0",
            ],
            timeout_s=10.0,
        )
        endpoints = (
            parse_topic_endpoints(result.stdout)
            if result.returncode == 0
            else []
        )
        publishers = [
            endpoint
            for endpoint in endpoints
            if endpoint.endpoint_type == "PUBLISHER"
        ]
        subscribers = [
            endpoint.node
            for endpoint in endpoints
            if endpoint.endpoint_type == "SUBSCRIPTION"
        ]
        if result.returncode != 0:
            normal = False
            actual = (
                "publisher discovery failed: "
                + (
                    result.stderr.strip()
                    or result.stdout.strip()
                    or f"exit {result.returncode}"
                )
            )
        elif publishers:
            normal = False
            actual = "present; publishers=" + str(
                sorted(
                    f"{endpoint.node}:{endpoint.topic_type}"
                    for endpoint in publishers
                )
            )
        else:
            normal = True
            actual = (
                "subscriber-only; no publisher; subscribers="
                + str(sorted(subscribers))
            )
        if not normal:
            present.append(topic)
        topic_checks.append(
            interface_check(
                "topic-publisher", topic, "no publisher", actual, normal
            )
        )
    present.extend(
        name for name in forbidden_nodes if has_node(graph.nodes, name)
    )
    checks = presence_checks(
        "service", graph.services, forbidden_services, expected_present=False
    )
    checks.extend(topic_checks)
    checks.extend(
        presence_checks(
            "node", graph.nodes, forbidden_nodes, expected_present=False
        )
    )
    reporter.add(
        test_id,
        "旧接口与旧节点域无残留",
        PASS if not present else FAIL,
        "无残留" if not present else f"仍存在：{', '.join(present)}",
        checks,
    )


def check_mapping_contract(
    reporter: Reporter, graph: GraphSnapshot
) -> None:
    nodes_missing = missing_nodes(graph.nodes, ("mapping_node",))
    checks = presence_checks(
        "node", graph.nodes, ("mapping_node",), expected_present=True
    )
    checks.extend(
        contract_interface_checks(
            reporter, graph, "input-topic", MAPPING_INPUT_TOPICS
        )
    )
    checks.extend(
        contract_interface_checks(
            reporter, graph, "output-topic", MAPPING_OUTPUT_TOPICS
        )
    )
    checks.extend(
        contract_interface_checks(
            reporter, graph, "service", MAPPING_SERVICES
        )
    )
    detail_parts = [
        f"{check.kind} {check.name} 期望 {check.expected}，实际 {check.actual}"
        for check in checks
        if not check.normal
    ]
    if nodes_missing:
        detail_parts.append(f"缺节点 {nodes_missing}")
    reporter.add(
        "MAP-A-02",
        "建图输入/输出/服务接口契约",
        PASS if not detail_parts else FAIL,
        (
            f"{len(MAPPING_INPUT_TOPICS)} 个输入 Topic、"
            f"{len(MAPPING_OUTPUT_TOPICS)} 个输出 Topic、"
            f"{len(MAPPING_SERVICES)} 个 Service 名称与类型一致"
        )
        if not detail_parts
        else "；".join(detail_parts),
        checks,
    )


def check_localization_contract(
    reporter: Reporter, graph: GraphSnapshot
) -> None:
    required_nodes = (
        "map_manager",
        "localization_node",
        "map_asset_manager",
        "robot_localization",
    )
    detail_parts = []
    nodes_missing = missing_nodes(graph.nodes, required_nodes)
    checks = presence_checks(
        "node", graph.nodes, required_nodes, expected_present=True
    )
    checks.extend(
        contract_interface_checks(
            reporter, graph, "input-topic", LOCALIZATION_INPUT_TOPICS
        )
    )
    checks.extend(
        contract_interface_checks(
            reporter, graph, "output-topic", LOCALIZATION_OUTPUT_TOPICS
        )
    )
    checks.extend(
        contract_interface_checks(
            reporter, graph, "service", LOCALIZATION_SERVICES
        )
    )
    if nodes_missing:
        detail_parts.append(f"缺节点 {nodes_missing}")
    detail_parts.extend(
        f"{check.kind} {check.name} 期望 {check.expected}，实际 {check.actual}"
        for check in checks
        if not check.normal
    )
    tracked_path_missing = (
        "/localization_node/loc_pose" in graph.topics
        and "/localization_node/tracked_path" not in graph.topics
    )
    if tracked_path_missing:
        deployment_hint = (
            "loc_pose 已存在但 tracked_path 缺失；当前源码在同一个 "
            "publish_tracked_pose 分支中同时创建两个 publisher，"
            "该组合证明运行的 localization_node 二进制与当前源码不一致；"
            "请重建 cartographer_ros、最后 source 对应 install 并重启定位栈"
        )
        reporter.script_step(
            "LOC-A-02",
            "判定 localization_node 部署漂移",
            deployment_hint,
        )
        detail_parts.append(deployment_hint)
    if "/robot_localization/set_pose" in graph.topics:
        endpoint_errors, endpoint_checks = check_set_pose_endpoints(reporter)
        detail_parts.extend(endpoint_errors)
        checks.extend(endpoint_checks)
    reporter.add(
        "LOC-A-02",
        "定位输入/输出/服务接口契约",
        PASS if not detail_parts else FAIL,
        (
            f"4 节点、{len(LOCALIZATION_INPUT_TOPICS)} 个输入 Topic、"
            f"{len(LOCALIZATION_OUTPUT_TOPICS)} 个输出 Topic、"
            f"{len(LOCALIZATION_SERVICES)} 个 Service 名称与类型一致；"
            "set_pose 发布/订阅端点正确"
        )
        if not detail_parts
        else "；".join(detail_parts),
        checks,
    )


def check_map_state(reporter: Reporter) -> dict[str, Any] | None:
    state = read_map_state(reporter)
    passed = bool(
        state
        and state["map_folder"]
        and state["map_name"]
        and state["epoch"] >= 1
    )
    reporter.add(
        "LOC-A-04",
        "MapState transient-local 原子状态",
        PASS if passed else FAIL,
        (
            f"folder={state['map_folder']} name={state['map_name']} "
            f"epoch={state['epoch']}"
            if state
            else "10 秒内未读到可解析的 MapState"
        ),
        [
            interface_check(
                "topic",
                "/map_manager/map_state",
                "MapState(folder/name non-empty, epoch>=1, transient-local)",
                (
                    f"folder={state['map_folder']}, name={state['map_name']}, "
                    f"epoch={state['epoch']}"
                    if state
                    else "unreadable"
                ),
                passed,
            )
        ],
    )
    return state


def check_localization_state(reporter: Reporter) -> None:
    state = read_localization_state(reporter, timeout_s=10)
    status = state["status"] if state else None
    score = state["score"] if state else None
    score_ok = (
        isinstance(score, float)
        and math.isfinite(score)
        and 0.0 <= score <= 1.0
    )
    passed = status in {-2, -1, 0, 1, 2, 3, 4} and score_ok
    reporter.add(
        "LOC-A-05",
        "LocalizationState 状态与得分可读",
        PASS if passed else FAIL,
        f"当前 status={status} score={score}"
        if passed
        else (
            f"状态或得分非法：status={status} score={score}"
            if state is not None
            else "10 秒内未收到 /localization_node/state"
        ),
        [
            interface_check(
                "topic",
                "/localization_node/state",
                "LocalizationState.status in {-2,-1,0,1,2,3,4}",
                status,
                status in {-2, -1, 0, 1, 2, 3, 4},
            ),
            interface_check(
                "message-field",
                "/localization_node/state.score",
                "finite value in [0,1]",
                score,
                score_ok,
            ),
        ],
    )


def parse_numbers(text: str) -> list[float]:
    return [
        float(value)
        for value in re.findall(r"[-+]?(?:\d+\.\d+|\d+)(?:[eE][-+]?\d+)?", text)
    ]


def check_ekf_parameters(reporter: Reporter) -> None:
    frequency_result = reporter.run(
        ["ros2", "param", "get", "/robot_localization", "frequency"]
    )
    frequency_numbers = parse_numbers(frequency_result.stdout)
    frequency = frequency_numbers[-1] if frequency_numbers else None
    passed = frequency is not None and abs(frequency - 25.0) <= 1e-6
    reporter.add(
        "LOC-A-06",
        "UKF 频率参数",
        PASS if passed else FAIL,
        f"frequency={frequency}",
        [
            interface_check(
                "parameter",
                "/robot_localization:frequency",
                25.0,
                frequency,
                frequency is not None and abs(frequency - 25.0) <= 1e-6,
            ),
        ],
    )


def check_ekf_rate(reporter: Reporter, sample_s: float) -> None:
    result = reporter.run_for(
        [
            "ros2",
            "topic",
            "hz",
            "/robot_localization/odometry/filtered",
            "--window",
            "50",
        ],
        sample_s,
    )
    rates = [
        float(value)
        for value in re.findall(r"average rate:\s*([0-9.]+)", result.stdout)
    ]
    rate = rates[-1] if rates else None
    passed = rate is not None and 23.0 <= rate <= 27.0
    reporter.add(
        "LOC-A-07",
        "EKF Odometry 稳态频率",
        PASS if passed else FAIL,
        f"average_rate={rate} Hz，门限 23~27 Hz"
        if rate is not None
        else f"{sample_s:.0f} 秒采样未得到 average rate",
        [
            interface_check(
                "topic-rate",
                "/robot_localization/odometry/filtered",
                "23~27 Hz",
                f"{rate} Hz" if rate is not None else "unavailable",
                passed,
            )
        ],
    )


def cpu_set(value: str) -> set[int]:
    result: set[int] = set()
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            start, end = item.split("-", 1)
            result.update(range(int(start), int(end) + 1))
        elif item.isdigit():
            result.add(int(item))
    return result


def check_ekf_runtime(reporter: Reporter) -> None:
    prefix_result = reporter.run(["ros2", "pkg", "prefix", "robot_localization"])
    prefix_lines = [
        line.strip() for line in prefix_result.stdout.splitlines() if line.strip()
    ]
    package_prefix = prefix_lines[-1] if prefix_lines else ""
    ukf_executable = (
        Path(package_prefix) / "lib" / "robot_localization" / "ukf_node"
        if package_prefix
        else Path("<unresolved>")
    )
    capability_result = reporter.run(["getcap", "-v", str(ukf_executable)])
    capability_text = (
        f"{capability_result.stdout}\n{capability_result.stderr}".strip()
    )
    capability_ok = (
        prefix_result.returncode == 0
        and capability_result.returncode == 0
        and "cap_sys_nice=ep" in capability_text
    )
    expected_rl_library = (
        Path(package_prefix) / "lib" / "librl_lib.so"
        if package_prefix
        else Path("<unresolved>")
    )
    ldd_result = reporter.run(["ldd", str(ukf_executable)])
    ldd_text = f"{ldd_result.stdout}\n{ldd_result.stderr}".strip()
    library_match = re.search(
        r"^\s*librl_lib\.so\s+=>\s+(\S+)",
        ldd_text,
        re.MULTILINE,
    )
    actual_rl_library = library_match.group(1) if library_match else ""
    library_ok = (
        prefix_result.returncode == 0
        and ldd_result.returncode == 0
        and bool(actual_rl_library)
        and Path(actual_rl_library).resolve() == expected_rl_library.resolve()
    )

    pid_result = reporter.run(["pgrep", "-n", "-x", "ukf_node"])
    pid = pid_result.stdout.strip()
    if not pid.isdigit():
        reporter.add(
            "LOC-A-08",
            "UKF 构建授权、动态库、调度策略与 CPU 亲和性",
            FAIL,
            f"build_capability={capability_text or 'unavailable'}；"
            f"librl_lib={actual_rl_library or 'unresolved'}；未找到 ukf_node 进程",
            [
                interface_check(
                    "file-capability",
                    str(ukf_executable),
                    "cap_sys_nice=ep",
                    capability_text or "unavailable",
                    capability_ok,
                ),
                interface_check(
                    "dynamic-library",
                    "librl_lib.so",
                    str(expected_rl_library),
                    actual_rl_library or ldd_text or "unresolved",
                    library_ok,
                ),
                interface_check(
                    "process", "ukf_node", "running", "not found", False
                )
            ],
        )
        return
    chrt_result = reporter.run(["chrt", "-p", pid])
    taskset_result = reporter.run(["taskset", "-pc", pid])
    chrt_text = f"{chrt_result.stdout}\n{chrt_result.stderr}"
    taskset_text = f"{taskset_result.stdout}\n{taskset_result.stderr}"
    policy_ok = "SCHED_FIFO" in chrt_text
    priority_match = re.search(r"priority:\s*(\d+)", chrt_text)
    priority = int(priority_match.group(1)) if priority_match else None
    affinity_match = re.search(r"affinity list:\s*([0-9,\-]+)", taskset_text)
    affinity = cpu_set(affinity_match.group(1)) if affinity_match else set()
    passed = (
        capability_ok
        and library_ok
        and policy_ok
        and priority == 99
        and affinity == {3, 4}
    )
    reporter.add(
        "LOC-A-08",
        "UKF 构建授权、动态库、调度策略与 CPU 亲和性",
        PASS if passed else FAIL,
        f"capability={'cap_sys_nice=ep' if capability_ok else capability_text or 'unavailable'} "
        f"librl_lib={actual_rl_library or 'unresolved'} "
        f"pid={pid} policy={'SCHED_FIFO' if policy_ok else 'other'} "
        f"priority={priority} cpus={sorted(affinity)}",
        [
            interface_check(
                "file-capability",
                str(ukf_executable),
                "cap_sys_nice=ep",
                capability_text or "unavailable",
                capability_ok,
            ),
            interface_check(
                "dynamic-library",
                "librl_lib.so",
                str(expected_rl_library),
                actual_rl_library or ldd_text or "unresolved",
                library_ok,
            ),
            interface_check(
                "process-scheduler",
                "ukf_node",
                "SCHED_FIFO/99",
                f"{'SCHED_FIFO' if policy_ok else 'other'}/{priority}",
                policy_ok and priority == 99,
            ),
            interface_check(
                "process-affinity",
                "ukf_node",
                "[3, 4]",
                sorted(affinity),
                affinity == {3, 4},
            ),
        ],
    )


def capture_package_prefixes(reporter: Reporter) -> None:
    prefixes: dict[str, str] = {}
    for package in (
        "cartographer_ros",
        "nav_interfaces",
        "map_manager",
        "map_asset_manager",
        "robot_localization",
    ):
        result = reporter.run(["ros2", "pkg", "prefix", package], timeout_s=8)
        prefixes[package] = (
            result.stdout.strip() if result.returncode == 0 else "NOT_FOUND"
        )
    reporter.metadata["package_prefixes"] = prefixes


def mode_checks(
    graph: GraphSnapshot, expected_mode: str
) -> list[InterfaceCheck]:
    mapping_present = has_node(graph.nodes, "mapping_node")
    localization_node_present = has_node(graph.nodes, "localization_node")
    map_manager_present = has_node(graph.nodes, "map_manager")
    if expected_mode == "mapping":
        return [
            interface_check(
                "node",
                "/mapping_node",
                "present",
                "present" if mapping_present else "absent",
                mapping_present,
            ),
            interface_check(
                "node-group",
                "/localization_node|/map_manager",
                "absent",
                (
                    "present"
                    if localization_node_present or map_manager_present
                    else "absent"
                ),
                not localization_node_present and not map_manager_present,
            ),
        ]
    return [
        interface_check(
            "node",
            "/mapping_node",
            "absent",
            "present" if mapping_present else "absent",
            not mapping_present,
        ),
        interface_check(
            "node-group",
            "/localization_node|/map_manager",
            "at least one present",
            (
                ",".join(
                    name
                    for name, present in (
                        ("/localization_node", localization_node_present),
                        ("/map_manager", map_manager_present),
                    )
                    if present
                )
                or "absent"
            ),
            localization_node_present or map_manager_present,
        ),
    ]


def run_check(
    reporter: Reporter,
    mode: str,
    *,
    rate_sample_s: float,
) -> tuple[GraphSnapshot | None, str]:
    requested_id = {
        "mapping": "MAP-A-01",
        "localization": "LOC-A-01",
    }.get(mode, "RUN-G-01")
    graph = graph_snapshot(reporter, requested_id)
    if graph is None:
        return None, mode
    mapping_present = has_node(graph.nodes, "mapping_node")
    localization_present = has_node(
        graph.nodes, "localization_node"
    ) or has_node(graph.nodes, "map_manager")
    detected = mode
    if mapping_present and localization_present:
        reporter.add(
            requested_id,
            "ROS2 图与运行模式唯一",
            FAIL,
            "mapping 与 localization 栈同时存在；生产契约要求两者互斥",
            [
                interface_check(
                    "runtime-mode",
                    "mapping/localization",
                    "exactly one stack",
                    "both stacks present",
                    False,
                )
            ],
        )
        reporter.metadata["detected_mode"] = "invalid"
        return graph, "invalid"
    if detected == "auto":
        if mapping_present:
            detected = "mapping"
        elif localization_present:
            detected = "localization"
        else:
            reporter.add(
                "RUN-G-01",
                "运行模式识别",
                PENDING_ENV,
                "未发现 mapping_node 或 localization/map_manager",
                [
                    interface_check(
                        "runtime-mode",
                        "mapping/localization",
                        "exactly one stack",
                        "no stack detected",
                        False,
                    )
                ],
            )
            return graph, detected
    mode_id = (
        "MAP-A-01" if detected == "mapping" else "LOC-A-01"
    )
    expected_present = (
        mapping_present if detected == "mapping" else localization_present
    )
    if not expected_present:
        reporter.add(
            mode_id,
            "ROS2 图与指定运行模式",
            FAIL,
            f"指定 mode={detected}，但对应业务栈未启动",
            mode_checks(graph, detected),
        )
        reporter.metadata["detected_mode"] = "invalid"
        return graph, "invalid"
    reporter.add(
        mode_id,
        "ROS2 图与运行模式唯一",
        PASS,
        f"检测到唯一业务栈：{detected}",
        mode_checks(graph, detected),
    )
    reporter.metadata["detected_mode"] = detected
    capture_package_prefixes(reporter)
    if detected == "mapping":
        check_mapping_contract(reporter, graph)
        check_legacy_interfaces(reporter, graph, "MAP-A-03")
    elif detected == "localization":
        check_localization_contract(reporter, graph)
        check_legacy_interfaces(reporter, graph, "LOC-A-03")
        check_map_state(reporter)
        check_localization_state(reporter)
        check_ekf_parameters(reporter)
        check_ekf_rate(reporter, rate_sample_s)
        check_ekf_runtime(reporter)
    return graph, detected


def call_service(
    reporter: Reporter,
    service: str,
    service_type: str,
    request: str,
    *,
    timeout_s: float = 30.0,
) -> CommandRecord:
    return reporter.run(
        ["ros2", "service", "call", service, service_type, request],
        timeout_s=timeout_s,
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def set_pose_request(pose: dict[str, float | str]) -> str:
    yaw = float(pose["yaw"])
    return (
        "{pose: {header: {frame_id: 'map'}, pose: {pose: {"
        f"position: {{x: {float(pose['x']):.9f}, "
        f"y: {float(pose['y']):.9f}, z: {float(pose['z']):.9f}}}, "
        "orientation: {x: 0.0, y: 0.0, "
        f"z: {math.sin(yaw / 2.0):.9f}, "
        f"w: {math.cos(yaw / 2.0):.9f}}}"
        "}}}}"
    )


def pose_trial_context(
    reference_pose: dict[str, float | str] | None,
    requested_pose: dict[str, float | str] | None,
) -> str:
    if reference_pose is None or requested_pose is None:
        return "perturbation=not-applicable"
    return (
        f"perturbation_distance_m={requested_pose.get('position_perturbation_m')} "
        f"perturbation_direction_deg={requested_pose.get('position_direction_deg')} "
        f"yaw_perturbation_deg={requested_pose.get('yaw_perturbation_deg')} "
        "reference_pose=("
        f"x={float(reference_pose['x']):.6f},"
        f"y={float(reference_pose['y']):.6f},"
        f"yaw_deg={math.degrees(float(reference_pose['yaw'])):.3f}) "
        "requested_pose=("
        f"x={float(requested_pose['x']):.6f},"
        f"y={float(requested_pose['y']):.6f},"
        f"yaw_deg={math.degrees(float(requested_pose['yaw'])):.3f})"
    )


def run_relocalization_trial(
    reporter: Reporter,
    *,
    trial: int,
    service: str,
    service_type: str,
    request: str,
    timeout_s: float,
    log_path: Path = DEFAULT_LOCALIZATION_LOG,
    log_tag: str = DEFAULT_LOCALIZATION_LOG_TAG,
    reference_pose: dict[str, float | str] | None = None,
    requested_pose: dict[str, float | str] | None = None,
) -> dict[str, Any]:
    scope = f"relocalization trial {'single' if trial == 0 else trial}"
    trial_context = pose_trial_context(reference_pose, requested_pose)
    if requested_pose is not None:
        reporter.script_step(scope, "生成本轮扰动位姿", trial_context)
        print(
            f"[PERTURBATION] trial={trial:02d}/{RELOCALIZATION_TRIALS} "
            f"{trial_context}"
        )
    reporter.script_step(
        scope,
        "启动持续状态观察器",
        "订阅 /localization_node/state，服务调用前必须先收到至少一个基线样本",
    )
    observer_started = time.monotonic()
    capture = LocalizationStateCapture(reporter)
    capture_started = capture.start()
    baseline_ready = capture_started and capture.wait_for_samples(
        1, min(STATE_CAPTURE_BASELINE_TIMEOUT_S, timeout_s)
    )
    observer_baseline_time_ms = round(
        (time.monotonic() - observer_started) * 1000.0, 3
    )
    baseline_count = len(capture.snapshot()) if baseline_ready else 0
    service_started = time.monotonic()
    service_ok = False
    samples: list[dict[str, Any]] = []
    seen_relocalizing = False
    latest_status: int | None = None
    failure_reason = ""
    final_score: float | None = None
    success = False
    transition_evidence = ""
    log_cursor: dict[str, Any] = {
        "available": False,
        "device": None,
        "inode": None,
        "offset": 0,
        "error": "not captured",
    }
    log_observation: dict[str, Any] = {
        "complete": False,
        "success": False,
        "total_ms": None,
        "timing_source": "unavailable",
        "pose_started_at_ros_s": None,
        "pose_succeeded_at_ros_s": None,
        "final_score": None,
        "timing_fields": {},
        "failure": "",
    }
    log_parse_reported = False
    log_wait_reported = False
    log_read_error = ""

    try:
        if not capture_started:
            failure_reason = f"state capture failed to start: {capture.start_error}"
            reporter.script_step(scope, "状态观察器启动失败", failure_reason)
        elif not baseline_ready:
            failure_reason = "state capture did not receive a baseline before service call"
            reporter.script_step(scope, "基线状态等待失败", failure_reason)
        else:
            baseline = capture.snapshot()[-1]
            reporter.script_step(
                scope,
                "确认订阅基线",
                f"status={baseline['status']} score={baseline['score']}",
            )
            log_cursor = capture_log_cursor(
                reporter, scope, log_path, log_tag
            )
            service_started = time.monotonic()
            reporter.script_step(
                scope,
                "调用重定位服务",
                f"service={service} type={service_type} request={request}",
            )
            service_result = call_service(
                reporter,
                service,
                service_type,
                request,
                timeout_s=min(timeout_s, 30.0),
            )
            service_completed = time.monotonic()
            service_ok = bool_success(
                f"{service_result.stdout}\n{service_result.stderr}"
            )
            reporter.script_step(
                scope,
                "解析服务响应",
                f"success={service_ok} returncode={service_result.returncode}",
            )
            if not service_ok:
                failure_reason = "service success=false"
            else:
                deadline = service_completed + timeout_s
                relocalizing_deadline = service_completed + min(
                    RELOCALIZING_STATE_OBSERVATION_TIMEOUT_S,
                    timeout_s,
                )
                processed = baseline_count
                first_relocalizing_at: float | None = None
                first_success_after_relocalizing_at: float | None = None
                while time.monotonic() < deadline:
                    captured = capture.snapshot()
                    for state in captured[processed:]:
                        elapsed_s = round(
                            max(0.0, state["captured_at"] - service_started),
                            3,
                        )
                        status = int(state["status"])
                        score = float(state["score"])
                        latest_status = status
                        samples.append(
                            {
                                "elapsed_s": elapsed_s,
                                "status": status,
                                "score": round(score, 6),
                            }
                        )
                        reporter.script_step(
                            scope,
                            "收到定位状态样本",
                            f"t={elapsed_s:.3f}s status={status} score={score:.6f}",
                        )
                        if status == 3:
                            seen_relocalizing = True
                            if first_relocalizing_at is None:
                                first_relocalizing_at = float(
                                    state["captured_at"]
                                )
                        if status in (-2, -1):
                            failure_reason = f"terminal failure status={status}"
                            break
                        if status == 4:
                            final_score = score
                            if (
                                first_relocalizing_at is not None
                                and first_success_after_relocalizing_at is None
                                and float(state["captured_at"])
                                >= first_relocalizing_at
                            ):
                                first_success_after_relocalizing_at = float(
                                    state["captured_at"]
                                )
                    processed = len(captured)
                    if failure_reason:
                        break

                    new_log_text, log_read_error = read_log_since_cursor(
                        log_path, log_cursor, log_tag
                    )
                    if new_log_text:
                        log_observation = parse_relocalization_log(new_log_text)
                    if log_observation["final_score"] is not None:
                        final_score = float(log_observation["final_score"])
                    if log_observation["complete"] and not log_parse_reported:
                        log_parse_reported = True
                        reporter.script_step(
                            scope,
                            "解析本轮定位完成日志",
                            (
                                f"success={log_observation['success']} "
                                f"total_ms={log_observation['total_ms']} "
                                f"source={log_observation['timing_source']} "
                                f"final_score={log_observation['final_score']} "
                                f"timing={log_observation['timing_fields']} "
                                f"failure={log_observation['failure'] or '-'}"
                            ),
                        )
                    if log_observation["complete"] and not log_observation["success"]:
                        failure_reason = (
                            str(log_observation["failure"])
                            or "same-trial log reports relocalization failure"
                        )
                        break

                    state_closed = seen_relocalizing and latest_status == 4
                    log_closed = bool(
                        log_observation["complete"]
                        and log_observation["success"]
                        and latest_status == 4
                    )
                    waiting_for_global_log_timing = bool(
                        state_closed
                        and service
                        == "/localization_node/trigger_global_relocalization"
                        and log_cursor["available"]
                        and not log_observation["complete"]
                        and time.monotonic() < relocalizing_deadline
                    )
                    if waiting_for_global_log_timing and not log_wait_reported:
                        log_wait_reported = True
                        reporter.script_step(
                            scope,
                            "状态已闭环，短暂等待本轮算法计时日志",
                            (
                                "observed status 3→4；最多等待到 5 秒门，"
                                "提取 Global relocalization took 后立即结束"
                            ),
                        )
                    if (state_closed and not waiting_for_global_log_timing) or log_closed:
                        if (
                            final_score is not None
                            and math.isfinite(final_score)
                            and 0.0 <= final_score <= 1.0
                        ):
                            success = True
                            transition_evidence = (
                                "observed status 3→4"
                                if state_closed
                                else "same-trial success log + post-call status 4"
                            )
                        else:
                            failure_reason = f"invalid final score={final_score}"
                        break
                    if (
                        time.monotonic() >= relocalizing_deadline
                        and not seen_relocalizing
                        and not log_observation["success"]
                    ):
                        failure_reason = (
                            "service success=true but neither status 3 nor a same-trial "
                            "success log was observed within "
                            f"{min(RELOCALIZING_STATE_OBSERVATION_TIMEOUT_S, timeout_s):.1f}s "
                            "after the service response"
                        )
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    capture.wait_for_samples(processed + 1, min(0.1, remaining))
                if not success and not failure_reason:
                    failure_reason = "timeout before observed status 3→4"
    finally:
        completed_at = time.monotonic()
        capture.close()

    duration_origin = service_started if baseline_ready else observer_started
    wall_duration_s = round(completed_at - duration_origin, 3)
    localization_time_ms = log_observation["total_ms"]
    duration_s = (
        round(float(localization_time_ms) / 1000.0, 3)
        if localization_time_ms is not None
        else wall_duration_s
    )
    timing_source = (
        str(log_observation["timing_source"])
        if localization_time_ms is not None
        else "wall_clock_fallback"
    )
    measured_time_ms = (
        round(float(localization_time_ms), 3)
        if localization_time_ms is not None
        else round(wall_duration_s * 1000.0, 3)
    )
    service_call_time_ms = (
        round(float(service_result.duration_s) * 1000.0, 3)
        if baseline_ready and "service_result" in locals()
        else None
    )
    status_3_observed_ms = (
        round(
            max(0.0, first_relocalizing_at - service_started) * 1000.0,
            3,
        )
        if baseline_ready
        and "first_relocalizing_at" in locals()
        and first_relocalizing_at is not None
        else None
    )
    status_recovery_time_ms = (
        round(
            (
                first_success_after_relocalizing_at - first_relocalizing_at
            )
            * 1000.0,
            3,
        )
        if baseline_ready
        and "first_relocalizing_at" in locals()
        and "first_success_after_relocalizing_at" in locals()
        and first_relocalizing_at is not None
        and first_success_after_relocalizing_at is not None
        else None
    )
    statuses = [sample["status"] for sample in samples]
    result = {
        "trial": trial,
        "service": service,
        "service_success": service_ok,
        "observer_baseline_received": baseline_ready,
        "observer_baseline_time_ms": observer_baseline_time_ms,
        "duration_s": duration_s,
        "wall_duration_s": wall_duration_s,
        "localization_time_ms": localization_time_ms,
        "measured_time_ms": measured_time_ms,
        "timing_source": timing_source,
        "service_call_time_ms": service_call_time_ms,
        "status_3_observed_ms": status_3_observed_ms,
        "status_recovery_time_ms": status_recovery_time_ms,
        "final_score": (
            round(final_score, 6) if final_score is not None else None
        ),
        "status_sequence": statuses,
        "samples": samples,
        "transition_evidence": transition_evidence,
        "log_path": str(log_path),
        "log_active_file": str(log_cursor.get("active_path", log_path)),
        "log_tag": log_tag,
        "log_cursor_offset": log_cursor["offset"],
        "log_available": log_cursor["available"],
        "log_read_error": log_read_error,
        "log_observation": log_observation,
        "success": success,
        "failure_reason": failure_reason,
    }
    if reference_pose is not None:
        result["reference_pose"] = reference_pose
    if requested_pose is not None:
        result["requested_pose"] = requested_pose
    reporter.script_step(
        scope,
        "重定位试验判定",
        (
            f"success={success} localization_time={duration_s:.3f}s "
            f"wall_time={wall_duration_s:.3f}s source={timing_source} "
            f"observer_baseline_ms={observer_baseline_time_ms} "
            f"service_call_ms={service_call_time_ms} "
            f"status3_at_ms={status_3_observed_ms} "
            f"status_recovery_ms={status_recovery_time_ms} "
            f"score={result['final_score']} statuses={statuses} "
            f"{trial_context} "
            f"evidence={transition_evidence or '-'} "
            f"reason={failure_reason or '-'}"
        ),
    )
    trial_label = (
        "single" if trial == 0 else f"{trial:02d}/{RELOCALIZATION_TRIALS}"
    )
    print(
        f"[METRIC] trial={trial_label} "
        f"service={service} success={success} "
        f"localization_time={duration_s:.3f}s "
        f"measured_time_ms={measured_time_ms} "
        f"log_time_ms={localization_time_ms} "
        f"timing_source={timing_source!r} "
        f"wall_time={wall_duration_s:.3f}s "
        f"observer_baseline_ms={observer_baseline_time_ms} "
        f"service_call_ms={service_call_time_ms} "
        f"status3_at_ms={status_3_observed_ms} "
        f"status_recovery_ms={status_recovery_time_ms} "
        f"score={result['final_score']} statuses={statuses} "
        f"{trial_context} "
        f"evidence={transition_evidence or '-'} "
        f"reason={failure_reason or '-'}"
    )
    return result


def trial_summary(trials: list[dict[str, Any]]) -> dict[str, Any]:
    succeeded = [trial for trial in trials if trial["success"]]
    times = [float(trial["duration_s"]) for trial in succeeded]
    times_ms = [
        float(trial["measured_time_ms"])
        for trial in succeeded
        if trial.get("measured_time_ms") is not None
    ]
    scores = [
        float(trial["final_score"])
        for trial in succeeded
        if trial["final_score"] is not None
    ]
    total = len(trials)
    success_count = len(succeeded)
    return {
        "total": total,
        "success_count": success_count,
        "failure_count": total - success_count,
        "success_rate": (
            round(success_count / total, 4) if total else 0.0
        ),
        "time_s": {
            "average": round(sum(times) / len(times), 3) if times else None,
            "minimum": round(min(times), 3) if times else None,
            "maximum": round(max(times), 3) if times else None,
        },
        "time_ms": {
            "average": round(sum(times_ms) / len(times_ms), 3)
            if times_ms
            else None,
            "minimum": round(min(times_ms), 3) if times_ms else None,
            "maximum": round(max(times_ms), 3) if times_ms else None,
            "sample_count": len(times_ms),
        },
        "score": {
            "average": round(sum(scores) / len(scores), 6)
            if scores
            else None,
            "minimum": round(min(scores), 6) if scores else None,
            "maximum": round(max(scores), 6) if scores else None,
        },
        "trials": trials,
    }


def store_relocalization_metrics(
    reporter: Reporter, key: str, summary: dict[str, Any]
) -> None:
    metrics = reporter.metadata.setdefault("relocalization_metrics", {})
    metrics[key] = summary
    atomic_json(reporter.artifact_dir / "relocalization_metrics.json", metrics)


def metric_interface_checks(
    service: str,
    trials: list[dict[str, Any]],
    log_tag: str = DEFAULT_LOCALIZATION_LOG_TAG,
) -> list[InterfaceCheck]:
    checks = []
    for trial in trials:
        score = trial["final_score"]
        checks.append(
            interface_check(
                "service+topic",
                (
                    f"{service} + /localization_node/state + "
                    f"{log_tag or 'all-tags'}@syslog "
                    f"trial {trial['trial']:02d}"
                ),
                (
                    "service success; status 3→4 or same-trial success log + "
                    "post-call status 4; score in [0,1]"
                ),
                (
                    f"success={trial['success']}, "
                    f"localization_time={trial['duration_s']}s, "
                    f"source={trial.get('timing_source', 'unavailable')}, score={score}, "
                    f"observer_baseline_ms={trial.get('observer_baseline_time_ms')}, "
                    f"service_call_ms={trial.get('service_call_time_ms')}, "
                    f"status3_at_ms={trial.get('status_3_observed_ms')}, "
                    f"status_recovery_ms={trial.get('status_recovery_time_ms')}, "
                    f"statuses={trial['status_sequence']}, "
                    f"{pose_trial_context(trial.get('reference_pose'), trial.get('requested_pose'))}, "
                    f"evidence={trial.get('transition_evidence', '-')}, "
                    f"reason={trial.get('failure_reason') or '-'}"
                ),
                bool(trial["success"]),
            )
        )
    return checks


def run_global_relocalization_metrics(
    reporter: Reporter,
    *,
    timeout_s: float,
    interval_s: float,
    log_path: Path,
    log_tag: str,
) -> dict[str, Any]:
    service = "/localization_node/trigger_global_relocalization"
    trials = []
    for trial in range(1, RELOCALIZATION_TRIALS + 1):
        trials.append(
            run_relocalization_trial(
                reporter,
                trial=trial,
                service=service,
                service_type="std_srvs/srv/Trigger",
                request="{}",
                timeout_s=timeout_s,
                log_path=log_path,
                log_tag=log_tag,
            )
        )
        if trial < RELOCALIZATION_TRIALS:
            time.sleep(interval_s)
    summary = trial_summary(trials)
    summary["required_success_rate"] = 1.0
    store_relocalization_metrics(reporter, "global_relocalization", summary)
    passed = (
        summary["total"] == RELOCALIZATION_TRIALS
        and summary["success_count"] == RELOCALIZATION_TRIALS
    )
    reporter.add(
        "LOC-A-13",
        "连续全局重定位 10 次",
        PASS if passed else FAIL,
        (
            f"成功={summary['success_count']}/{summary['total']} "
            f"成功率={summary['success_rate'] * 100:.1f}% "
            f"定位耗时(ms)={summary['time_ms']} "
            f"得分={summary['score']}"
        ),
        metric_interface_checks(service, trials, log_tag),
    )
    return summary


def run_perturbed_pose_metrics(
    reporter: Reporter,
    *,
    timeout_s: float,
    interval_s: float,
    seed: int,
    position_min_m: float,
    position_max_m: float,
    yaw_min_deg: float,
    yaw_max_deg: float,
    log_path: Path,
    log_tag: str,
) -> dict[str, Any]:
    service = "/localization_node/relocalization"
    bounds_ok = (
        0.0 <= position_min_m <= position_max_m
        and 0.0 <= yaw_min_deg <= yaw_max_deg <= 180.0
    )
    if not bounds_ok:
        summary = {
            "total": 0,
            "success_count": 0,
            "failure_count": 0,
            "success_rate": 0.0,
            "configuration_error": "invalid perturbation bounds",
            "trials": [],
        }
        store_relocalization_metrics(
            reporter, "perturbed_pose_relocalization", summary
        )
        reporter.add(
            "LOC-A-14",
            "当前全局位姿加扰动后的指定位姿重定位",
            ERROR,
            "扰动范围参数非法",
            [
                interface_check(
                    "argument",
                    "pose perturbation bounds",
                    "0<=min<=max; yaw max<=180deg",
                    (
                        f"position={position_min_m}..{position_max_m}m, "
                        f"yaw={yaw_min_deg}..{yaw_max_deg}deg"
                    ),
                    False,
                )
            ],
        )
        return summary

    rng = random.Random(seed)
    trials = []
    for trial in range(1, RELOCALIZATION_TRIALS + 1):
        reference = read_localization_pose(reporter, timeout_s=10.0)
        if (
            reference is None
            or str(reference["frame_id"]).lstrip("/") != "map"
        ):
            actual_frame = (
                reference["frame_id"] if reference is not None else "unreadable"
            )
            result = {
                "trial": trial,
                "service": service,
                "service_success": False,
                "duration_s": 0.0,
                "final_score": None,
                "status_sequence": [],
                "samples": [],
                "success": False,
                "failure_reason": (
                    "cannot read map-frame /localization_node/loc_pose; "
                    f"frame={actual_frame}"
                ),
            }
            trials.append(result)
            print(
                f"[METRIC] trial={trial:02d}/{RELOCALIZATION_TRIALS} "
                f"service={service} success=False time=0.000s score=None "
                "statuses=[] reason=cannot read map-frame global pose"
            )
        else:
            direction = rng.uniform(-math.pi, math.pi)
            radius = rng.uniform(position_min_m, position_max_m)
            yaw_magnitude = math.radians(
                rng.uniform(yaw_min_deg, yaw_max_deg)
            )
            yaw_delta = yaw_magnitude if rng.random() >= 0.5 else -yaw_magnitude
            requested = {
                "frame_id": "map",
                "x": round(
                    float(reference["x"]) + radius * math.cos(direction), 6
                ),
                "y": round(
                    float(reference["y"]) + radius * math.sin(direction), 6
                ),
                "z": round(float(reference["z"]), 6),
                "yaw": round(
                    normalize_angle(float(reference["yaw"]) + yaw_delta), 6
                ),
                "position_perturbation_m": round(radius, 6),
                "position_direction_deg": round(math.degrees(direction), 6),
                "yaw_perturbation_deg": round(math.degrees(yaw_delta), 6),
            }
            trials.append(
                run_relocalization_trial(
                    reporter,
                    trial=trial,
                    service=service,
                    service_type="nav_interfaces/srv/SetPose",
                    request=set_pose_request(requested),
                    timeout_s=timeout_s,
                    log_path=log_path,
                    log_tag=log_tag,
                    reference_pose=reference,
                    requested_pose=requested,
                )
            )
        if trial < RELOCALIZATION_TRIALS:
            time.sleep(interval_s)

    summary = trial_summary(trials)
    summary.update(
        {
            "required_success_rate": 1.0,
            "random_seed": seed,
            "position_perturbation_m": [
                position_min_m,
                position_max_m,
            ],
            "yaw_perturbation_deg": [yaw_min_deg, yaw_max_deg],
            "pose_source": "/localization_node/loc_pose",
        }
    )
    store_relocalization_metrics(
        reporter, "perturbed_pose_relocalization", summary
    )
    passed = (
        summary["total"] == RELOCALIZATION_TRIALS
        and summary["success_count"] == RELOCALIZATION_TRIALS
    )
    pose_acquisition_count = sum(
        "reference_pose" in trial for trial in trials
    )
    reporter.add(
        "LOC-A-14",
        "当前全局位姿加扰动后的指定位姿重定位 10 次",
        PASS if passed else FAIL,
        (
            f"seed={seed}；位置扰动={position_min_m}..{position_max_m}m；"
            f"角度扰动={yaw_min_deg}..{yaw_max_deg}deg；"
            f"成功={summary['success_count']}/{summary['total']} "
            f"成功率={summary['success_rate'] * 100:.1f}% "
            f"定位耗时(ms)={summary['time_ms']} "
            f"得分={summary['score']}"
        ),
        [
            interface_check(
                "topic",
                "/localization_node/loc_pose",
                "current pose readable in map frame before each trial",
                (
                    f"{pose_acquisition_count}/{RELOCALIZATION_TRIALS} "
                    "map-frame poses"
                ),
                pose_acquisition_count == RELOCALIZATION_TRIALS,
            ),
            *metric_interface_checks(service, trials, log_tag),
        ],
    )
    return summary


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonicalize_asset_json(
    value: Any, path: tuple[str, ...] = ()
) -> Any:
    if isinstance(value, dict):
        return {
            key: canonicalize_asset_json(item, path + (key,))
            for key, item in value.items()
        }
    if isinstance(value, list):
        normalized = [
            canonicalize_asset_json(item, path + ("*",))
            for item in value
        ]
        if (
            path in _UNORDERED_ASSET_ARRAY_PATHS
            or (path and path[-1] in _UNORDERED_ASSET_MEMBER_KEYS)
        ):
            return sorted(
                normalized,
                key=lambda item: json.dumps(
                    item,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ),
            )
        return normalized
    return value


def asset_semantic_digest(path: Path) -> str:
    value = json.loads(path.read_text(encoding="utf-8"))
    normalized = canonicalize_asset_json(value)
    payload = json.dumps(
        normalized,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def slam_file_hashes(slam_dir: Path) -> dict[str, str]:
    return {
        name: sha256_file(slam_dir / name)
        for name in SLAM_MAP_FILES
    }


def slam_backup_versions(map_dir: Path) -> dict[str, Path]:
    backup_root = map_dir / "slam" / "backups"
    if not backup_root.is_dir():
        return {}
    return {
        path.name.removeprefix("backup."): path
        for path in sorted(backup_root.iterdir())
        if path.is_dir() and path.name.startswith("backup.")
    }


def asset_backup_versions(map_dir: Path) -> dict[str, Path]:
    backup_root = map_dir / "assets" / "backups"
    if not backup_root.is_dir():
        return {}
    prefix = "map_assets."
    suffix = ".json"
    return {
        path.name[len(prefix) : -len(suffix)]: path
        for path in sorted(backup_root.iterdir())
        if path.is_file()
        and path.name.startswith(prefix)
        and path.name.endswith(suffix)
    }


def parse_unified_backup_versions(text: str) -> tuple[str, str]:
    match = re.search(
        r"backed up\s*\(\s*map=([^,\)\s]+)\s*,\s*"
        r"assets=([^\)\s]+)\s*\)",
        text,
    )
    return (match.group(1), match.group(2)) if match else ("", "")


def select_alternative_slam_version(
    versions: dict[str, Path],
    *,
    excluded_ts: str,
    current_hashes: dict[str, str],
) -> tuple[str, dict[str, str]] | None:
    for version_ts in sorted(versions, reverse=True):
        if version_ts == excluded_ts:
            continue
        try:
            candidate_hashes = slam_file_hashes(versions[version_ts])
        except OSError:
            continue
        if candidate_hashes != current_hashes:
            return version_ts, candidate_hashes
    return None


def select_alternative_asset_version(
    versions: dict[str, Path],
    *,
    excluded_ts: str,
    current_digest: str,
) -> tuple[str, str] | None:
    for version_ts in sorted(versions, reverse=True):
        if version_ts == excluded_ts:
            continue
        try:
            candidate_digest = asset_semantic_digest(versions[version_ts])
        except (OSError, ValueError, json.JSONDecodeError):
            continue
        if candidate_digest != current_digest:
            return version_ts, candidate_digest
    return None


def service_type(reporter: Reporter, service: str) -> str:
    result = reporter.run(["ros2", "service", "type", service], timeout_s=10)
    return result.stdout.strip() if result.returncode == 0 else ""


def model_c_rollback_once(
    reporter: Reporter,
    *,
    rollback_service: str,
    map_dir: Path,
    map_name: str,
    map_version_ts: str,
    assets_version_ts: str,
    before_epoch: int,
    expected_slam_hashes: dict[str, str],
    expected_asset_digest: str,
    localization_timeout_s: float,
) -> dict[str, Any]:
    service = rollback_service
    response = call_service(
        reporter,
        service,
        "nav_interfaces/srv/RollbackMap",
        (
            f"{{map_version_ts: '{map_version_ts}', "
            f"assets_version_ts: '{assets_version_ts}'}}"
        ),
        timeout_s=90,
    )
    text = f"{response.stdout}\n{response.stderr}"
    response_map_ts = parse_scalar(text, "map_version_ts")
    response_assets_ts = parse_scalar(text, "assets_version_ts")
    message = parse_scalar(text, "message")
    state = wait_map_state(
        reporter,
        min_epoch=before_epoch + 1,
        map_name=map_name,
        timeout_s=30,
    )
    localization_ok, statuses = wait_localization(
        reporter,
        timeout_s=localization_timeout_s,
    )
    final_state = read_localization_state(reporter, timeout_s=10)
    try:
        actual_slam_hashes = slam_file_hashes(map_dir / "slam")
        actual_asset_digest = asset_semantic_digest(
            map_dir / "assets" / "map_assets.json"
        )
        content_error = ""
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        actual_slam_hashes = {}
        actual_asset_digest = ""
        content_error = str(exc)
    service_ok = bool_success(text)
    response_versions_ok = (
        response_map_ts == map_version_ts
        and response_assets_ts == assets_version_ts
    )
    redeploy_ok = "redeploy ok" in message.lower()
    state_ok = bool(
        state
        and state["map_name"] == map_name
        and state["epoch"] == before_epoch + 1
    )
    score = final_state["score"] if final_state else None
    localization_state_ok = bool(
        localization_ok
        and final_state
        and final_state["status"] == 4
        and math.isfinite(float(final_state["score"]))
        and 0.0 <= float(final_state["score"]) <= 1.0
    )
    slam_ok = actual_slam_hashes == expected_slam_hashes
    assets_ok = actual_asset_digest == expected_asset_digest
    return {
        "service": service,
        "service_success": service_ok,
        "requested_map_version_ts": map_version_ts,
        "requested_assets_version_ts": assets_version_ts,
        "response_map_version_ts": response_map_ts,
        "response_assets_version_ts": response_assets_ts,
        "response_message": message,
        "response_versions_ok": response_versions_ok,
        "redeploy_ok": redeploy_ok,
        "before_epoch": before_epoch,
        "after_epoch": state["epoch"] if state else None,
        "map_state_ok": state_ok,
        "localization_statuses": statuses,
        "final_localization_state": final_state,
        "final_score": score,
        "localization_ok": localization_state_ok,
        "expected_slam_hashes": expected_slam_hashes,
        "actual_slam_hashes": actual_slam_hashes,
        "slam_content_ok": slam_ok,
        "expected_asset_semantic_digest": expected_asset_digest,
        "actual_asset_semantic_digest": actual_asset_digest,
        "asset_content_ok": assets_ok,
        "content_error": content_error,
        "success": bool(
            service_ok
            and response_versions_ok
            and redeploy_ok
            and state_ok
            and localization_state_ok
            and slam_ok
            and assets_ok
        ),
    }


def model_c_result_checks(
    label: str, result: dict[str, Any]
) -> list[InterfaceCheck]:
    return [
        interface_check(
            "service",
            result["service"],
            "success=true and response versions match request",
            (
                f"success={result['service_success']}, "
                f"map={result['response_map_version_ts']}, "
                f"assets={result['response_assets_version_ts']}"
            ),
            bool(
                result["service_success"]
                and result["response_versions_ok"]
            ),
        ),
        interface_check(
            "redeploy",
            f"{label} rollback_map response",
            "message contains redeploy ok",
            result["response_message"] or "empty",
            bool(result["redeploy_ok"]),
        ),
        interface_check(
            "topic",
            "/map_manager/map_state",
            f"same map and epoch={result['before_epoch'] + 1}",
            (
                f"epoch={result['after_epoch']}, "
                f"map_state_ok={result['map_state_ok']}"
            ),
            bool(result["map_state_ok"]),
        ),
        interface_check(
            "filesystem",
            f"{label} slam/map.pbstream+map.png+map.yaml",
            "SHA256 matches selected backup",
            f"matches={result['slam_content_ok']}",
            bool(result["slam_content_ok"]),
        ),
        interface_check(
            "filesystem",
            f"{label} assets/map_assets.json",
            "schema-aware semantic digest matches selected backup",
            f"matches={result['asset_content_ok']}",
            bool(result["asset_content_ok"]),
        ),
        interface_check(
            "topic",
            "/localization_node/state",
            "final status=4 and finite score in [0,1]",
            (
                f"statuses={result['localization_statuses']}, "
                f"score={result['final_score']}"
            ),
            bool(result["localization_ok"]),
        ),
    ]


def run_model_c_backup_rollback(
    reporter: Reporter,
    *,
    maps_root: Path,
    map_name: str,
    localization_timeout_s: float,
) -> dict[str, Any]:
    test_id = "LOC-A-15"
    graph = graph_snapshot(reporter, test_id)
    if graph is None:
        return {}
    complete_owners = [
        (node, backup, rollback)
        for node, backup, rollback in MODEL_C_OWNER_CANDIDATES
        if has_node(graph.nodes, node.lstrip("/"))
        and backup in graph.services
        and rollback in graph.services
    ]
    selected_owner = complete_owners[0] if len(complete_owners) == 1 else None
    owner_actual = (
        selected_owner[0]
        if selected_owner
        else (
            "none"
            if not complete_owners
            else "multiple: " + ", ".join(owner[0] for owner in complete_owners)
        )
    )
    checks = [
        interface_check(
            "node/service-owner",
            "Model C command board",
            "exactly one complete owner: command_board, command_board_manager or nav2_command_board",
            owner_actual,
            selected_owner is not None,
        )
    ]
    if selected_owner is None:
        for node, backup, rollback in MODEL_C_OWNER_CANDIDATES:
            checks.extend(
                [
                    interface_check(
                        "service-candidate",
                        backup,
                        "present together with matching owner and rollback service",
                        "present" if backup in graph.services else "unavailable",
                        False,
                    ),
                    interface_check(
                        "service-candidate",
                        rollback,
                        "present together with matching owner and backup service",
                        "present" if rollback in graph.services else "unavailable",
                        False,
                    ),
                ]
            )
        reporter.add(
            test_id,
            "Model C 统一备份、组合回滚与重新定位",
            FAIL,
            "未发现唯一完整的 Model C command board 服务提供方",
            checks,
        )
        return {}

    owner_node, backup_service, rollback_service = selected_owner
    service_types = {
        backup_service: "std_srvs/srv/Trigger",
        rollback_service: "nav_interfaces/srv/RollbackMap",
        "/localization_node/save_map": "nav_interfaces/srv/SaveMap",
        "/map_asset_manager/save_assets": "nav_interfaces/srv/SaveAssets",
    }
    observed_types = {
        service: service_type(reporter, service)
        for service in service_types
    }
    checks.extend(
        [
            interface_check(
                "service-type",
                service,
                expected,
                observed_types[service] or "unavailable",
                observed_types[service] == expected,
            )
            for service, expected in service_types.items()
        ]
    )
    if not all(check.normal for check in checks):
        reporter.add(
            test_id,
            "Model C 统一备份、组合回滚与重新定位",
            FAIL,
            "Model C 前置接口或唯一所有者不满足",
            checks,
        )
        return {}

    evidence: dict[str, Any] = {
        "map_name": map_name,
        "owner_node": owner_node,
        "backup_service": backup_service,
        "rollback_service": rollback_service,
        "service_types": observed_types,
        "started_at": now_iso(),
    }
    needs_restore = False
    backup_ok = False
    distinct_fixture = False
    target_result: dict[str, Any] | None = None
    restore_result: dict[str, Any] | None = None
    baseline_map_ts = ""
    baseline_assets_ts = ""
    original_slam_hashes: dict[str, str] = {}
    original_asset_digest = ""
    before_state: dict[str, Any] | None = None
    failure_reason = ""

    try:
        map_dir = safe_target(maps_root, map_name)
        before_state = read_map_state(reporter, timeout_s=10)
        if (
            not map_dir.is_dir()
            or not before_state
            or before_state["map_name"] != map_name
        ):
            raise RuntimeError(
                "active safe test map is required before Model C automation"
            )
        original_slam_hashes = slam_file_hashes(map_dir / "slam")
        original_asset_digest = asset_semantic_digest(
            map_dir / "assets" / "map_assets.json"
        )
        slam_before = slam_backup_versions(map_dir)
        assets_before = asset_backup_versions(map_dir)
        versions_absent = not (map_dir / "versions").exists()
        checks.append(
            interface_check(
                "filesystem",
                str(map_dir / "versions"),
                "absent in Model C",
                "absent" if versions_absent else "present",
                versions_absent,
            )
        )
        evidence["before"] = {
            "map_state": before_state,
            "slam_hashes": original_slam_hashes,
            "asset_semantic_digest": original_asset_digest,
            "slam_backup_versions": sorted(slam_before),
            "asset_backup_versions": sorted(assets_before),
        }

        backup_response = call_service(
            reporter,
            backup_service,
            "std_srvs/srv/Trigger",
            "{}",
            timeout_s=120,
        )
        backup_text = (
            f"{backup_response.stdout}\n{backup_response.stderr}"
        )
        baseline_map_ts, baseline_assets_ts = (
            parse_unified_backup_versions(backup_text)
        )
        slam_after = slam_backup_versions(map_dir)
        assets_after = asset_backup_versions(map_dir)
        new_slam_versions = sorted(set(slam_after) - set(slam_before))
        new_asset_versions = sorted(
            set(assets_after) - set(assets_before)
        )
        baseline_slam_hashes = (
            slam_file_hashes(slam_after[baseline_map_ts])
            if baseline_map_ts in slam_after
            else {}
        )
        baseline_asset_digest = (
            asset_semantic_digest(assets_after[baseline_assets_ts])
            if baseline_assets_ts in assets_after
            else ""
        )
        backup_ok = bool(
            bool_success(backup_text)
            and baseline_map_ts
            and baseline_assets_ts
            and baseline_map_ts in slam_after
            and baseline_assets_ts in assets_after
            and new_slam_versions == [baseline_map_ts]
            and new_asset_versions == [baseline_assets_ts]
            and baseline_slam_hashes == original_slam_hashes
            and baseline_asset_digest == original_asset_digest
        )
        needs_restore = bool(
            baseline_map_ts
            and baseline_assets_ts
            and baseline_map_ts in slam_after
            and baseline_assets_ts in assets_after
        )
        checks.extend(
            [
                interface_check(
                    "service",
                    backup_service,
                    "success=true with non-empty map/assets versions",
                    (
                        f"success={bool_success(backup_text)}, "
                        f"map={baseline_map_ts or 'empty'}, "
                        f"assets={baseline_assets_ts or 'empty'}"
                    ),
                    bool(
                        bool_success(backup_text)
                        and baseline_map_ts
                        and baseline_assets_ts
                    ),
                ),
                interface_check(
                    "filesystem",
                    "slam/backups",
                    "exactly returned map version newly created",
                    new_slam_versions,
                    new_slam_versions == [baseline_map_ts],
                ),
                interface_check(
                    "filesystem",
                    "assets/backups",
                    "exactly returned assets version newly created",
                    new_asset_versions,
                    new_asset_versions == [baseline_assets_ts],
                ),
                interface_check(
                    "filesystem",
                    "unified backup content",
                    "new backup matches pre-test map/assets content",
                    (
                        f"slam={baseline_slam_hashes == original_slam_hashes}, "
                        f"assets={baseline_asset_digest == original_asset_digest}"
                    ),
                    bool(
                        baseline_slam_hashes == original_slam_hashes
                        and baseline_asset_digest == original_asset_digest
                    ),
                ),
            ]
        )
        evidence["backup"] = {
            "service_success": bool_success(backup_text),
            "map_version_ts": baseline_map_ts,
            "assets_version_ts": baseline_assets_ts,
            "new_slam_versions": new_slam_versions,
            "new_asset_versions": new_asset_versions,
            "baseline_slam_hashes": baseline_slam_hashes,
            "baseline_asset_semantic_digest": baseline_asset_digest,
            "success": backup_ok,
        }
        if not backup_ok:
            failure_reason = "backup_unified or backup content validation failed"
        else:
            alternative_slam = select_alternative_slam_version(
                slam_after,
                excluded_ts=baseline_map_ts,
                current_hashes=original_slam_hashes,
            )
            alternative_assets = select_alternative_asset_version(
                assets_after,
                excluded_ts=baseline_assets_ts,
                current_digest=original_asset_digest,
            )
            distinct_fixture = bool(
                alternative_slam and alternative_assets
            )
            checks.append(
                interface_check(
                    "fixture",
                    str(map_dir),
                    "distinct map and assets backup versions available",
                    (
                        f"map={alternative_slam[0] if alternative_slam else 'none'}, "
                        f"assets={alternative_assets[0] if alternative_assets else 'none'}"
                    ),
                    distinct_fixture,
                )
            )
            evidence["alternative_fixture"] = {
                "map_version_ts": (
                    alternative_slam[0] if alternative_slam else ""
                ),
                "assets_version_ts": (
                    alternative_assets[0] if alternative_assets else ""
                ),
                "available": distinct_fixture,
            }
            if distinct_fixture:
                assert alternative_slam is not None
                assert alternative_assets is not None
                target_result = model_c_rollback_once(
                    reporter,
                    rollback_service=rollback_service,
                    map_dir=map_dir,
                    map_name=map_name,
                    map_version_ts=alternative_slam[0],
                    assets_version_ts=alternative_assets[0],
                    before_epoch=int(before_state["epoch"]),
                    expected_slam_hashes=alternative_slam[1],
                    expected_asset_digest=alternative_assets[1],
                    localization_timeout_s=localization_timeout_s,
                )
                checks.extend(
                    model_c_result_checks("selected-version", target_result)
                )
                evidence["target_rollback"] = target_result
            else:
                failure_reason = (
                    "no distinct map/assets backup pair in prepared test map"
                )
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        checks.append(
            interface_check(
                "exception",
                "LOC-A-15 execution",
                "completed without exception",
                failure_reason,
                False,
            )
        )
    finally:
        if needs_restore and before_state:
            try:
                latest_state = read_map_state(reporter, timeout_s=10)
                restore_before_epoch = int(
                    latest_state["epoch"]
                    if latest_state
                    and latest_state["map_name"] == map_name
                    else before_state["epoch"]
                )
                restore_result = model_c_rollback_once(
                    reporter,
                    rollback_service=rollback_service,
                    map_dir=safe_target(maps_root, map_name),
                    map_name=map_name,
                    map_version_ts=baseline_map_ts,
                    assets_version_ts=baseline_assets_ts,
                    before_epoch=restore_before_epoch,
                    expected_slam_hashes=original_slam_hashes,
                    expected_asset_digest=original_asset_digest,
                    localization_timeout_s=localization_timeout_s,
                )
                checks.extend(
                    model_c_result_checks("baseline-restore", restore_result)
                )
                evidence["baseline_restore"] = restore_result
            except (
                OSError,
                RuntimeError,
                ValueError,
                json.JSONDecodeError,
            ) as exc:
                restore_result = {
                    "success": False,
                    "failure_reason": f"{type(exc).__name__}: {exc}",
                }
                evidence["baseline_restore"] = restore_result
                checks.append(
                    interface_check(
                        "restore",
                        "Model C pre-test baseline",
                        "restored in finally",
                        restore_result["failure_reason"],
                        False,
                    )
                )

    target_ok = bool(target_result and target_result.get("success"))
    restore_ok = bool(restore_result and restore_result.get("success"))
    overall_ok = bool(
        backup_ok and distinct_fixture and target_ok and restore_ok
    )
    if not distinct_fixture and backup_ok and restore_ok:
        status = PENDING_ENV
    else:
        status = PASS if overall_ok else FAIL
    evidence.update(
        {
            "finished_at": now_iso(),
            "failure_reason": failure_reason,
            "success": overall_ok,
        }
    )
    reporter.metadata["model_c_backup_rollback"] = evidence
    atomic_json(reporter.artifact_dir / "model_c_metrics.json", evidence)
    reporter.add(
        test_id,
        "Model C 统一备份、组合回滚与重新定位",
        status,
        (
            f"backup={backup_ok} distinct_fixture={distinct_fixture} "
            f"rollback={target_ok} restore={restore_ok} "
            f"reason={failure_reason or '-'}"
        ),
        checks,
    )
    return evidence


def allowed_test_map(name: str) -> bool:
    return bool(re.fullmatch(r"[A-Za-z0-9_.-]+", name)) and name.startswith(
        ALLOWED_TEST_PREFIXES
    )


def restore_original_map(
    reporter: Reporter,
    original: dict[str, Any],
) -> tuple[bool, list[dict[str, Any]]]:
    attempts: list[dict[str, Any]] = []
    target_name = str(original["map_name"])
    for attempt in range(1, MAP_RESTORE_ATTEMPTS + 1):
        before = read_map_state(reporter, timeout_s=10.0)
        expected_epoch = (
            int(before["epoch"]) + 1
            if before is not None
            else int(original["epoch"]) + 1
        )
        reporter.script_step(
            "LOC-G-03",
            f"恢复原地图尝试 {attempt}/{MAP_RESTORE_ATTEMPTS}",
            (
                f"target={target_name} before_name="
                f"{before['map_name'] if before else 'unreadable'} "
                f"before_epoch={before['epoch'] if before else None} "
                f"expected_epoch={expected_epoch}"
            ),
        )
        response = call_service(
            reporter,
            "/map_manager/load_map",
            "nav_interfaces/srv/LoadMap",
            f"{{map_name: '{target_name}'}}",
            timeout_s=60.0,
        )
        response_text = f"{response.stdout}\n{response.stderr}"
        service_ok = response.returncode == 0 and bool_success(response_text)
        observed = wait_map_state(
            reporter,
            min_epoch=expected_epoch,
            map_name=target_name,
            timeout_s=MAP_RESTORE_STATE_TIMEOUT_S,
        )
        state_ok = bool(
            observed
            and observed["map_name"] == target_name
            and int(observed["epoch"]) == expected_epoch
        )
        attempt_result = {
            "attempt": attempt,
            "target_map": target_name,
            "before_map": before["map_name"] if before else None,
            "before_epoch": before["epoch"] if before else None,
            "expected_min_epoch": expected_epoch,
            "service_returncode": response.returncode,
            "service_success": service_ok,
            "service_stdout": response.stdout.strip(),
            "service_stderr": response.stderr.strip(),
            "observed_map": observed["map_name"] if observed else None,
            "observed_epoch": observed["epoch"] if observed else None,
            "state_restored": state_ok,
            "success": service_ok and state_ok,
        }
        attempts.append(attempt_result)
        reporter.script_step(
            "LOC-G-03",
            f"判定恢复尝试 {attempt}",
            (
                f"service_rc={response.returncode} service_success={service_ok} "
                f"observed_name={attempt_result['observed_map']} "
                f"observed_epoch={attempt_result['observed_epoch']} "
                f"state_restored={state_ok}"
            ),
        )
        if attempt_result["success"]:
            return True, attempts
    return False, attempts


def run_localization_mutations(
    reporter: Reporter,
    *,
    maps_root: Path,
    map_name: str,
    localization_timeout_s: float,
    skip_relocalization: bool,
    relocalization_interval_s: float,
    pose_random_seed: int,
    pose_position_min_m: float,
    pose_position_max_m: float,
    pose_yaw_min_deg: float,
    pose_yaw_max_deg: float,
    localization_log: Path,
    localization_log_tag: str,
) -> None:
    if not allowed_test_map(map_name):
        reporter.add(
            "LOC-G-02",
            "测试地图安全门",
            ERROR,
            "map-name 必须以 slam2d_test_、mm_test_ 或 test_ 开头，且只能含安全字符",
            [
                interface_check(
                    "argument",
                    "--map-name",
                    "safe test prefix",
                    map_name,
                    False,
                )
            ],
        )
        return
    original = read_map_state(reporter)
    if not original or not original["map_name"]:
        reporter.add(
            "LOC-G-02",
            "测试地图安全门",
            PENDING_ENV,
            "无法记录当前生产地图，拒绝执行会改变地图的测试",
            [
                interface_check(
                    "topic",
                    "/map_manager/map_state",
                    "readable current map",
                    "unreadable",
                    False,
                )
            ],
        )
        return
    reporter.add(
        "LOC-G-02",
        "测试地图与原地图记录安全门",
        PASS,
        f"测试图={map_name}；原地图={original['map_name']}",
        [
            interface_check(
                "argument",
                "--map-name",
                "safe test prefix",
                map_name,
                True,
            ),
            interface_check(
                "topic",
                "/map_manager/map_state",
                "readable current map",
                f"name={original['map_name']}, epoch={original['epoch']}",
                True,
            ),
        ],
    )
    reporter.metadata["original_map_state"] = original
    restored = original["map_name"] == map_name
    relocalization_smoke_passed = False
    try:
        load_result = call_service(
            reporter,
            "/map_manager/load_map",
            "nav_interfaces/srv/LoadMap",
            f"{{map_name: '{map_name}'}}",
        )
        loaded = wait_map_state(
            reporter,
            min_epoch=original["epoch"] + 1,
            map_name=map_name,
            timeout_s=15,
        )
        epoch_ok = bool(
            bool_success(load_result.stdout)
            and loaded
            and loaded["epoch"] == original["epoch"] + 1
        )
        reporter.add(
            "LOC-A-09",
            "同名/指定测试图 load_map 后 epoch 精确 +1",
            PASS if epoch_ok else FAIL,
            (
                f"name={map_name} epoch {original['epoch']}→"
                f"{loaded['epoch'] if loaded else None}"
            ),
            [
                interface_check(
                    "service",
                    "/map_manager/load_map",
                    "success=true",
                    f"success={bool_success(load_result.stdout)}",
                    bool_success(load_result.stdout),
                ),
                interface_check(
                    "topic",
                    "/map_manager/map_state",
                    f"name={map_name}, epoch={original['epoch'] + 1}",
                    (
                        f"name={loaded['map_name']}, epoch={loaded['epoch']}"
                        if loaded
                        else "unreadable"
                    ),
                    epoch_ok,
                ),
            ],
        )
        if not epoch_ok:
            return

        success, statuses = wait_localization(
            reporter, timeout_s=localization_timeout_s
        )
        reporter.add(
            "LOC-A-10",
            "地图加载后定位恢复 LOC_SUCCESS",
            PASS if success else FAIL,
            f"状态序列={statuses[-20:]}",
            [
                interface_check(
                    "topic",
                    "/localization_node/state",
                    "final status=4 without -2/-1",
                    statuses[-20:],
                    success,
                )
            ],
        )
        if not success:
            return

        if not skip_relocalization:
            relocalization_trial = run_relocalization_trial(
                reporter,
                trial=0,
                service="/localization_node/trigger_global_relocalization",
                service_type="std_srvs/srv/Trigger",
                request="{}",
                timeout_s=localization_timeout_s,
                log_path=localization_log,
                log_tag=localization_log_tag,
            )
            service_success = bool(relocalization_trial["service_success"])
            success = bool(relocalization_trial["success"])
            relocalization_smoke_passed = service_success and success
            statuses = list(relocalization_trial["status_sequence"])
            reporter.add(
                "LOC-A-11",
                "全局重定位 3→4 状态闭环",
                PASS if service_success and success else FAIL,
                f"service_success={service_success} "
                f"baseline={relocalization_trial['observer_baseline_received']} "
                f"定位耗时={relocalization_trial['duration_s']}s "
                f"日志耗时={relocalization_trial['localization_time_ms']}ms "
                f"计时来源={relocalization_trial['timing_source']} "
                f"得分={relocalization_trial['final_score']} "
                f"闭环证据={relocalization_trial['transition_evidence']} "
                f"状态序列={statuses[-20:]}",
                [
                    interface_check(
                        "service",
                        "/localization_node/trigger_global_relocalization",
                        "success=true",
                        f"success={service_success}",
                        service_success,
                    ),
                    interface_check(
                        "topic+log",
                        (
                            "/localization_node/state + "
                            f"{localization_log_tag}@syslog"
                        ),
                        (
                            "status 3→4 or same-trial success log + "
                            "post-call status 4"
                        ),
                        (
                            f"statuses={statuses[-20:]}, "
                            f"evidence={relocalization_trial['transition_evidence']}, "
                            f"time_source={relocalization_trial['timing_source']}"
                        ),
                        success,
                    ),
                ],
            )
        else:
            reporter.add(
                "LOC-A-11",
                "全局重定位 3→4 状态闭环",
                PENDING_ENV,
                "按 --skip-relocalization 跳过",
                [
                    interface_check(
                        "argument",
                        "--skip-relocalization",
                        "absent for official acceptance",
                        "provided",
                        False,
                    )
                ],
            )

        save_result = call_service(
            reporter,
            "/localization_node/save_map",
            "nav_interfaces/srv/SaveMap",
            f"{{map_name: '{map_name}'}}",
            timeout_s=90,
        )
        text = f"{save_result.stdout}\n{save_result.stderr}"
        map_path = parse_scalar(text, "map_path")
        backup_ts = parse_scalar(text, "backup_ts")
        expected_root = (maps_root / map_name).resolve()
        try:
            actual_path = Path(map_path).resolve()
            path_ok = actual_path == expected_root or expected_root in actual_path.parents
        except (OSError, RuntimeError):
            path_ok = False
        save_ok = bool_success(text) and bool(backup_ts) and path_ok
        reporter.add(
            "LOC-A-12",
            "localization save_map 分层路径与版本时间戳",
            PASS if save_ok else FAIL,
            f"success={bool_success(text)} path={map_path!r} backup_ts={backup_ts!r}",
            [
                interface_check(
                    "service",
                    "/localization_node/save_map",
                    "success=true",
                    f"success={bool_success(text)}",
                    bool_success(text),
                ),
                interface_check(
                    "filesystem-path",
                    "SaveMap.map_path",
                    str(expected_root),
                    map_path or "empty",
                    path_ok,
                ),
                interface_check(
                    "response-field",
                    "SaveMap.backup_ts",
                    "non-empty",
                    backup_ts or "empty",
                    bool(backup_ts),
                ),
            ],
        )
        if not skip_relocalization and relocalization_smoke_passed:
            run_global_relocalization_metrics(
                reporter,
                timeout_s=localization_timeout_s,
                interval_s=relocalization_interval_s,
                log_path=localization_log,
                log_tag=localization_log_tag,
            )
            run_perturbed_pose_metrics(
                reporter,
                timeout_s=localization_timeout_s,
                interval_s=relocalization_interval_s,
                seed=pose_random_seed,
                position_min_m=pose_position_min_m,
                position_max_m=pose_position_max_m,
                yaw_min_deg=pose_yaw_min_deg,
                yaw_max_deg=pose_yaw_max_deg,
                log_path=localization_log,
                log_tag=localization_log_tag,
            )
            run_model_c_backup_rollback(
                reporter,
                maps_root=maps_root,
                map_name=map_name,
                localization_timeout_s=localization_timeout_s,
            )
        else:
            pending_reason = (
                "按 --skip-relocalization 跳过"
                if skip_relocalization
                else "LOC-A-11 前置全局重定位 3→4 闭环失败，停止后续重复/扰动/回滚重定位测试"
            )
            for test_id, name in (
                ("LOC-A-13", "连续全局重定位 10 次"),
                (
                    "LOC-A-14",
                    "当前全局位姿加扰动后的指定位姿重定位 10 次",
                ),
                (
                    "LOC-A-15",
                    "Model C 统一备份、组合回滚与重新定位",
                ),
            ):
                reporter.add(
                    test_id,
                    name,
                    PENDING_ENV,
                    pending_reason,
                    [
                        interface_check(
                            "prerequisite",
                            "LOC-A-11",
                            "PASS before dependent relocalization tests",
                            (
                                "skipped by argument"
                                if skip_relocalization
                                else "FAIL"
                            ),
                            False,
                        )
                    ],
                )
    finally:
        restore_attempts: list[dict[str, Any]] = []
        if original["map_name"] != map_name:
            restored, restore_attempts = restore_original_map(
                reporter, original
            )
        else:
            restore_attempts.append(
                {
                    "attempt": 0,
                    "target_map": original["map_name"],
                    "before_map": original["map_name"],
                    "before_epoch": original["epoch"],
                    "expected_min_epoch": original["epoch"],
                    "service_returncode": None,
                    "service_success": True,
                    "service_stdout": "not needed; test map was already active",
                    "service_stderr": "",
                    "observed_map": original["map_name"],
                    "observed_epoch": original["epoch"],
                    "state_restored": True,
                    "success": True,
                }
            )
        reporter.metadata["original_map_restore"] = {
            "target_map": original["map_name"],
            "success": restored,
            "attempts": restore_attempts,
        }
        restore_checks = []
        for attempt in restore_attempts:
            label = (
                "not-needed"
                if attempt["attempt"] == 0
                else f"attempt-{attempt['attempt']}"
            )
            restore_checks.extend(
                [
                    interface_check(
                        "service",
                        f"/map_manager/load_map ({label})",
                        "returncode=0 and success=true",
                        (
                            f"returncode={attempt['service_returncode']}, "
                            f"success={attempt['service_success']}, "
                            f"stdout={attempt['service_stdout']!r}, "
                            f"stderr={attempt['service_stderr']!r}"
                        ),
                        bool(attempt["service_success"]),
                    ),
                    interface_check(
                        "topic",
                        f"/map_manager/map_state ({label})",
                        (
                            f"name={original['map_name']}, "
                            f"epoch={attempt['expected_min_epoch']}"
                        ),
                        (
                            f"name={attempt['observed_map']}, "
                            f"epoch={attempt['observed_epoch']}"
                        ),
                        bool(attempt["state_restored"]),
                    ),
                ]
            )
        reporter.add(
            "LOC-G-03",
            "测试结束恢复原地图",
            PASS if restored else FAIL,
            (
                f"目标={original['map_name']} restored={restored} "
                f"attempts={len(restore_attempts)}"
            ),
            restore_checks,
        )


def managed_state_path(result_root: Path) -> Path:
    return result_root / "managed_mapping_test.json"


def safe_target(maps_root: Path, map_name: str) -> Path:
    root = maps_root.resolve()
    target = (root / map_name).resolve()
    if target.parent != root or not allowed_test_map(map_name):
        raise ValueError(f"unsafe test map target: {target}")
    if target.is_symlink():
        raise ValueError(f"refusing symlink target: {target}")
    return target


def remove_managed_mapping_test(
    reporter: Reporter | None,
    *,
    result_root: Path,
) -> int:
    state_path = managed_state_path(result_root)
    if not state_path.is_file():
        if reporter:
            reporter.add(
                "MAP-G-03",
                "受管测试地图处置",
                PASS,
                "无 managed state，跳过",
                [
                    interface_check(
                        "filesystem",
                        str(state_path),
                        "absent or managed target cleaned",
                        "absent",
                        True,
                    )
                ],
            )
        else:
            print("managed state does not exist; nothing to clean")
        return 0
    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state.get("schema_version") != 1:
            raise ValueError("unsupported managed state schema")
        maps_root = Path(state["maps_root"]).resolve()
        target = safe_target(maps_root, state["map_name"])
        if target != Path(state["target"]).resolve():
            raise ValueError("managed target mismatch")
        if target.exists():
            shutil.rmtree(target)
        state_path.unlink()
    except Exception as exc:
        if reporter:
            reporter.add(
                "MAP-G-03",
                "受管测试地图处置",
                ERROR,
                str(exc),
                [
                    interface_check(
                        "filesystem",
                        str(state_path),
                        "managed target cleaned",
                        str(exc),
                        False,
                    )
                ],
            )
        else:
            print(f"cleanup failed: {exc}", file=sys.stderr)
        return 1
    if reporter:
        reporter.add(
            "MAP-G-03",
            "受管测试地图处置",
            PASS,
            str(target),
            [
                interface_check(
                    "filesystem",
                    str(target),
                    "absent",
                    "absent" if not target.exists() else "present",
                    not target.exists(),
                )
            ],
        )
    else:
        print(f"removed: {target}")
    return 0


def prepare_mapping_sentinels(
    maps_root: Path,
    map_name: str,
    result_root: Path,
) -> Path:
    target = safe_target(maps_root, map_name)
    state_path = managed_state_path(result_root)
    if target.exists():
        raise FileExistsError(
            f"test map already exists: {target}; run cleanup or choose another name"
        )
    if state_path.exists():
        raise FileExistsError(
            f"managed state already exists: {state_path}; run cleanup first"
        )
    try:
        (target / "slam").mkdir(parents=True)
        (target / "assets").mkdir()
        (target / "versions").mkdir()
        (target / "slam/old_slam.sentinel").write_text(
            "old\n", encoding="utf-8"
        )
        (target / "assets/old_assets.sentinel").write_text(
            "old\n", encoding="utf-8"
        )
        (target / "versions/old_versions.sentinel").write_text(
            "old\n", encoding="utf-8"
        )
        (target / "old_root.sentinel").write_text("old\n", encoding="utf-8")
        atomic_json(
            state_path,
            {
                "schema_version": 1,
                "created_at": now_iso(),
                "maps_root": str(maps_root.resolve()),
                "map_name": map_name,
                "target": str(target),
            },
        )
    except Exception:
        if target.exists():
            shutil.rmtree(target)
        raise
    return target


def run_mapping_mutations(
    reporter: Reporter,
    *,
    maps_root: Path,
    result_root: Path,
    map_name: str,
    keep_test_map: bool,
) -> None:
    started = False
    prepared = False
    target: Path | None = None
    initial_state = read_mapping_state(reporter, timeout_s=10)
    if initial_state is None:
        reporter.add(
            "MAP-G-02",
            "建图变更前状态门",
            PENDING_ENV,
            "10 秒内未读到 /mapping_node/state；未创建测试目录、未调用控制服务",
            [
                interface_check(
                    "topic",
                    "/mapping_node/state",
                    "status=IDLE(0)",
                    "unreadable",
                    False,
                )
            ],
        )
        return
    if initial_state[0] != 0:
        reporter.add(
            "MAP-G-02",
            "建图变更前状态门",
            FAIL,
            f"当前 status={initial_state[0]} map_name={initial_state[1]!r}，"
            "要求 IDLE(0)；未创建测试目录、未调用控制服务",
            [
                interface_check(
                    "topic",
                    "/mapping_node/state",
                    "status=IDLE(0)",
                    f"status={initial_state[0]}",
                    False,
                )
            ],
        )
        return
    reporter.add(
        "MAP-G-02",
        "建图变更前状态门",
        PASS,
        f"当前 IDLE(0)，map_name={initial_state[1]!r}",
        [
            interface_check(
                "topic",
                "/mapping_node/state",
                "status=IDLE(0)",
                f"status={initial_state[0]}",
                True,
            )
        ],
    )
    try:
        target = prepare_mapping_sentinels(maps_root, map_name, result_root)
        prepared = True
        start = call_service(
            reporter,
            "/mapping_node/start_mapping",
            "nav_interfaces/srv/StartMapping",
            f"{{map_name: '{map_name}', overwrite: true}}",
        )
        started = bool_success(start.stdout)
        reached_mapping, statuses = wait_mapping_status(
            reporter, wanted=1, timeout_s=20
        )
        reporter.add(
            "MAP-A-04",
            "start_mapping(map_name, overwrite) → MAPPING",
            PASS
            if started and reached_mapping
            else FAIL,
            f"start={started} "
            f"状态序列={statuses}",
            [
                interface_check(
                    "service",
                    "/mapping_node/start_mapping",
                    "success=true",
                    f"success={started}",
                    started,
                ),
                interface_check(
                    "topic",
                    "/mapping_node/state",
                    "contains MAPPING(1)",
                    statuses,
                    reached_mapping,
                ),
            ],
        )
        if not started:
            return
        root_entries = sorted(path.name for path in target.iterdir())
        sentinels = list(target.rglob("*.sentinel"))
        reset_ok = root_entries == ["slam"] and not sentinels
        reporter.add(
            "MAP-A-05",
            "同名建图全目录重建",
            PASS if reset_ok else FAIL,
            f"root_entries={root_entries} sentinel_count={len(sentinels)}",
            [
                interface_check(
                    "filesystem",
                    str(target),
                    "root_entries=['slam'], sentinel_count=0",
                    f"root_entries={root_entries}, sentinel_count={len(sentinels)}",
                    reset_ok,
                )
            ],
        )
        cancel = call_service(
            reporter,
            "/mapping_node/cancel_mapping",
            "std_srvs/srv/Trigger",
            "{}",
        )
        reached_idle, statuses = wait_mapping_status(
            reporter, wanted=0, timeout_s=20
        )
        started = not reached_idle
        reporter.add(
            "MAP-A-06",
            "cancel_mapping 后回到 IDLE",
            PASS if bool_success(cancel.stdout) and reached_idle else FAIL,
            f"cancel={bool_success(cancel.stdout)} 状态序列={statuses}",
            [
                interface_check(
                    "service",
                    "/mapping_node/cancel_mapping",
                    "success=true",
                    f"success={bool_success(cancel.stdout)}",
                    bool_success(cancel.stdout),
                ),
                interface_check(
                    "topic",
                    "/mapping_node/state",
                    "final IDLE(0)",
                    statuses,
                    reached_idle,
                ),
            ],
        )
    except Exception as exc:
        reporter.add(
            "MAP-A-04",
            "受管建图测试准备/执行",
            ERROR,
            str(exc),
            [
                interface_check(
                    "managed-operation",
                    "/mapping_node/start_mapping",
                    "completed without exception",
                    f"{type(exc).__name__}: {exc}",
                    False,
                )
            ],
        )
    finally:
        if started:
            call_service(
                reporter,
                "/mapping_node/cancel_mapping",
                "std_srvs/srv/Trigger",
                "{}",
            )
        if prepared and keep_test_map:
            reporter.add(
                "MAP-G-03",
                "受管测试地图处置",
                PASS,
                f"按 --keep-test-map 保留：{target}",
                [
                    interface_check(
                        "filesystem",
                        str(target),
                        "present by explicit request",
                        "present" if target and target.exists() else "absent",
                        bool(target and target.exists()),
                    )
                ],
            )
        elif prepared:
            remove_managed_mapping_test(reporter, result_root=result_root)


MANUAL_TEST_CATALOG = {
    "mapping": (
        ("MAP-M-01", "mapping_operation 建图保存后的状态与产物"),
        ("MAP-M-02", "建图中的 /map 更新与 map→base_link TF"),
    ),
    "localization": (
        ("LOC-M-01", "RViz 初始位姿/全局重定位后的定位状态"),
    ),
}
MANUAL_TEST_IDS = tuple(
    test_id
    for mode in ("mapping", "localization")
    for test_id, _ in MANUAL_TEST_CATALOG[mode]
)


def topic_rate_check(
    reporter: Reporter, topic: str, sample_s: float
) -> InterfaceCheck:
    result = reporter.run_for(
        ["ros2", "topic", "hz", topic, "--window", "20"],
        sample_s,
    )
    rates = [
        float(value)
        for value in re.findall(r"average rate:\s*([0-9.]+)", result.stdout)
    ]
    rate = rates[-1] if rates else None
    return interface_check(
        "topic-rate",
        topic,
        ">0 Hz",
        f"{rate} Hz" if rate is not None else "unavailable",
        rate is not None and rate > 0.0,
    )


def tf_check(reporter: Reporter, sample_s: float) -> InterfaceCheck:
    result = reporter.run_for(
        ["ros2", "run", "tf2_ros", "tf2_echo", "map", "base_link"],
        sample_s,
    )
    text = f"{result.stdout}\n{result.stderr}"
    normal = bool(result.stdout.strip()) and not re.search(
        r"could not transform|invalid frame|does not exist", text, re.IGNORECASE
    )
    return interface_check(
        "tf",
        "map→base_link",
        "transform readable",
        "readable" if normal else "unavailable",
        normal,
    )


def systemd_check(
    reporter: Reporter, unit: str, expected: str = "active"
) -> InterfaceCheck:
    result = reporter.run(
        ["systemctl", "is-active", unit], timeout_s=8
    )
    actual = result.stdout.strip() or result.stderr.strip() or "unavailable"
    return interface_check(
        "systemd-service",
        unit,
        expected,
        actual,
        result.returncode == 0 and actual == expected,
    )


def manual_graph(
    reporter: Reporter, test_id: str
) -> GraphSnapshot | None:
    return graph_snapshot(reporter, f"{test_id}-ENV")


def handle_manual_mapping(
    reporter: Reporter, args: argparse.Namespace
) -> None:
    test_id = args.case
    checks: list[InterfaceCheck] = []
    detail_parts = [
        "仅检查机器可判定状态；"
        "仍须按文档完成人工操作与 RViz 观察"
    ]
    if test_id == "MAP-M-01":
        if not args.map_name:
            reporter.add(
                test_id,
                "建图保存后状态辅助检查",
                ERROR,
                "缺少 --map-name",
                [
                    interface_check(
                        "argument", "--map-name", "non-empty", "empty", False
                    )
                ],
            )
            return
        graph = manual_graph(reporter, test_id)
        if graph is None:
            return
        checks.extend(
            presence_checks(
                "node", graph.nodes, ("mapping_node",), expected_present=True
            )
        )
        state = read_mapping_state(reporter, timeout_s=10)
        state_ok = state is not None and state[0] == 0
        checks.append(
            interface_check(
                "topic",
                "/mapping_node/state",
                "final IDLE(0)",
                f"status={state[0]}, map_name={state[1]}" if state else "unreadable",
                state_ok,
            )
        )
        checks.append(systemd_check(reporter, "dexe-map.service"))
        try:
            target = safe_target(args.maps_root, args.map_name)
            slam_files = (
                [path for path in (target / "slam").rglob("*") if path.is_file()]
                if (target / "slam").is_dir()
                else []
            )
            checks.append(
                interface_check(
                    "filesystem",
                    str(target / "slam"),
                    "saved SLAM files present",
                    f"{len(slam_files)} files",
                    bool(slam_files),
                )
            )
        except (OSError, RuntimeError, ValueError) as exc:
            checks.append(
                interface_check(
                    "filesystem",
                    f"{args.maps_root}/{args.map_name}/slam",
                    "saved SLAM files present",
                    exc,
                    False,
                )
            )
        name = "mapping_operation 建图保存后状态辅助检查"
    elif test_id == "MAP-M-02":
        graph = manual_graph(reporter, test_id)
        if graph is None:
            return
        checks.extend(
            presence_checks(
                "topic", graph.topics, ("/map",), expected_present=True
            )
        )
        state = read_mapping_state(reporter, timeout_s=10)
        checks.append(
            interface_check(
                "topic",
                "/mapping_node/state",
                "MAPPING(1) while observing",
                f"status={state[0]}" if state else "unreadable",
                state is not None and state[0] == 1,
            )
        )
        checks.append(topic_rate_check(reporter, "/map", args.sample_seconds))
        checks.append(tf_check(reporter, min(args.sample_seconds, 5.0)))
        name = "建图中 /map 与 TF 状态辅助检查"
    passed = bool(checks) and all(check.normal for check in checks)
    reporter.add(
        test_id,
        name,
        PASS if passed else FAIL,
        "；".join(detail_parts),
        checks,
    )


def handle_manual_localization(
    reporter: Reporter, args: argparse.Namespace
) -> None:
    test_id = args.case
    graph = manual_graph(reporter, test_id)
    if graph is None:
        return
    checks: list[InterfaceCheck] = []
    detail_parts = [
        "仅检查机器可判定状态；"
        "仍须按文档完成人工操作与 RViz 观察"
    ]
    checks.extend(
        presence_checks(
            "node",
            graph.nodes,
            ("localization_node", "robot_localization"),
            expected_present=True,
        )
    )
    checks.extend(
        presence_checks(
            "topic",
            graph.topics,
            (
                "/localization_node/state",
                "/robot_localization/odometry/filtered",
                "/robot_localization/set_pose",
            ),
            expected_present=True,
        )
    )
    endpoint_errors, endpoint_checks = check_set_pose_endpoints(reporter)
    checks.extend(endpoint_checks)
    detail_parts.extend(endpoint_errors)
    status = read_localization_status(reporter, timeout_s=10)
    checks.append(
        interface_check(
            "topic",
            "/localization_node/state",
            "LOC_SUCCESS(4)",
            status,
            status == 4,
        )
    )
    checks.append(tf_check(reporter, min(args.sample_seconds, 5.0)))
    name = "RViz 初始位姿/全局重定位后状态辅助检查"
    passed = bool(checks) and all(check.normal for check in checks)
    reporter.add(
        test_id,
        name,
        PASS if passed else FAIL,
        "；".join(detail_parts),
        checks,
    )


def manual_artifact_dir(args: argparse.Namespace) -> Path:
    if args.artifact_dir:
        return args.artifact_dir.resolve()
    base = args.manual_result_root.resolve()
    candidate = base / timestamp()
    suffix = 1
    while candidate.exists():
        candidate = base / f"{timestamp()}_{suffix:02d}"
        suffix += 1
    return candidate


def artifact_dir(args: argparse.Namespace, command: str) -> Path:
    if args.artifact_dir:
        return args.artifact_dir.resolve()
    base = args.result_root.resolve()
    candidate = base / f"{command}_{timestamp()}"
    suffix = 1
    while candidate.exists():
        candidate = base / f"{command}_{timestamp()}_{suffix:02d}"
        suffix += 1
    return candidate


def add_verbose_argument(
    parser: argparse.ArgumentParser, *, suppress_default: bool = False
) -> None:
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=argparse.SUPPRESS if suppress_default else False,
        help=(
            "print detailed STEP/OP commands, timings and raw stdout/stderr; "
            "without it, print test actions, interface checks and conclusions"
        ),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "SLAM 2D interface checks with JSON/Markdown evidence. "
            "The tool never switches systemd services."
        )
    )
    add_verbose_argument(parser)
    parser.add_argument(
        "--result-root", type=Path, default=DEFAULT_RESULT_ROOT
    )
    parser.add_argument(
        "--manual-result-root",
        type=Path,
        default=DEFAULT_MANUAL_RESULT_ROOT,
        help=(
            "manual state-check root "
            "(default: ~/.dexe/localization/slam_2d_manual)"
        ),
    )
    parser.add_argument("--artifact-dir", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_tests = subparsers.add_parser(
        "list-tests", help="print the automated test catalog"
    )
    add_verbose_argument(list_tests, suppress_default=True)
    list_tests.add_argument(
        "--mode",
        choices=("all", "mapping", "localization"),
        default="all",
        help="limit the catalog to one test column",
    )
    list_tests.add_argument(
        "--include-gates",
        action="store_true",
        help="also print non-functional safety gates",
    )
    list_tests.add_argument(
        "--include-manual",
        action="store_true",
        help="also print manual cases supported by manual-check",
    )
    list_tests.set_defaults(handler=handle_list_tests)

    check = subparsers.add_parser(
        "check", help="read-only interface and runtime checks"
    )
    add_verbose_argument(check, suppress_default=True)
    check.add_argument(
        "--mode",
        choices=("auto", "mapping", "localization"),
        default="auto",
    )
    check.add_argument("--rate-sample", type=float, default=8.0)
    check.set_defaults(handler=handle_check)

    localization = subparsers.add_parser(
        "localization",
        help="run localization checks plus test-map mutations and restore",
    )
    add_verbose_argument(localization, suppress_default=True)
    localization.add_argument(
        "--maps-root", type=Path, default=DEFAULT_MAPS_ROOT
    )
    localization.add_argument("--map-name", required=True)
    localization.add_argument("--confirm-mutations", action="store_true")
    localization.add_argument(
        "--skip-relocalization",
        action="store_true",
        help=(
            "diagnostic-only: skip LOC-A-11/13/14/15 and mark them PENDING_ENV"
        ),
    )
    localization.add_argument("--localization-timeout", type=float, default=120.0)
    localization.add_argument(
        "--localization-log",
        type=Path,
        default=DEFAULT_LOCALIZATION_LOG,
        help=(
            "shared syslog directory or explicit log file used for "
            "same-trial timing and score (default: /var/log/syslog)"
        ),
    )
    localization.add_argument(
        "--localization-log-tag",
        default=DEFAULT_LOCALIZATION_LOG_TAG,
        help=(
            "logger(1) tag selected from the shared syslog "
            "(default: dexe_localization; empty keeps all lines)"
        ),
    )
    localization.add_argument(
        "--relocalization-interval",
        type=float,
        default=0.5,
        help="settling interval between the fixed 10 trials",
    )
    localization.add_argument(
        "--pose-random-seed", type=int, default=20260731
    )
    localization.add_argument(
        "--pose-position-min", type=float, default=0.8
    )
    localization.add_argument(
        "--pose-position-max", type=float, default=1.5
    )
    localization.add_argument(
        "--pose-yaw-min-deg", type=float, default=20.0
    )
    localization.add_argument(
        "--pose-yaw-max-deg", type=float, default=45.0
    )
    localization.add_argument("--rate-sample", type=float, default=8.0)
    localization.set_defaults(handler=handle_localization)

    mapping = subparsers.add_parser(
        "mapping",
        help="run mapping checks plus managed same-name reset test",
    )
    add_verbose_argument(mapping, suppress_default=True)
    mapping.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    mapping.add_argument("--map-name", default=f"slam2d_test_{timestamp()}")
    mapping.add_argument("--confirm-mutations", action="store_true")
    mapping.add_argument("--keep-test-map", action="store_true")
    mapping.set_defaults(handler=handle_mapping)

    manual = subparsers.add_parser(
        "manual-check",
        help=(
            "capture read-only machine-state evidence for one manual case; "
            "never records a ROS bag"
        ),
    )
    add_verbose_argument(manual, suppress_default=True)
    manual.add_argument("--case", choices=MANUAL_TEST_IDS, required=True)
    manual.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    manual.add_argument("--map-name")
    manual.add_argument("--sample-seconds", type=float, default=8.0)
    manual.set_defaults(handler=handle_manual_check)

    cleanup = subparsers.add_parser(
        "cleanup", help="remove only the mapping test recorded in managed state"
    )
    add_verbose_argument(cleanup, suppress_default=True)
    cleanup.set_defaults(handler=handle_cleanup)
    return parser


TEST_CATALOG = {
    "mapping": (
        ("MAP-A-01", "ROS2 图与建图模式唯一"),
        ("MAP-A-02", "建图输入/输出/服务接口名称与类型契约"),
        ("MAP-A-03", "建图模式旧接口与旧节点域无残留"),
        ("MAP-A-04", "受管测试图 start_mapping(map_name)"),
        ("MAP-A-05", "同名建图全目录重建"),
        ("MAP-A-06", "cancel_mapping 后回 IDLE"),
    ),
    "localization": (
        ("LOC-A-01", "ROS2 图与定位模式唯一"),
        ("LOC-A-02", "定位输入/输出/服务接口名称与类型契约"),
        ("LOC-A-03", "定位模式旧接口与旧节点域无残留"),
        ("LOC-A-04", "MapState transient-local 原子状态"),
        ("LOC-A-05", "LocalizationState 状态与得分可读"),
        ("LOC-A-06", "UKF 25 Hz 参数"),
        ("LOC-A-07", "EKF Odometry 23~27 Hz"),
        ("LOC-A-08", "UKF SCHED_FIFO 99 / CPU 3,4"),
        ("LOC-A-09", "测试图 load_map 后 epoch 精确 +1"),
        ("LOC-A-10", "地图加载后定位恢复"),
        ("LOC-A-11", "全局重定位 3→4"),
        ("LOC-A-12", "localization save_map 路径与版本"),
        ("LOC-A-13", "连续全局重定位 10 次耗时/得分/成功率"),
        (
            "LOC-A-14",
            "当前全局位姿加扰动后的指定位姿重定位 10 次",
        ),
        (
            "LOC-A-15",
            "Model C 统一备份、组合回滚与重新定位",
        ),
    ),
}

GATE_CATALOG = {
    "mapping": (
        ("MAP-G-01", "破坏性操作显式确认"),
        ("MAP-G-02", "建图变更前 MappingState=IDLE"),
        ("MAP-G-03", "受管测试地图处置"),
    ),
    "localization": (
        ("LOC-G-01", "破坏性操作显式确认"),
        ("LOC-G-02", "测试地图名合法且原地图已记录"),
        ("LOC-G-03", "测试结束恢复原地图"),
    ),
}


def handle_list_tests(args: argparse.Namespace) -> int:
    modes = (
        ("mapping", "localization")
        if args.mode == "all"
        else (args.mode,)
    )
    for mode in modes:
        for test_id, name in TEST_CATALOG[mode]:
            print(f"{mode}\tautomated\t{test_id}\t{name}")
        if args.include_gates:
            for test_id, name in GATE_CATALOG[mode]:
                print(f"{mode}\tgate\t{test_id}\t{name}")
        if args.include_manual:
            for test_id, name in MANUAL_TEST_CATALOG[mode]:
                print(f"{mode}\tmanual\t{test_id}\t{name}")
    return 0


def handle_manual_check(args: argparse.Namespace) -> int:
    reporter = Reporter(
        f"manual-check {args.case}",
        manual_artifact_dir(args),
        verbose=args.verbose,
    )
    reporter.metadata.update(
        {
            "evidence_scope": "manual-machine-state-helper",
            "manual_case": args.case,
            "records_rosbag": False,
            "manual_result_root": str(args.manual_result_root.resolve()),
        }
    )
    if args.sample_seconds <= 0:
        reporter.add(
            args.case,
            "人工状态辅助检查参数",
            ERROR,
            "--sample-seconds 必须大于 0",
            [
                interface_check(
                    "argument",
                    "--sample-seconds",
                    ">0",
                    args.sample_seconds,
                    False,
                )
            ],
        )
    elif args.case.startswith("MAP-"):
        handle_manual_mapping(reporter, args)
    else:
        handle_manual_localization(reporter, args)
    return reporter.finalize()


def handle_check(args: argparse.Namespace) -> int:
    reporter = Reporter(
        "check", artifact_dir(args, "check"), verbose=args.verbose
    )
    run_check(reporter, args.mode, rate_sample_s=args.rate_sample)
    return reporter.finalize()


def require_mutation_confirmation(
    reporter: Reporter, confirmed: bool, gate_id: str
) -> bool:
    if confirmed:
        reporter.add(
            gate_id,
            "破坏性操作显式确认",
            PASS,
            "已提供 --confirm-mutations",
            [
                interface_check(
                    "argument",
                    "--confirm-mutations",
                    "provided",
                    "provided",
                    True,
                )
            ],
        )
        return True
    reporter.add(
        gate_id,
        "破坏性操作显式确认",
        ERROR,
        "缺少 --confirm-mutations；未执行任何地图/定位变更",
        [
            interface_check(
                "argument",
                "--confirm-mutations",
                "provided",
                "missing",
                False,
            )
        ],
    )
    return False


def handle_localization(args: argparse.Namespace) -> int:
    reporter = Reporter(
        "localization",
        artifact_dir(args, "localization"),
        verbose=args.verbose,
    )
    if not require_mutation_confirmation(
        reporter, args.confirm_mutations, "LOC-G-01"
    ):
        return reporter.finalize()
    graph, detected = run_check(
        reporter, "localization", rate_sample_s=args.rate_sample
    )
    if graph is not None and detected == "localization":
        run_localization_mutations(
            reporter,
            maps_root=args.maps_root.resolve(),
            map_name=args.map_name,
            localization_timeout_s=args.localization_timeout,
            skip_relocalization=args.skip_relocalization,
            relocalization_interval_s=max(
                0.0, args.relocalization_interval
            ),
            pose_random_seed=args.pose_random_seed,
            pose_position_min_m=args.pose_position_min,
            pose_position_max_m=args.pose_position_max,
            pose_yaw_min_deg=args.pose_yaw_min_deg,
            pose_yaw_max_deg=args.pose_yaw_max_deg,
            localization_log=args.localization_log.resolve(),
            localization_log_tag=args.localization_log_tag,
        )
    return reporter.finalize()


def handle_mapping(args: argparse.Namespace) -> int:
    reporter = Reporter(
        "mapping", artifact_dir(args, "mapping"), verbose=args.verbose
    )
    if not require_mutation_confirmation(
        reporter, args.confirm_mutations, "MAP-G-01"
    ):
        return reporter.finalize()
    graph, detected = run_check(
        reporter, "mapping", rate_sample_s=0.0
    )
    if graph is not None and detected == "mapping":
        run_mapping_mutations(
            reporter,
            maps_root=args.maps_root.resolve(),
            result_root=args.result_root.resolve(),
            map_name=args.map_name,
            keep_test_map=args.keep_test_map,
        )
    return reporter.finalize()


def handle_cleanup(args: argparse.Namespace) -> int:
    return remove_managed_mapping_test(
        None, result_root=args.result_root.resolve()
    )


def main() -> int:
    args = build_parser().parse_args()
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
