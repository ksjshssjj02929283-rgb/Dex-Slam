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
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable


DEFAULT_MAPS_ROOT = Path(
    os.environ.get("SLAM_2D_MAPS_ROOT", "/home/dexforce/.dexe/data/maps")
)
DEFAULT_RESULT_ROOT = Path(
    os.environ.get(
        "SLAM_2D_TEST_RESULT_ROOT",
        str(Path.home() / ".dexe/navigation/slam_2d_interface_test"),
    )
)
ALLOWED_TEST_PREFIXES = ("slam2d_test_", "mm_test_", "test_")
PASS = "PASS"
FAIL = "FAIL"
ERROR = "ERROR"
PENDING_ENV = "PENDING_ENV"


@dataclass
class CommandRecord:
    command: str
    returncode: int
    duration_s: float
    stdout: str
    stderr: str
    timed_out: bool = False


@dataclass
class TestResult:
    tc: str
    name: str
    status: str
    detail: str


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
    def __init__(self, command: str, artifact_dir: Path) -> None:
        self.command = command
        self.artifact_dir = artifact_dir
        self.started_at = now_iso()
        self.results: list[TestResult] = []
        self.commands: list[CommandRecord] = []
        self.metadata: dict[str, Any] = {}
        artifact_dir.mkdir(parents=True, exist_ok=False)

    def add(self, tc: str, name: str, status: str, detail: str) -> None:
        self.results.append(TestResult(tc, name, status, detail))
        symbol = {PASS: "✓", FAIL: "✗", ERROR: "!", PENDING_ENV: "…"}.get(
            status, "-"
        )
        print(f"[{status:<11}] {symbol} {tc} {name}: {detail}")

    def run(
        self,
        args: Iterable[str],
        *,
        timeout_s: float = 15.0,
    ) -> CommandRecord:
        command = [str(value) for value in args]
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
        )
        self.commands.append(record)
        return record

    def run_for(self, args: Iterable[str], duration_s: float) -> CommandRecord:
        command = [str(value) for value in args]
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
        )
        self.commands.append(record)
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
            "schema_version": 2,
            "command": self.command,
            "started_at": self.started_at,
            "finished_at": now_iso(),
            "overall": overall,
            "counts": counts,
            "metadata": self.metadata,
            "tests": [asdict(result) for result in self.results],
            "commands": [asdict(command) for command in self.commands],
        }
        atomic_json(self.artifact_dir / "summary.json", payload)
        atomic_json(
            self.artifact_dir / "commands.json",
            {"commands": payload["commands"]},
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
            "| 测试项 ID | 测试项 | 状态 | 详情 |",
            "|---|---|:---:|---|",
        ]
        for result in self.results:
            detail = result.detail.replace("|", "\\|").replace("\n", " ")
            lines.append(
                f"| {result.tc} | {result.name} | {result.status} | {detail} |"
            )
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
        )
        return None
    return snapshot


def has_node(nodes: set[str], name: str) -> bool:
    return any(node == f"/{name}" or node.endswith(f"/{name}") for node in nodes)


def missing_nodes(nodes: set[str], required: Iterable[str]) -> list[str]:
    return [name for name in required if not has_node(nodes, name)]


def missing_names(existing: set[str], required: Iterable[str]) -> list[str]:
    return sorted(set(required) - existing)


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


def bool_success(text: str) -> bool:
    return bool(
        re.search(r"\bsuccess\s*(?:=|:)\s*(?:True|true)\b", text)
    )


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


def read_localization_status(
    reporter: Reporter, timeout_s: float = 6.0
) -> int | None:
    result = read_topic_once(
        reporter,
        "/localization_node/state",
        "nav_interfaces/msg/LocalizationState",
        timeout_s=timeout_s,
    )
    value = parse_scalar(f"{result.stdout}\n{result.stderr}", "status")
    try:
        return int(value)
    except ValueError:
        return None


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
    present = sorted(
        (forbidden_services & graph.services)
        | (forbidden_topics & graph.topics)
    )
    present.extend(
        name for name in forbidden_nodes if has_node(graph.nodes, name)
    )
    reporter.add(
        test_id,
        "旧接口与旧节点域无残留",
        PASS if not present else FAIL,
        "无残留" if not present else f"仍存在：{', '.join(present)}",
    )


def check_mapping_contract(
    reporter: Reporter, graph: GraphSnapshot
) -> None:
    required_services = {
        "/mapping_node/start_mapping",
        "/mapping_node/cancel_mapping",
        "/mapping_node/finish_mapping",
        "/mapping_node/save_map",
        "/mapping_node/set_map_name",
    }
    required_topics = {"/mapping_node/state"}
    nodes_missing = missing_nodes(graph.nodes, ("mapping_node",))
    services_missing = missing_names(graph.services, required_services)
    topics_missing = missing_names(graph.topics, required_topics)
    detail_parts = []
    if nodes_missing:
        detail_parts.append(f"缺节点 {nodes_missing}")
    if services_missing:
        detail_parts.append(f"缺服务 {services_missing}")
    if topics_missing:
        detail_parts.append(f"缺话题 {topics_missing}")
    reporter.add(
        "MAP-A-02",
        "mapping_node 接口契约",
        PASS if not detail_parts else FAIL,
        "节点、5 个控制服务和 state 话题齐全"
        if not detail_parts
        else "；".join(detail_parts),
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
    required_services = {
        "/map_manager/load_map",
        "/map_manager/get_current_map_info",
        "/map_manager/get_map_list",
        "/localization_node/start_localization",
        "/localization_node/stop_localization",
        "/localization_node/reset_localization",
        "/localization_node/save_map",
        "/localization_node/reset_map_origin",
        "/localization_node/relocalization",
        "/localization_node/trigger_global_relocalization",
        "/map_asset_manager/save_assets",
        "/map_asset_manager/reload_data",
        "/map_asset_manager/transform_assets",
        "/robot_localization/set_pose",
        "/robot_localization/start_filter",
        "/robot_localization/stop_filter",
    }
    required_topics = {
        "/map_manager/map_state",
        "/localization_node/state",
        "/robot_localization/odometry/filtered",
        "/map",
    }
    detail_parts = []
    nodes_missing = missing_nodes(graph.nodes, required_nodes)
    services_missing = missing_names(graph.services, required_services)
    topics_missing = missing_names(graph.topics, required_topics)
    if nodes_missing:
        detail_parts.append(f"缺节点 {nodes_missing}")
    if services_missing:
        detail_parts.append(f"缺服务 {services_missing}")
    if topics_missing:
        detail_parts.append(f"缺话题 {topics_missing}")
    reporter.add(
        "LOC-A-02",
        "定位栈节点、服务与话题契约",
        PASS if not detail_parts else FAIL,
        "4 节点、16 个关键服务和 4 个关键话题齐全"
        if not detail_parts
        else "；".join(detail_parts),
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
    )
    return state


def check_localization_state(reporter: Reporter) -> None:
    status = read_localization_status(reporter, timeout_s=10)
    passed = status in {-2, -1, 0, 1, 2, 3, 4}
    reporter.add(
        "LOC-A-05",
        "LocalizationState 当前值可读且枚举合法",
        PASS if passed else FAIL,
        f"当前 status={status}"
        if passed
        else (
            f"收到未定义的 status={status}"
            if status is not None
            else "10 秒内未收到 /localization_node/state"
        ),
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
    covariance_result = reporter.run(
        [
            "ros2",
            "param",
            "get",
            "/robot_localization",
            "odom0_pose_covariance",
        ]
    )
    frequency_numbers = parse_numbers(frequency_result.stdout)
    covariance_numbers = parse_numbers(covariance_result.stdout)
    frequency = frequency_numbers[-1] if frequency_numbers else None
    covariance = covariance_numbers[-9:] if len(covariance_numbers) >= 9 else []
    passed = (
        frequency is not None
        and abs(frequency - 25.0) <= 1e-6
        and len(covariance) == 9
        and abs(covariance[-1] - 0.0012) <= 1e-9
    )
    reporter.add(
        "LOC-A-06",
        "UKF 频率与 yaw covariance 参数",
        PASS if passed else FAIL,
        f"frequency={frequency} covariance_count={len(covariance)} "
        f"yaw={covariance[-1] if covariance else None}",
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
    pid_result = reporter.run(["pgrep", "-n", "-x", "ukf_node"])
    pid = pid_result.stdout.strip()
    if not pid.isdigit():
        reporter.add(
            "LOC-A-08",
            "UKF 实际调度策略与 CPU 亲和性",
            FAIL,
            "未找到 ukf_node 进程",
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
    passed = policy_ok and priority == 99 and affinity == {3, 4}
    reporter.add(
        "LOC-A-08",
        "UKF 实际调度策略与 CPU 亲和性",
        PASS if passed else FAIL,
        f"pid={pid} policy={'SCHED_FIFO' if policy_ok else 'other'} "
        f"priority={priority} cpus={sorted(affinity)}",
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
        )
        reporter.metadata["detected_mode"] = "invalid"
        return graph, "invalid"
    reporter.add(
        mode_id,
        "ROS2 图与运行模式唯一",
        PASS,
        f"检测到唯一业务栈：{detected}",
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


def allowed_test_map(name: str) -> bool:
    return bool(re.fullmatch(r"[A-Za-z0-9_.-]+", name)) and name.startswith(
        ALLOWED_TEST_PREFIXES
    )


def run_localization_mutations(
    reporter: Reporter,
    *,
    maps_root: Path,
    map_name: str,
    localization_timeout_s: float,
    skip_relocalization: bool,
) -> None:
    if not allowed_test_map(map_name):
        reporter.add(
            "LOC-G-02",
            "测试地图安全门",
            ERROR,
            "map-name 必须以 slam2d_test_、mm_test_ 或 test_ 开头，且只能含安全字符",
        )
        return
    original = read_map_state(reporter)
    if not original or not original["map_name"]:
        reporter.add(
            "LOC-G-02",
            "测试地图安全门",
            PENDING_ENV,
            "无法记录当前生产地图，拒绝执行会改变地图的测试",
        )
        return
    reporter.add(
        "LOC-G-02",
        "测试地图与原地图记录安全门",
        PASS,
        f"测试图={map_name}；原地图={original['map_name']}",
    )
    reporter.metadata["original_map_state"] = original
    restored = original["map_name"] == map_name
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
        )
        if not success:
            return

        if not skip_relocalization:
            relocalize = call_service(
                reporter,
                "/localization_node/trigger_global_relocalization",
                "std_srvs/srv/Trigger",
                "{}",
            )
            success, statuses = wait_localization(
                reporter,
                timeout_s=localization_timeout_s,
                require_relocalizing=True,
            )
            reporter.add(
                "LOC-A-11",
                "全局重定位 3→4 状态闭环",
                PASS if bool_success(relocalize.stdout) and success else FAIL,
                f"service_success={bool_success(relocalize.stdout)} "
                f"状态序列={statuses[-20:]}",
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
        )
    finally:
        if original["map_name"] != map_name:
            restore_result = call_service(
                reporter,
                "/map_manager/load_map",
                "nav_interfaces/srv/LoadMap",
                f"{{map_name: '{original['map_name']}'}}",
            )
            restored_state = wait_map_state(
                reporter,
                min_epoch=original["epoch"] + 1,
                map_name=original["map_name"],
                timeout_s=20,
            )
            restored = bool(bool_success(restore_result.stdout) and restored_state)
        reporter.add(
            "LOC-G-03",
            "测试结束恢复原地图",
            PASS if restored else FAIL,
            f"目标={original['map_name']}",
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
            reporter.add("MAP-G-03", "受管测试地图处置", ERROR, str(exc))
        else:
            print(f"cleanup failed: {exc}", file=sys.stderr)
        return 1
    if reporter:
        reporter.add("MAP-G-03", "受管测试地图处置", PASS, str(target))
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
        )
        return
    if initial_state[0] != 0:
        reporter.add(
            "MAP-G-02",
            "建图变更前状态门",
            FAIL,
            f"当前 status={initial_state[0]} map_name={initial_state[1]!r}，"
            "要求 IDLE(0)；未创建测试目录、未调用控制服务",
        )
        return
    reporter.add(
        "MAP-G-02",
        "建图变更前状态门",
        PASS,
        f"当前 IDLE(0)，map_name={initial_state[1]!r}",
    )
    try:
        target = prepare_mapping_sentinels(maps_root, map_name, result_root)
        prepared = True
        set_name = call_service(
            reporter,
            "/mapping_node/set_map_name",
            "nav_interfaces/srv/SetMapName",
            f"{{map_name: '{map_name}'}}",
        )
        set_name_ok = bool_success(set_name.stdout)
        if not set_name_ok:
            reporter.add(
                "MAP-A-04",
                "set_map_name → start_mapping → MAPPING",
                FAIL,
                "set_map_name 失败；按 fail-closed 未调用 start_mapping",
            )
            return
        start = call_service(
            reporter,
            "/mapping_node/start_mapping",
            "std_srvs/srv/Trigger",
            "{}",
        )
        started = bool_success(start.stdout)
        reached_mapping, statuses = wait_mapping_status(
            reporter, wanted=1, timeout_s=20
        )
        reporter.add(
            "MAP-A-04",
            "set_map_name → start_mapping → MAPPING",
            PASS
            if set_name_ok and started and reached_mapping
            else FAIL,
            f"set={set_name_ok} start={started} "
            f"状态序列={statuses}",
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
        )
    except Exception as exc:
        reporter.add(
            "MAP-A-04", "受管建图测试准备/执行", ERROR, str(exc)
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
            )
        elif prepared:
            remove_managed_mapping_test(reporter, result_root=result_root)


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "SLAM 2D interface checks with JSON/Markdown evidence. "
            "The tool never switches systemd services."
        )
    )
    parser.add_argument(
        "--result-root", type=Path, default=DEFAULT_RESULT_ROOT
    )
    parser.add_argument("--artifact-dir", type=Path)
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_tests = subparsers.add_parser(
        "list-tests", help="print the automated test catalog"
    )
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
    list_tests.set_defaults(handler=handle_list_tests)

    check = subparsers.add_parser(
        "check", help="read-only interface and runtime checks"
    )
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
    localization.add_argument(
        "--maps-root", type=Path, default=DEFAULT_MAPS_ROOT
    )
    localization.add_argument("--map-name", required=True)
    localization.add_argument("--confirm-mutations", action="store_true")
    localization.add_argument("--skip-relocalization", action="store_true")
    localization.add_argument("--localization-timeout", type=float, default=120.0)
    localization.add_argument("--rate-sample", type=float, default=8.0)
    localization.set_defaults(handler=handle_localization)

    mapping = subparsers.add_parser(
        "mapping",
        help="run mapping checks plus managed same-name reset test",
    )
    mapping.add_argument("--maps-root", type=Path, default=DEFAULT_MAPS_ROOT)
    mapping.add_argument("--map-name", default=f"slam2d_test_{timestamp()}")
    mapping.add_argument("--confirm-mutations", action="store_true")
    mapping.add_argument("--keep-test-map", action="store_true")
    mapping.set_defaults(handler=handle_mapping)

    cleanup = subparsers.add_parser(
        "cleanup", help="remove only the mapping test recorded in managed state"
    )
    cleanup.set_defaults(handler=handle_cleanup)
    return parser


TEST_CATALOG = {
    "mapping": (
        ("MAP-A-01", "ROS2 图与建图模式唯一"),
        ("MAP-A-02", "mapping_node 节点/服务/话题契约"),
        ("MAP-A-03", "建图模式旧接口与旧节点域无残留"),
        ("MAP-A-04", "受管测试图 set_map_name/start_mapping"),
        ("MAP-A-05", "同名建图全目录重建"),
        ("MAP-A-06", "cancel_mapping 后回 IDLE"),
    ),
    "localization": (
        ("LOC-A-01", "ROS2 图与定位模式唯一"),
        ("LOC-A-02", "定位栈节点/服务/话题契约"),
        ("LOC-A-03", "定位模式旧接口与旧节点域无残留"),
        ("LOC-A-04", "MapState transient-local 原子状态"),
        ("LOC-A-05", "LocalizationState 当前值可读且枚举合法"),
        ("LOC-A-06", "UKF 25 Hz 与 yaw covariance"),
        ("LOC-A-07", "EKF Odometry 23~27 Hz"),
        ("LOC-A-08", "UKF SCHED_FIFO 99 / CPU 3,4"),
        ("LOC-A-09", "测试图 load_map 后 epoch 精确 +1"),
        ("LOC-A-10", "地图加载后定位恢复"),
        ("LOC-A-11", "全局重定位 3→4"),
        ("LOC-A-12", "localization save_map 路径与版本"),
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
    return 0


def handle_check(args: argparse.Namespace) -> int:
    reporter = Reporter("check", artifact_dir(args, "check"))
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
        )
        return True
    reporter.add(
        gate_id,
        "破坏性操作显式确认",
        ERROR,
        "缺少 --confirm-mutations；未执行任何地图/定位变更",
    )
    return False


def handle_localization(args: argparse.Namespace) -> int:
    reporter = Reporter("localization", artifact_dir(args, "localization"))
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
        )
    return reporter.finalize()


def handle_mapping(args: argparse.Namespace) -> int:
    reporter = Reporter("mapping", artifact_dir(args, "mapping"))
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
