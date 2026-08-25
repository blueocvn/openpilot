#!/usr/bin/env python3
"""Build a complete Markdown report from Deep Research result JSON files."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import yaml


PROJECT_DIR = Path(__file__).resolve().parent
OUTLINE_PATH = PROJECT_DIR / "outline.yaml"
FIELDS_PATH = PROJECT_DIR / "fields.yaml"

CATEGORY_MAPPING = {
    "Basic Info": ["basic_info", "Basic Info"],
    "Technical Features": ["technical_features", "technical_characteristics", "Technical Features"],
    "Performance Metrics": ["performance_metrics", "performance", "Performance Metrics"],
    "Milestone Significance": ["milestone_significance", "milestones", "Milestone Significance"],
    "Business Info": ["business_info", "commercial_info", "Business Info"],
    "Competition & Ecosystem": ["competition_ecosystem", "competition", "Competition & Ecosystem"],
    "History": ["history", "History"],
    "Market Positioning": ["market_positioning", "market", "Market Positioning"],
}
INTERNAL_FIELDS = {"_source_file", "uncertain"}


def slug(value: str) -> str:
    value = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return value or "item"


def has_uncertainty(value: Any) -> bool:
    if isinstance(value, str):
        return "[uncertain]" in value.lower()
    if isinstance(value, dict):
        return any(has_uncertainty(part) for part in value.values())
    if isinstance(value, list):
        return any(has_uncertainty(part) for part in value)
    return False


def find_value(data: dict[str, Any], field_name: str, category_name: str) -> Any:
    """Find flat or nested field values, including known multilingual categories."""
    if field_name in data:
        return data[field_name]

    category_keys = list(CATEGORY_MAPPING.get(category_name, []))
    category_keys.extend([category_name, category_name.lower().replace(" ", "_")])
    for key in category_keys:
        nested = data.get(key)
        if isinstance(nested, dict) and field_name in nested:
            return nested[field_name]

    def walk(node: Any) -> Any:
        if isinstance(node, dict):
            if field_name in node:
                return node[field_name]
            for child in node.values():
                result = walk(child)
                if result is not None:
                    return result
        elif isinstance(node, list):
            for child in node:
                result = walk(child)
                if result is not None:
                    return result
        return None

    return walk(data)


def format_value(value: Any, depth: int = 0) -> str:
    if isinstance(value, dict):
        lines = []
        for key, child in value.items():
            if key in INTERNAL_FIELDS or has_uncertainty(child):
                continue
            label = key.replace("_", " ").capitalize()
            rendered = format_value(child, depth + 1)
            if rendered:
                lines.append(f"- **{label}:** {rendered}")
        return "\n".join(lines)
    if isinstance(value, list):
        if not value:
            return ""
        if all(not isinstance(item, (dict, list)) for item in value):
            clean = [str(item) for item in value if not has_uncertainty(item)]
            return "; ".join(clean) if len(clean) <= 3 else "\n".join(f"- {item}" for item in clean)
        rendered_items = []
        for item in value:
            if has_uncertainty(item):
                continue
            rendered = format_value(item, depth + 1)
            if rendered:
                rendered_items.append(f"- {rendered}" if not rendered.startswith("-") else rendered)
        return "\n".join(rendered_items)
    if value is None:
        return ""
    text = str(value).strip()
    if not text or has_uncertainty(text):
        return ""
    return text


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def main() -> None:
    outline = load_yaml(OUTLINE_PATH)
    fields_definition = load_yaml(FIELDS_PATH)
    output_dir = Path(outline.get("execution", {}).get("output_dir", PROJECT_DIR / "results"))
    if not output_dir.is_absolute():
        output_dir = (PROJECT_DIR / output_dir).resolve()

    results: dict[str, dict[str, Any]] = {}
    for result_path in sorted(output_dir.glob("*.json")):
        with result_path.open(encoding="utf-8") as file:
            result = json.load(file)
        name = result.get("item_name")
        if isinstance(name, str) and name:
            result["_source_file"] = result_path.name
            results[name] = result

    items = outline.get("items", [])
    complete_count = sum(1 for item in items if item.get("name") in results)
    lines = [
        "# Báo cáo nghiên cứu: openpilot tích hợp CARLA trước tình huống xe cắt ngang",
        "",
        f"**Chủ đề:** {outline.get('topic', '')}",
        "",
        f"**Trạng thái:** {complete_count}/{len(items)} hạng mục đã có kết quả nghiên cứu sâu và qua validator.",
        "",
        "## Tóm tắt điều hành",
        "",
        "Các artifact hiện có cho thấy bridge CARLA hiện chưa có đủ tính hợp lệ để dùng lỗi phanh dọc làm bằng chứng retrain policy: quyền longitudinal đang thuộc bridge-owned stock ACC, không phải openpilot. Cần sửa và xác nhận quyền actuator, đồng bộ thời gian/frame, tần số sensor, chuyển đổi tọa độ và mismatch vehicle model; sau đó chạy lại baseline trước khi đánh giá các ca xe cắt ngang.",
        "",
        "Một kết quả `failed` chỉ được quy cho policy/planning khi scenario và simulator hợp lệ, input/model/perception đúng và đủ sớm, lệnh policy có quyền actuator thực tế, nhưng hành động vẫn vi phạm safety gate lặp lại. Các lỗi bridge, sensor, timing, perception hay control phải được sửa theo tầng tương ứng trước.",
        "",
        "## Mục lục",
        "",
    ]
    for index, item in enumerate(items, start=1):
        name = item.get("name", f"Item {index}")
        status = "Hoàn tất" if name in results else "Chờ nghiên cứu"
        lines.append(f"{index}. [{name}](#{slug(name)}) — {status}")

    categories = fields_definition.get("field_categories", [])
    for index, item in enumerate(items, start=1):
        name = item.get("name", f"Item {index}")
        lines.extend(["", f"## {index}. {name}", ""])
        result = results.get(name)
        if result is None:
            lines.append("*Chưa có JSON kết quả cho hạng mục này; không suy diễn kết luận.*")
            continue

        lines.append(f"Nguồn artifact: `{result.get('_source_file', '')}`")
        uncertain = set(result.get("uncertain", []))
        for category in categories:
            category_name = category.get("category", "Other Info")
            values = []
            for field in category.get("fields", []):
                field_name = field.get("name")
                if not field_name or field_name in uncertain:
                    continue
                value = find_value(result, field_name, category_name)
                if value is None or has_uncertainty(value):
                    continue
                rendered = format_value(value)
                if rendered:
                    values.append((field_name, rendered))
            if values:
                lines.extend(["", f"### {category_name}", ""])
                for field_name, rendered in values:
                    lines.extend([f"#### {field_name}", "", rendered, ""])

        known_fields = {field.get("name") for category in categories for field in category.get("fields", [])}
        extras = {key: value for key, value in result.items() if key not in known_fields | INTERNAL_FIELDS}
        extras = {key: value for key, value in extras.items() if not has_uncertainty(value)}
        if extras:
            lines.extend(["### Other Info", ""])
            for key, value in extras.items():
                rendered = format_value(value)
                if rendered:
                    lines.extend([f"#### {key}", "", rendered, ""])

    report_path = PROJECT_DIR / "report.md"
    report_path.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")
    print(f"Generated {report_path} from {complete_count} result file(s).")


if __name__ == "__main__":
    main()
