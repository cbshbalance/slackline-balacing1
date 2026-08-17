from __future__ import annotations

import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "tmp" / "prior_docs_mining_20260810"
OUT = Path(__file__).resolve().parent / "학생보고서_원문병합본_출처표시.md"
CLEAN_OUT = Path(__file__).resolve().parent / "학생보고서_원문병합본.md"


def load_doc(name: str) -> dict[int, dict]:
    data = json.loads((SRC / name).read_text(encoding="utf-8"))
    return {int(record["index"]): record for record in data["records"]}


def load_student() -> dict[int, str]:
    path = SRC / "latest_student" / "extracted_paragraphs.txt"
    records: dict[int, str] = {}
    current: int | None = None
    chunks: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^\[(\d+)\]\s?(.*)$", line)
        if match:
            if current is not None:
                records[current] = "\n".join(chunks).strip()
            current = int(match.group(1))
            chunks = [match.group(2)]
        elif current is not None:
            chunks.append(line)
    if current is not None:
        records[current] = "\n".join(chunks).strip()
    return records


THEORY = load_doc("theory_v3.json")
REPORT = load_doc("report_v3.json")
SYNTHESIS = load_doc("synthesis_41.json")
STUDENT = load_student()

lines: list[str] = []


def blank() -> None:
    if lines and lines[-1] != "":
        lines.append("")


def heading(level: int, title: str) -> None:
    blank()
    lines.append(f"{'#' * level} {title}")
    lines.append("")


def source_note(source: str, selection: str) -> None:
    lines.append(f"<!-- 원문 출처: {source}, {selection} -->")
    lines.append("")


def emit_text(text: str, style: str = "") -> None:
    text = text.strip()
    if not text:
        return
    if style.startswith("Heading3"):
        heading(5, text)
    elif style.startswith("Heading2"):
        heading(4, text)
    elif style.startswith("Heading1"):
        heading(3, text)
    elif style == "BlockText":
        blank()
        lines.append("> " + text.replace("\n", "\n> "))
        lines.append("")
    else:
        blank()
        lines.append(text)
        lines.append("")


def emit_table(rows: list[list[str]]) -> None:
    if not rows:
        return
    width = max(len(row) for row in rows)
    normalized = [list(row) + [""] * (width - len(row)) for row in rows]
    blank()
    lines.append("| " + " | ".join(normalized[0]) + " |")
    lines.append("| " + " | ".join(["---"] * width) + " |")
    for row in normalized[1:]:
        lines.append("| " + " | ".join(str(cell) for cell in row) + " |")
    lines.append("")


def emit_range(
    records: dict[int, dict],
    start: int,
    end: int,
    *,
    source: str,
    skip: set[int] | None = None,
) -> None:
    skip = skip or set()
    source_note(source, f"문단 {start}–{end}")
    for index in sorted(records):
        if not (start <= index <= end) or index in skip:
            continue
        record = records[index]
        text = str(record.get("text", "")).strip()
        if not text:
            continue
        if record.get("kind") == "tbl" and record.get("rows"):
            emit_table([[str(cell) for cell in row] for row in record["rows"]])
        else:
            emit_text(text, str(record.get("style", "")))


def emit_student(index: int) -> None:
    text = STUDENT.get(index, "").strip()
    if text:
        emit_text(text)


heading(1, "한국 줄타기는 어떻게 장대 없이 균형을 잡는가?")
heading(2, "처진 줄 진자 위 이중 역진자가 허리 접기 하나로 서는 원리의 규명과 로봇 실증")
lines.append("> 세 DOCX와 학생 최신본의 원문 문단을 새 목차에 맞춰 배열한 병합 초안이다. 문장 재작성 없이 중복되는 설명만 선택하여 제외하였다.")
lines.append("")

heading(2, "Ⅰ. 탐구 동기 및 목적")
heading(3, "1. 탐구 동기")
heading(4, "가. 문제 의식 — 서양 줄타기와 한국 줄타기")
source_note("학생 최신 보고서", "문단 122–128")
for idx in (122, 123, 124, 125, 127, 128):
    emit_student(idx)

heading(4, "나. 얼음판 위의 사고실험 — 이건 불가능한 일이다")
emit_range(THEORY, 17, 22, source="이론부 초안 v3")

heading(4, "다. 역발상")
source_note("학생 최신 보고서", "문단 130–132")
for idx in range(130, 133):
    emit_student(idx)

source_note("선생님 동기 메모", "‘몸을 세우는 대신 줄을 끌어오면 된다’")
emit_text(
    "무게중심을 줄 위에 있게 하는 것이 균형이라면, 꼭 무게중심을 움직여야 할까? "
    "몸을 세우는 대신 느슨한 줄을 내 밑으로 끌어오면 되지 않을까? "
    "그렇다면 한국 줄타기의 줄은 균형을 어렵게 하면서도, 동시에 줄과 발을 움직일 수 있게 하려고 느슨해야 했던 것은 아닐까?"
)
emit_text("“몸을 세우는 대신 줄을 끌어오면 된다.”", "BlockText")

heading(3, "2. 탐구 목적")
heading(4, STUDENT[134])
source_note("학생 최신 보고서", "문단 135")
emit_student(135)
heading(4, STUDENT[136])
source_note("학생 최신 보고서", "문단 137")
emit_student(137)
heading(4, STUDENT[138])
source_note("학생 최신 보고서", "문단 139")
emit_student(139)

heading(2, "Ⅱ. 탐구 과정")
heading(3, "1. 문헌 조사 — 찾은 것은 답이 아니라 공백이었다")
emit_range(THEORY, 24, 30, source="이론부 초안 v3")

heading(3, "2. 첫 번째 열쇠 — 처진 줄은 받침이 아니라 변환기다")
emit_range(THEORY, 33, 69, source="이론부 초안 v3")

heading(3, "3. 이야기에 필요한 세 개의 도구")
emit_range(THEORY, 73, 83, source="이론부 초안 v3")

heading(3, "4. 답을 지어내다 — FWE 메커니즘의 제안")
emit_range(THEORY, 88, 116, source="이론부 초안 v3")

heading(3, "5. 균형이란 무엇인가 — 후보 소거와 모드의 발견")
emit_range(THEORY, 118, 135, source="이론부 초안 v3")
emit_range(SYNTHESIS, 38, 58, source="종합 41")

heading(3, "6. 분리가정의 실패와 닫힌 공식 ε*")
emit_range(THEORY, 153, 216, source="이론부 초안 v3", skip={173})

heading(3, "7. 공식의 위계 — 현실을 향한 네 번의 확장")
emit_range(THEORY, 220, 285, source="이론부 초안 v3")

heading(3, "8. 그림자와 실체 — 예측점 평면")
emit_range(SYNTHESIS, 89, 126, source="종합 41")

heading(3, "9. 메커니즘의 진화 — FWE에서 FFFF까지")
emit_range(SYNTHESIS, 130, 197, source="종합 41")

heading(3, "10. 시뮬레이션의 발전")
emit_range(REPORT, 128, 129, source="보고서 3차수정안")

heading(3, "11. 줄타기 로봇 ‘어름이’ — 실물 부분 검증")
emit_range(REPORT, 137, 137, source="보고서 3차수정안")

heading(2, "Ⅲ. 결론 및 향후 탐구 계획")
emit_range(REPORT, 144, 150, source="보고서 3차수정안")

heading(2, "Ⅳ. 참고 문헌")
emit_range(REPORT, 155, 158, source="보고서 3차수정안")

merged = "\n".join(lines).rstrip() + "\n"
OUT.write_text(merged, encoding="utf-8")
clean = re.sub(r"\n?<!-- 원문 출처:.*?-->\n?", "\n", merged)
clean = re.sub(r"\n{3,}", "\n\n", clean)
CLEAN_OUT.write_text(clean, encoding="utf-8")
print(OUT)
print(CLEAN_OUT)
