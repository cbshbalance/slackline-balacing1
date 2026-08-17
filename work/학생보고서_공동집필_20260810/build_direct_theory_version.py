from __future__ import annotations

import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WORK = Path(__file__).resolve().parent
SRC = ROOT / "tmp" / "prior_docs_mining_20260810"
STUDENT_SRC = ROOT / "tmp" / "latest_student_report_20260810" / "extracted" / "extracted_paragraphs.txt"
BASE = WORK / "학생보고서_원문병합본.md"
TRACE_OUT = WORK / "학생보고서_직선서술판_출처표시.md"
CLEAN_OUT = WORK / "학생보고서_직선서술판.md"


def load_doc(name: str) -> dict[int, dict]:
    data = json.loads((SRC / name).read_text(encoding="utf-8"))
    return {int(record["index"]): record for record in data["records"]}


def load_student_report() -> dict[int, dict]:
    records: dict[int, dict] = {}
    for line in STUDENT_SRC.read_text(encoding="utf-8").splitlines():
        match = re.match(r"\[(\d+)]\s*(.*)", line)
        if match:
            records[int(match.group(1))] = {"text": match.group(2), "style": ""}
    return records


REPORT = load_doc("report_v3.json")
SYNTHESIS = load_doc("synthesis_41.json")
STUDENT = load_student_report()

base_text = BASE.read_text(encoding="utf-8")
cut_marker = "\n### 3. 이야기에 필요한 세 개의 도구"
if cut_marker not in base_text:
    raise RuntimeError("기존 병합본에서 교체 시작점을 찾지 못했습니다.")

lines = base_text.split(cut_marker, 1)[0].rstrip().splitlines()


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


heading(3, "3. 발을 무게중심 너머로 보내 쓰러짐 방향을 바꾼다 — FWE 메커니즘")
emit_range(REPORT, 60, 60, source="보고서 3차수정안")
emit_text(
    "그러나 ‘얼마나 접고 얼마나 기다릴 것인가’를 계산하려면, 그보다 먼저 무엇을 균형이라 부를 것인지부터 "
    "정해야 했다. 흔들림을 완전히 멈추는 것만이 균형이라면, 역동적으로 줄을 타는 광대의 모습은 끝내 설명할 수 없기 때문이다."
)

heading(3, "4. 완전히 정지할 필요 없이 떨어지지만 않으면 된다 — 한국 줄타기에 맞는 균형의 정의")
emit_range(SYNTHESIS, 14, 23, source="종합 41", skip={20, 21, 22})
heading(4, "가. 입력 없이도 떨어지지 않고 흔들리는 상태가 가능한가")
emit_range(SYNTHESIS, 27, 30, source="종합 41")
emit_text(
    "사고실험은 줄과 몸이 엇갈린 채 어떤 비율로 함께 움직여야 한다는 데까지 도달했다. 우리가 시간에 따라 "
    "달라져야 한다고 생각했던 그 비율이 실제로 일정하게 유지될 수 있는지 확인하려면, 줄과 몸의 운동을 따로 보지 않고 "
    "하나의 결합된 운동으로 살펴보아야 했다."
)

heading(3, "5. 흔들림 속에서 균형의 길을 찾다 — 결합 모드와 안정모드선")
emit_range(SYNTHESIS, 39, 39, source="종합 41")
emit_text(
    "우리가 놓친 것은 관성력이 약해질 때 중력 토크도 정확히 같은 박자로 약해진다는 사실이었다. 줄의 각도 φ와 "
    "몸의 기울기 β가 같은 진동을 공유한다면 두 효과가 함께 작아지므로, 처음 정해진 비율은 시간이 지나도 깨지지 않는다."
)
emit_range(SYNTHESIS, 44, 57, source="종합 41", skip={50, 52, 55, 56})
emit_text(
    "모드 분석으로 균형이 놓이는 자리는 안정모드선이라는 한 줄로 정해졌다. 이제 남은 문제는 현재의 상태를 "
    "그 선 위로 어떻게 옮길 것인가이다. 허리 접기가 만드는 상태 변화와 안정모드선의 만남을 상태공간의 기하로 살펴보았다."
)

heading(3, "6. 기운 크기가 달라도 같은 비율로 — 상태공간의 기하와 최적 접기 비율 ε*")
emit_range(SYNTHESIS, 66, 82, source="종합 41", skip={72, 81, 82})
emit_range(REPORT, 94, 96, source="보고서 3차수정안")
emit_text(
    "ε*는 정지한 채 기울어진 순간에는 완전한 답이었다. 그러나 실제 줄 위의 몸은 이미 움직이고 있다. 같은 위치에 있어도 "
    "속도가 다르면 한순간 뒤의 운명도 달라지므로, 위치와 속도를 함께 읽을 새로운 평면이 필요했다."
)

heading(3, "7. 위치와 속도를 하나의 점에 담다 — 예측점 평면과 차원 축소")
emit_range(SYNTHESIS, 90, 104, source="종합 41", skip={93, 103})
emit_range(SYNTHESIS, 120, 122, source="종합 41")
emit_text(
    "예측점 평면은 한 번의 접기만으로 발산을 없애는 조건을 보여 주었다. 하지만 광대는 접은 채로 줄을 타지 않는다. "
    "다시 몸을 펴고도 균형을 잃지 않으려면, 접기에서 시작한 이야기를 기다리기와 펴기까지 이어 하나의 사이클로 닫아야 한다."
)

heading(3, "8. 접고, 기다리고, 펴는 박자를 닫다 — 순간동작 가정의 완전 FWE 사이클")
emit_text(
    "펴기까지 포함한 완전 사이클에서는 ε*보다 더 접어 반대 부호의 불안정 성분을 남기고, "
    "선택한 대기시간 뒤 펴기의 킥과 상쇄되도록 한다. 종합 41의 닫힌형은 다음과 같다."
)
emit_range(SYNTHESIS, 137, 139, source="종합 41")
emit_text(
    "이로써 순간적으로 접고 펼 수 있다는 가정 아래에서는 ‘얼마나 접고, 얼마나 기다리고, 언제 펼 것인가’에 대한 "
    "이론적 답이 닫혔다. 그러나 실제 모터는 순간적으로 움직이지 않으며, 한 번의 완벽한 동작을 언제나 정확히 재현할 수도 없다. "
    "이제부터는 원리를 더 일반화하는 문제가 아니라, 완성된 원리를 유한한 속도와 실행오차가 있는 현실에 적용하는 문제다."
)

heading(3, "9. 완벽한 한 번보다 불완전한 여러 번 — 유한시간 접기와 증분 접기의 시뮬레이션")

heading(4, "가. 실제 동작은 순간적이지 않다 — 유한시간 접기")
source_note("최신 학생 보고서", "문단 356을 간결하게 정리")
emit_text(
    "이론의 접기는 순간이지만 실제 서보가 몸을 접는 데에는 시간이 걸린다. 그 시간 T_f 동안에도 불안정 모드는 "
    "계속 자라므로, 유한시간 접기는 순간접기보다 항상 더 많이 접어야 한다. 이 차이를 보정하는 계수가 γ이다. "
    "T_f가 0에 가까워지면 γ도 1로 수렴하여 순간접기 공식이 그대로 복원되고, 사람 스케일에서 T_f≈0.25초일 때는 "
    "순간접기보다 약 53% 더 접어야 했다. 상태공간에서 직선이던 순간접기의 점프는 유한시간 동안 곡선이 되며, "
    "γ는 그 사이에 불안정 모드가 자란 만큼을 접기량으로 미리 갚는 보정이다."
)

heading(4, "나. 완전 사이클의 약점 — 기다리는 동안 오차도 자란다")
emit_range(SYNTHESIS, 149, 150, source="종합 41")
emit_text(
    "따라서 완전 FWE 사이클은 원리를 가장 선명하게 보여 주지만, 실물 제어기로는 정확한 접기량과 대기시간, "
    "펴기 시점을 한 번에 모두 맞춰야 하는 민감한 전략이었다. 작은 실행오차를 다음 측정에서 다시 고칠 수 있는 "
    "폐루프 방식이 필요했다."
)

heading(4, "다. 펴기를 균형의 필수 동작에서 제외하다 — 증분 접기")
source_note("최신 학생 보고서", "문단 363·365와 종합 41 문단 190–193을 정리")
emit_text(
    "해법은 가장 처음의 단일접기 정리 안에 있었다. ε*로 접어 불안정 성분을 없애면, 다시 펴지 않아도 상태는 "
    "안정모드선 위에서 유계로 진동한다. 그렇다면 펴기는 균형을 만들기 위한 필수 동작이 아니라, 접힌 자세를 "
    "회복하기 위한 편의 동작이다. 이 관점에서 한 번의 큰 완전 사이클을 버리고, 현재의 불안정도가 다시 자란 만큼만 "
    "조금씩 더 접는 증분 접기를 구성하였다."
)
emit_table(
    [
        ["단계", "증분 접기의 동작"],
        ["1. 위험도 계산", "현재 상태의 예측점을 구하고 안정모드선에서 벗어난 불안정 성분 A를 계산한다."],
        ["2. 사건 판단", "|A|가 정한 문턱을 넘을 때만 개입하고, 문턱 아래에서는 기다린다."],
        ["3. 증분 접기", "A의 부호와 크기에 따라 현재 자세에서 필요한 만큼만 허리를 조금 더 접는다."],
        ["4. 저속 복귀", "위험이 작을 때에는 불안정도를 계속 확인하며 허리를 천천히 원래 자세로 되돌린다."],
    ]
)
emit_text(
    "이 제어에는 접기와 펴기의 엄격한 순서가 없다. 문턱을 넘을 때의 작은 접기가 F의 역할을 하고, 개입하지 않는 "
    "시간이 W가 되며, 위험이 작을 때의 느린 자세 복귀가 E가 된다. 크게 기울면 크게 접고, 이후 미세조정을 거듭하다가 "
    "한가해지면 천천히 몸을 세우는 모습은 외란을 맞은 광대의 회복 동작과도 닮아 있다. 증분 접기는 FWE 이론을 "
    "폐기한 새 제어가 아니라, ‘한 번 접으면 펴지 않아도 유계’라는 이론을 실행오차가 있는 폐루프 제어로 옮긴 형태다."
)

heading(4, "라. 물리 엔진 시뮬레이션을 통한 적용 가능성 검증")
emit_range(REPORT, 128, 129, source="보고서 3차수정안")
source_note("최신 학생 보고서", "문단 367 — 시뮬레이션 결과의 층위를 명시")
emit_text(
    "실측 물리량과 줄 마찰, 서보와 센서의 한계를 반영한 전물리 시뮬레이션에서 증분 접기 이전의 FWE 메커니즘은 "
    "평균 약 90초, 최대 약 120초 동안 생존하였다. 증분 접기를 적용하자 시험한 20개의 초기조건이 모두 시험 상한인 "
    "120초까지 생존하였다. 이어 생존시간 상한을 없앤 후속 시뮬레이션에서는 27,000초까지 균형을 유지하였다. "
    "정해진 시간이 지나 낙하한 것이 아니라 계산을 종료할 때까지 균형이 계속되었으므로, 전물리 시뮬레이션 안에서는 "
    "사실상 영구적으로 생존할 수 있다는 결과를 얻었다. 이는 실물 로봇의 성공 기록이 아니라, 불완전한 실행을 반복 "
    "측정과 작은 보정으로 견디는 증분 접기 전략이 전물리 시뮬레이션에서 작동했음을 보인 결과다."
)
emit_text(
    "시뮬레이션은 이론의 인과 사슬이 유한한 속도와 실행오차 속에서도 이어질 수 있음을 보여 주었다. 그러나 마찰과 "
    "유격, 센서 오차가 매번 다르게 나타나는 현실의 줄은 또 다른 시험대다. 그래서 마지막으로 증분 접기를 실제 로봇의 몸에 옮겼다."
)

heading(3, "10. 이론을 현실의 줄 위에 세우다 — 줄타기 로봇 ‘어름이’")
emit_range(REPORT, 137, 137, source="보고서 3차수정안")
emit_text(
    "로봇의 균형 실증은 아직 진행 중이지만, 그 성패와 별개로 한국 줄타기의 불가능해 보이던 장면에서 출발한 질문은 "
    "균형의 재정의, 결합 모드, 최적 접기 비율과 예측점 평면으로 이어지는 하나의 물리적 설명에 도달했다."
)

heading(2, "Ⅲ. 탐구 결론 및 의의")
heading(3, "1. 탐구 결론")
heading(4, "가. 처진 줄과 허리 접기가 만드는 균형 회복 원리")
emit_range(STUDENT, 400, 401, source="최신 학생 보고서 결론")
heading(4, "나. 자연이 가진 균형의 리듬을 찾아 들어가는 기술")
emit_range(STUDENT, 402, 402, source="최신 학생 보고서 결론")

heading(3, "2. 탐구 의의")
heading(4, "가. 한국 줄타기의 균형 원리를 검증 가능한 물리 가설로 제시하였다")
emit_range(STUDENT, 405, 405, source="최신 학생 보고서 결론")
heading(4, "나. 답뿐 아니라 질문과 판단과 검증의 책임까지 감당하였다")
source_note("최신 학생 보고서 결론", "문단 407 — 검증 수치 표현은 본문과 일치하도록 정리")
emit_text(
    "선행 연구가 없는 문제에서 분리 가설을 스스로 세우고, 질량 행렬의 비대각 성분을 근거로 스스로 기각하며 "
    "결합 모델에 도달하였다. LQR로 균형을 잡고도 ‘이것은 설명이 아니다’라고 판단하여 우리만의 메커니즘을 세웠으며, "
    "균형의 정의를 후보 소거로 직접 재정의하는 과정에서 모드 해석이라는 도구의 필요성을 추론으로부터 이끌어냈다. "
    "메커니즘은 여러 차례 모습을 바꾸었지만 매 전환은 문제가 강제한 것이었다. 유도한 공식은 수식과 물리 엔진 "
    "시뮬레이션으로 교차 검증하였고, 실물 로봇에서는 폐루프 접기 반응과 초 단위 생존까지의 부분 실증을 확인하였다. "
    "이론적 증명과 시뮬레이션 검증, 실물 실증의 층위를 구분하여 서술했으며, 답뿐 아니라 질문과 판단과 검증의 책임까지 "
    "스스로 감당한 것에 이 탐구의 방법론적 의의가 있다."
)

heading(3, "3. 한계 및 향후 탐구 계획")
emit_range(REPORT, 150, 150, source="보고서 3차수정안")

heading(2, "Ⅳ. 참고 문헌")
emit_range(REPORT, 155, 158, source="보고서 3차수정안")

merged = "\n".join(lines).rstrip() + "\n"
TRACE_OUT.write_text(merged, encoding="utf-8")
clean = re.sub(r"\n?<!-- 원문 출처:.*?-->\n?", "\n", merged)
clean = re.sub(r"\n{3,}", "\n\n", clean)
CLEAN_OUT.write_text(clean, encoding="utf-8")
print(TRACE_OUT)
print(CLEAN_OUT)
