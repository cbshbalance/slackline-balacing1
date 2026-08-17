from __future__ import annotations

import copy
import zipfile
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "1167_작품설명서_캡션작업본.hwpx"
OUTPUT = ROOT / "1167_작품설명서_그림캡션자동번호.hwpx"
SECTION_NAME = "Contents/section0.xml"

HP = "http://www.hancom.co.kr/hwpml/2011/paragraph"
NS = {"hp": HP, "hc": "http://www.hancom.co.kr/hwpml/2011/core"}
Q = lambda tag: f"{{{HP}}}{tag}"


# Picture order follows the document's visual reading order. Picture 6 is the
# displayed mass/gravity matrix and is intentionally excluded from figure
# numbering because it functions as an equation, not a figure.
CAPTIONS = {
    1: "서양의 장대 줄타기와 한국의 전통 줄타기. 한국 줄타기는 장대 없이 부채만 든 채 처진 줄 위에서 역동적인 동작을 수행한다.",
    2: "Paoletti와 Mahadevan의 선행연구: 원호 위 카트–단일 역진자 모델과 줄 처짐에 따른 제어 입력의 변화.",
    3: "Paoletti 선행연구의 복원 토크를 적용한 시뮬레이션. 발보다 몸이 더 바깥에 있어도 몸이 직접 직립 방향으로 돌아와 한국 줄타기의 복귀 과정과 맞지 않았다.",
    4: "처진 줄의 실제 기하. 발은 V자 줄의 꼭짓점에 고정되어 줄과 함께 움직이며, 발의 자취는 반경 R의 원호를 이룬다.",
    5: "처진 줄 진자 위 상·하체 2링크 모델과 좌표 정의. φ·α·θ는 모두 연직 기준 오른쪽을 양(+)으로 잡았다.",
    7: "균형 운동을 설명하기 위한 모드 좌표 (φ, β, δ). β는 가중 무게중심 기울기이고 δ는 허리 접힘각이다.",
    8: "FWE의 인과 사슬: 허리를 접어 발을 무게중심 너머로 보내고, 기다린 뒤 자세를 회복한다.",
    9: "가운데로 되돌아오는 정진자와 기울수록 더 쓰러지는 역진자의 대조.",
    10: "입력 없이 떨어지지 않는 운동을 찾기 위한 세 후보: β=0, β=−φ, 그리고 그 사이의 일정한 비율.",
    11: "결합계의 두 운동: 줄과 몸이 엇갈려 유계 진동하는 안정모드와 지수적으로 커지는 불안정모드.",
    12: "(β, φ) 평면의 안정모드선 φ=rβ. 선 위의 상태는 불안정모드 성분 없이 유계 진동한다.",
    13: "과접기 비율 ε의 정의. 접기 전 발–무게중심 거리 hβ₀에 대해 발이 무게중심을 지나친 거리를 hεβ₀로 나타냈다.",
    14: "접기 전 상태. 상태점과 실제 2링크 자세에서 같은 무게중심 위치 x_CoM을 표시하였다.",
    15: "ε만큼 접은 뒤의 상태. 발과 몸의 배치는 바뀌지만 무게중심 위치 x_CoM은 보존된다.",
    16: "무게중심 등고선을 따라 안정모드선으로 이동하는 순간접기의 상태공간 기하.",
    17: "초기 기울기에서 ε*를 계산하고, 접기로 상태를 안정모드선 위에 옮긴 뒤 유계 진동에 들어가는 과정.",
    18: "ε* 접기의 상태공간 기하와 시간응답. 접은 뒤 φ와 β는 반대 위상으로 일정한 진폭의 유계 진동을 유지한다.",
    19: "예측점 평면에서 본 완전 FWE 사이클의 개념: 과접기–대기–펴기로 안정모드선에 착지한다.",
    20: "순간 완전 FWE 사이클의 상태공간 궤적과 시간응답. 과접기–대기–펴기 뒤 상태가 안정모드선에 놓인다.",
    21: "완전 FWE 사이클의 초기 상태.",
    22: "안정모드선을 지나쳐 과접기한 상태.",
    23: "반대편에 남긴 불안정모드 성분이 자라도록 기다리는 상태.",
    24: "펴기 뒤 안정모드선 위에 착지한 상태.",
    25: "예측점 평면에 나타낸 완전 FWE 사이클의 과접기–대기–펴기 궤적.",
    26: "유한시간 접기. 접는 동안에도 불안정모드가 자라므로 순간접기보다 더 많이 접어야 한다.",
    27: "반복 FWE 사이클. 대기시간이 길수록 같은 실행오차가 크게 증폭되며, 짧은 반복 보정이 더 강건하다.",
    28: "허리 토크 하나로 균형과 외란 복원을 확인한 LQR 제어 시뮬레이션.",
    29: "독립적인 MuJoCo 전물리 모델에 구현한 줄타기 시뮬레이션(V14).",
    30: "유한시간 접기 보정계수 γ를 적용한 FWE 시뮬레이션 검증(V15–V17).",
    31: "증분 접기를 적용한 전물리 시뮬레이션 검증(V19).",
    32: "줄타기 로봇 ‘어름이’의 CAD 설계와 제작된 실물.",
    33: "로봇 ‘어름이’의 전체 프레임, 상체, 하체·중앙 결합부.",
    34: "허리 모터의 전류–토크 관계를 확인하기 위한 서보 보정.",
    35: "비틀림 진자를 이용한 로봇의 관성모멘트 실측.",
    36: "나이프 에지 균형법을 이용한 로봇의 무게중심 위치 측정.",
}


# Ordinary-text captions and authoring placeholders replaced by native object
# captions. These indices refer to top-level paragraphs in the source XML.
REMOVE_TOP_PARAGRAPHS = {
    150,
    167,
    196,
    211,
    218,
    230,
    241,
    264,
    274,
    296,
    305,
    365,
    366,
    383,
    388,
    445,
    450,
}


def make_caption(template: etree._Element, number: int, description: str, width: str) -> etree._Element:
    cap = copy.deepcopy(template)
    cap.set("side", "BOTTOM")
    cap.set("fullSz", "0")
    cap.set("width", "8504")
    cap.set("gap", "850")
    cap.set("lastWidth", width)

    p = cap.find(".//hp:p", namespaces=NS)
    if p is None:
        raise RuntimeError("Caption template has no paragraph")
    p.set("paraPrIDRef", "34")
    p.set("styleIDRef", "0")
    p.set("pageBreak", "0")
    p.set("columnBreak", "0")
    p.set("merged", "0")
    for child in list(p):
        p.remove(child)

    run = etree.SubElement(p, Q("run"), charPrIDRef="34")
    lead = etree.SubElement(run, Q("t"))
    lead.text = "그림 "

    ctrl = etree.SubElement(run, Q("ctrl"))
    auto = etree.SubElement(ctrl, Q("autoNum"), num=str(number), numType="PICTURE")
    etree.SubElement(
        auto,
        Q("autoNumFormat"),
        type="DIGIT",
        userChar="",
        prefixChar="",
        suffixChar="",
        supscript="0",
    )

    tail = etree.SubElement(run, Q("t"))
    tail.text = f". {description}"
    return cap


def main() -> None:
    with zipfile.ZipFile(SOURCE, "r") as zin:
        section_bytes = zin.read(SECTION_NAME)
        info_by_name = {info.filename: info for info in zin.infolist()}

    parser = etree.XMLParser(remove_blank_text=False, recover=False)
    root = etree.fromstring(section_bytes, parser)
    pics = root.xpath(".//hp:pic", namespaces=NS)
    if len(pics) != 36:
        raise RuntimeError(f"Expected 36 pictures, found {len(pics)}")

    template = root.xpath(".//hp:pic[hp:caption//hp:autoNum]/hp:caption", namespaces=NS)
    if not template:
        raise RuntimeError("No native automatic-number caption template found")
    template = template[0]

    figure_number = 0
    for picture_order, pic in enumerate(pics, start=1):
        # Make all image objects editable and keep the equation image uncaptioned.
        pic.set("lock", "0")
        sz = pic.find("hp:sz", namespaces=NS)
        if sz is not None:
            sz.set("protect", "0")

        for old_caption in pic.findall("hp:caption", namespaces=NS):
            pic.remove(old_caption)

        description = CAPTIONS.get(picture_order)
        if description is None:
            continue

        figure_number += 1
        width = sz.get("width", "48190") if sz is not None else "48190"
        pic.insert(0, make_caption(template, figure_number, description, width))

    if figure_number != 35:
        raise RuntimeError(f"Expected 35 figure captions, created {figure_number}")

    top_paras = root.xpath("./hp:p", namespaces=NS)
    for index in sorted(REMOVE_TOP_PARAGRAPHS, reverse=True):
        if index >= len(top_paras):
            raise RuntimeError(f"Paragraph index out of range: {index}")
        root.remove(top_paras[index])

    # Remove stale cached layout runs only inside the new captions. Hancom
    # regenerates them when it opens and saves the edited HWPX.
    for lines in root.xpath(".//hp:pic/hp:caption//hp:linesegarray", namespaces=NS):
        parent = lines.getparent()
        if parent is not None:
            parent.remove(lines)

    new_section = etree.tostring(
        root,
        xml_declaration=True,
        encoding="UTF-8",
        standalone=True,
    )

    with zipfile.ZipFile(SOURCE, "r") as zin, zipfile.ZipFile(OUTPUT, "w") as zout:
        names = [info.filename for info in zin.infolist()]
        if "mimetype" in names:
            data = zin.read("mimetype")
            info = copy.copy(info_by_name["mimetype"])
            info.compress_type = zipfile.ZIP_STORED
            zout.writestr(info, data)
        for name in names:
            if name == "mimetype":
                continue
            info = copy.copy(info_by_name[name])
            data = new_section if name == SECTION_NAME else zin.read(name)
            zout.writestr(info, data)

    with zipfile.ZipFile(OUTPUT, "r") as check:
        bad = check.testzip()
        if bad:
            raise RuntimeError(f"CRC failure in {bad}")
        first = check.infolist()[0]
        if first.filename != "mimetype" or first.compress_type != zipfile.ZIP_STORED:
            raise RuntimeError("HWPX mimetype entry is not first and stored")

    print(f"created={OUTPUT.name}")
    print(f"figures={figure_number}")
    print(f"removed_text_captions={len(REMOVE_TOP_PARAGRAPHS)}")


if __name__ == "__main__":
    main()
