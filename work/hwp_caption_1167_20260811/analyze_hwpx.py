from __future__ import annotations

import json
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parent
XML_PATH = ROOT / "hwpx_unpacked" / "Contents" / "section0.xml"
OUT_JSON = ROOT / "hwpx_inventory.json"
OUT_TEXT = ROOT / "hwpx_inventory.txt"

NS = {
    "hs": "http://www.hancom.co.kr/hwpml/2011/section",
    "hp": "http://www.hancom.co.kr/hwpml/2011/paragraph",
    "hc": "http://www.hancom.co.kr/hwpml/2011/core",
}


def node_text(node: etree._Element | None) -> str:
    if node is None:
        return ""
    return "".join(node.xpath(".//hp:t/text()", namespaces=NS)).strip()


def direct_paragraph_text(p: etree._Element) -> str:
    parts: list[str] = []
    for t in p.xpath(".//hp:t", namespaces=NS):
        # Exclude text embedded in native object captions so ordinary paragraph
        # captions can be distinguished from object captions.
        if t.xpath("ancestor::hp:caption", namespaces=NS):
            continue
        if t.text:
            parts.append(t.text)
    return "".join(parts).strip()


def main() -> None:
    tree = etree.parse(str(XML_PATH))
    root = tree.getroot()
    top_paras = root.xpath("./hp:p", namespaces=NS)

    page = 1
    para_rows: list[dict] = []
    pic_rows: list[dict] = []

    for i, p in enumerate(top_paras):
        if i and p.get("pageBreak") == "1":
            page += 1
        text = direct_paragraph_text(p)
        para_rows.append(
            {
                "top_para_index": i,
                "page_estimate": page,
                "page_break": p.get("pageBreak"),
                "para_pr": p.get("paraPrIDRef"),
                "char_prs": p.xpath("./hp:run/@charPrIDRef", namespaces=NS),
                "text": text,
                "has_pic": bool(p.xpath(".//hp:pic", namespaces=NS)),
            }
        )

        for pic in p.xpath(".//hp:pic", namespaces=NS):
            image_ref = ""
            refs = pic.xpath(".//hc:img/@binaryItemIDRef", namespaces=NS)
            if refs:
                image_ref = refs[0]
            caption = pic.find("hp:caption", namespaces=NS)
            auto = caption.xpath(".//hp:autoNum", namespaces=NS) if caption is not None else []
            pic_rows.append(
                {
                    "order": len(pic_rows) + 1,
                    "top_para_index": i,
                    "page_estimate": page,
                    "pic_id": pic.get("id"),
                    "image_ref": image_ref,
                    "width": (pic.find("hp:sz", namespaces=NS) or {}).get("width") if pic.find("hp:sz", namespaces=NS) is not None else None,
                    "height": (pic.find("hp:sz", namespaces=NS) or {}).get("height") if pic.find("hp:sz", namespaces=NS) is not None else None,
                    "numbering_type": pic.get("numberingType"),
                    "caption_present": caption is not None,
                    "caption_text": node_text(caption),
                    "caption_auto_num": [dict(a.attrib) for a in auto],
                    "paragraph_text": text,
                }
            )

    all_auto = []
    for auto in root.xpath(".//hp:autoNum", namespaces=NS):
        parent_caption = auto.xpath("ancestor::hp:caption[1]", namespaces=NS)
        owner = parent_caption[0].getparent() if parent_caption else None
        all_auto.append(
            {
                "attrs": dict(auto.attrib),
                "owner_tag": etree.QName(owner).localname if owner is not None else "",
                "owner_id": owner.get("id") if owner is not None else "",
                "caption_text": node_text(parent_caption[0]) if parent_caption else "",
            }
        )

    ordinary_caption_candidates = [
        row
        for row in para_rows
        if row["text"].startswith("그림")
        or "그림 " in row["text"]
        or "그림." in row["text"]
    ]

    data = {
        "estimated_pages": page,
        "top_level_paragraphs": len(top_paras),
        "pictures": pic_rows,
        "all_auto_numbers": all_auto,
        "ordinary_caption_candidates": ordinary_caption_candidates,
        "paragraphs": para_rows,
    }
    OUT_JSON.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    lines: list[str] = []
    lines.append(f"estimated pages: {page}")
    lines.append(f"top-level paragraphs: {len(top_paras)}")
    lines.append(f"pictures: {len(pic_rows)}")
    lines.append(f"auto numbers: {len(all_auto)}")
    lines.append("")
    lines.append("[PICTURES]")
    for row in pic_rows:
        lines.append(
            f"#{row['order']:02d} p{row['page_estimate']:02d} {row['image_ref']} "
            f"pic_id={row['pic_id']} native_caption={row['caption_present']} "
            f"auto={row['caption_auto_num']} cap={row['caption_text']!r} "
            f"para={row['paragraph_text'][:180]!r}"
        )
    lines.append("")
    lines.append("[AUTO NUMBERS]")
    for row in all_auto:
        lines.append(str(row))
    lines.append("")
    lines.append("[ORDINARY CAPTION CANDIDATES]")
    for row in ordinary_caption_candidates:
        lines.append(
            f"p{row['page_estimate']:02d} para#{row['top_para_index']}: {row['text']!r}"
        )
    lines.append("")
    lines.append("[ALL PARAGRAPHS]")
    for row in para_rows:
        if row["text"] or row["has_pic"]:
            lines.append(
                f"p{row['page_estimate']:02d} para#{row['top_para_index']} "
                f"pic={row['has_pic']} text={row['text']!r}"
            )
    OUT_TEXT.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
