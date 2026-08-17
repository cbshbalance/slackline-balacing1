from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parent / "extracted"


def local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def element_text(element: etree._Element) -> str:
    values: list[str] = []
    for node in element.iter():
        if local(node.tag) == "t" and node.text:
            values.append(node.text)
        elif local(node.tag) in {"lineBreak", "br"}:
            values.append("\n")
    return "".join(values).strip()


tree = etree.parse(str(ROOT / "Contents" / "section0.xml"))
root = tree.getroot()
tags = Counter(local(node.tag) for node in root.iter())

paragraphs = [node for node in root.iter() if local(node.tag) == "p"]
paragraph_records: list[dict[str, object]] = []
for index, paragraph in enumerate(paragraphs):
    text = element_text(paragraph)
    pictures = [
        node.get("binaryItemIDRef")
        for node in paragraph.iter()
        if local(node.tag) == "img" and node.get("binaryItemIDRef")
    ]
    equations = []
    for equation in paragraph.iter():
        if local(equation.tag) != "equation":
            continue
        scripts = [
            node.text or ""
            for node in equation.iter()
            if local(node.tag) in {"script", "text"} and node.text
        ]
        equations.append(" | ".join(scripts))
    if text or pictures or equations:
        paragraph_records.append(
            {
                "element_index": index,
                "text": text,
                "pictures": pictures,
                "equations": equations,
                "pageBreak": paragraph.get("pageBreak"),
                "columnBreak": paragraph.get("columnBreak"),
            }
        )

equations = []
for index, equation in enumerate(
    [node for node in root.iter() if local(node.tag) == "equation"], 1
):
    equations.append(
        {
            "index": index,
            "attributes": dict(equation.attrib),
            "scripts": [
                node.text or ""
                for node in equation.iter()
                if local(node.tag) in {"script", "text"} and node.text
            ],
        }
    )

tables = []
for index, table in enumerate([node for node in root.iter() if local(node.tag) == "tbl"], 1):
    rows = [node for node in table if local(node.tag) == "tr"]
    tables.append(
        {
            "index": index,
            "attributes": dict(table.attrib),
            "rows": [
                [element_text(cell) for cell in row if local(cell.tag) == "tc"]
                for row in rows
            ],
        }
    )

sections = [dict(node.attrib) for node in root.iter() if local(node.tag) == "secPr"]
start_nums = [dict(node.attrib) for node in root.iter() if local(node.tag) == "startNum"]
page_nums = [dict(node.attrib) for node in root.iter() if local(node.tag) == "pageNum"]
fields = [
    {"tag": local(node.tag), "attributes": dict(node.attrib), "text": element_text(node)}
    for node in root.iter()
    if local(node.tag) in {"fieldBegin", "fieldEnd"}
]

inventory = {
    "tag_counts": dict(tags),
    "paragraphs": paragraph_records,
    "equations": equations,
    "tables": tables,
    "secPr": sections,
    "startNum": start_nums,
    "pageNum": page_nums,
    "fields": fields,
}

(Path(__file__).resolve().parent / "package_analysis.json").write_text(
    json.dumps(inventory, ensure_ascii=False, indent=2), encoding="utf-8"
)

print("paragraph records", len(paragraph_records))
print("equations", len(equations))
print("tables", len(tables))
print("secPr", sections)
print("startNum", start_nums)
print("pageNum", page_nums)
print("fields", fields)
print("page breaks", sum(record["pageBreak"] == "1" for record in paragraph_records))
print("picture refs", sum(len(record["pictures"]) for record in paragraph_records))
print("unique pictures", sorted({pic for record in paragraph_records for pic in record["pictures"]}))
print("\nEQUATIONS")
for record in equations:
    print(record["index"], " || ".join(record["scripts"]))
print("\nTABLES")
for table in tables:
    print("TABLE", table["index"], json.dumps(table["rows"], ensure_ascii=False))
