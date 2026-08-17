import json

d = json.load(open("hwpx_inventory.json", encoding="utf-8"))
paragraphs = d["paragraphs"]
for pic in d["pictures"]:
    i = pic["top_para_index"]
    print(f"PIC {pic['order']:02d} {pic['image_ref']} para={i}")
    for p in paragraphs[max(0, i - 2) : min(len(paragraphs), i + 4)]:
        text = p["text"].replace("\n", " ")[:260]
        print(f"  {p['top_para_index']}: {text}")
    print()
