from __future__ import annotations

from pathlib import Path

import pypdfium2 as pdfium
from PIL import Image, ImageDraw, ImageFont


WORK = Path(__file__).resolve().parent
PDF = Path(r"C:\Users\Public\Documents\ESTsoft\CreatorTemp\report_qa.pdf")
OUT = WORK / "rendered_report"
PAGES = OUT / "pages"
CONTACTS = OUT / "contacts"
PAGES.mkdir(parents=True, exist_ok=True)
CONTACTS.mkdir(parents=True, exist_ok=True)

font_path = Path(r"C:\Windows\Fonts\malgunbd.ttf")
label_font = ImageFont.truetype(str(font_path), 30)

pdf = pdfium.PdfDocument(str(PDF))
page_paths: list[Path] = []
for index in range(len(pdf)):
    page = pdf[index]
    bitmap = page.render(scale=2.0)
    image = bitmap.to_pil().convert("RGB")
    path = PAGES / f"page-{index + 1:03d}.png"
    image.save(path, quality=94)
    page_paths.append(path)

for start in range(0, len(page_paths), 4):
    subset = page_paths[start : start + 4]
    page_images: list[Image.Image] = []
    for number, path in enumerate(subset, start=start + 1):
        with Image.open(path) as source:
            page = source.convert("RGB")
        target_width = 690
        target_height = round(page.height * target_width / page.width)
        page = page.resize((target_width, target_height), Image.Resampling.LANCZOS)
        tile = Image.new("RGB", (target_width + 20, target_height + 70), "white")
        tile.paste(page, (10, 55))
        draw = ImageDraw.Draw(tile)
        draw.text((10, 8), f"Page {number}", fill="#20364a", font=label_font)
        draw.rectangle((8, 53, target_width + 11, target_height + 56), outline="#777777", width=2)
        page_images.append(tile)
    rows = (len(page_images) + 1) // 2
    contact = Image.new("RGB", (1420, rows * page_images[0].height), "#d8dde2")
    for offset, tile in enumerate(page_images):
        x = (offset % 2) * 710
        y = (offset // 2) * tile.height
        contact.paste(tile, (x, y))
    contact.save(CONTACTS / f"contact-{start + 1:03d}-{start + len(subset):03d}.jpg", quality=92)

print(f"rendered {len(page_paths)} pages")
