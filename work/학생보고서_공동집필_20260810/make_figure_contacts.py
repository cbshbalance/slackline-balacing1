from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parent
IMAGE_DIR = ROOT / "latest_hwpx_0810_modified" / "extracted" / "BinData"
OUT_DIR = ROOT / "latest_hwpx_0810_modified" / "figure_contacts"
OUT_DIR.mkdir(parents=True, exist_ok=True)

paths = sorted(
    (path for path in IMAGE_DIR.iterdir() if path.suffix.lower() in {".bmp", ".png", ".jpg", ".jpeg"}),
    key=lambda path: int("".join(ch for ch in path.stem if ch.isdigit()) or 0),
)

font = ImageFont.truetype(r"C:\Windows\Fonts\malgun.ttf", 22)
small = ImageFont.truetype(r"C:\Windows\Fonts\malgun.ttf", 17)

cell_w, cell_h = 440, 340
cols, rows = 3, 3
for offset in range(0, len(paths), cols * rows):
    selected = paths[offset : offset + cols * rows]
    canvas = Image.new("RGB", (cell_w * cols, cell_h * rows), "#d9d9d9")
    draw = ImageDraw.Draw(canvas)
    for index, path in enumerate(selected):
        row, col = divmod(index, cols)
        x0, y0 = col * cell_w, row * cell_h
        with Image.open(path) as source:
            original_size = source.size
            image = ImageOps.contain(source.convert("RGB"), (cell_w - 24, cell_h - 62))
        x = x0 + (cell_w - image.width) // 2
        y = y0 + 8 + (cell_h - 62 - image.height) // 2
        canvas.paste(image, (x, y))
        draw.rectangle((x0, y0, x0 + cell_w - 1, y0 + cell_h - 1), outline="#777777", width=1)
        draw.rectangle((x0, y0 + cell_h - 54, x0 + cell_w, y0 + cell_h), fill="white")
        draw.text((x0 + 10, y0 + cell_h - 50), path.name, fill="black", font=font)
        draw.text(
            (x0 + 180, y0 + cell_h - 45),
            f"{original_size[0]}×{original_size[1]}",
            fill="#555555",
            font=small,
        )
    start = offset + 1
    end = offset + len(selected)
    canvas.save(OUT_DIR / f"contact_{start:02d}_{end:02d}.jpg", quality=94)

print(f"wrote {len(list(OUT_DIR.glob('contact_*.jpg')))} contact sheets to {OUT_DIR}")
