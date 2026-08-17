from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parents[2]
WORK = Path(__file__).resolve().parent
HWPX = WORK / "latest_hwpx_0810_modified" / "extracted" / "BinData"
DOCX_MEDIA = WORK / "docx_source_figures" / "word" / "media"
OUT = WORK / "report_assets"
OUT.mkdir(parents=True, exist_ok=True)

MALGUN = Path(r"C:\Windows\Fonts\malgun.ttf")
MALGUN_BOLD = Path(r"C:\Windows\Fonts\malgunbd.ttf")
FONT = ImageFont.truetype(str(MALGUN), 30)
FONT_SMALL = ImageFont.truetype(str(MALGUN), 24)
FONT_BOLD = ImageFont.truetype(str(MALGUN_BOLD), 32)


def numbered(name: str) -> Path:
    for path in HWPX.iterdir():
        if path.stem.lower() == name.lower():
            return path
    raise FileNotFoundError(name)


def load(path: Path) -> Image.Image:
    with Image.open(path) as source:
        return source.convert("RGB")


def save_copy(source: Path, target: str, max_width: int = 2200) -> Path:
    image = load(source)
    if image.width > max_width:
        height = round(image.height * max_width / image.width)
        image = image.resize((max_width, height), Image.Resampling.LANCZOS)
    output = OUT / target
    image.save(output, quality=96)
    return output


def collage(
    sources: list[Path],
    labels: list[str],
    target: str,
    *,
    columns: int,
    cell_width: int = 860,
    cell_height: int = 610,
) -> Path:
    rows = (len(sources) + columns - 1) // columns
    canvas = Image.new("RGB", (cell_width * columns, cell_height * rows), "white")
    draw = ImageDraw.Draw(canvas)
    for index, (source, label) in enumerate(zip(sources, labels)):
        row, col = divmod(index, columns)
        x0, y0 = col * cell_width, row * cell_height
        image = ImageOps.contain(load(source), (cell_width - 36, cell_height - 92))
        x = x0 + (cell_width - image.width) // 2
        y = y0 + 18 + (cell_height - 92 - image.height) // 2
        canvas.paste(image, (x, y))
        draw.rectangle((x0, y0, x0 + cell_width - 1, y0 + cell_height - 1), outline="#b7b7b7", width=2)
        draw.rectangle((x0, y0 + cell_height - 64, x0 + cell_width, y0 + cell_height), fill="#f5f7f8")
        bbox = draw.textbbox((0, 0), label, font=FONT_BOLD)
        draw.text(
            (x0 + (cell_width - (bbox[2] - bbox[0])) // 2, y0 + cell_height - 55),
            label,
            fill="#20364a",
            font=FONT_BOLD,
        )
    output = OUT / target
    canvas.save(output, quality=96)
    return output


collage(
    [numbered("image1"), numbered("image2")],
    ["(가) 서양 줄타기: 장대 사용", "(나) 한국 줄타기: 장대 없이 부채 사용"],
    "fig01_west_korea.png",
    columns=2,
    cell_height=760,
)

collage(
    [numbered("image3"), numbered("image4")],
    ["(가) 선행연구의 원호 위 카트-역진자 모델", "(나) 줄 처짐과 제어 노력의 관계"],
    "fig02_prior_study.png",
    columns=2,
    cell_height=650,
)

save_copy(numbered("image6"), "fig03_rope_geometry.png")

schematic = Image.new("RGB", (950, 720), "white")
sd = ImageDraw.Draw(schematic)
cx, cy, radius = 450, 80, 430
arc_points: list[tuple[float, float]] = []
for degree in range(-65, 66, 2):
    rad = degree * 3.141592653589793 / 180
    arc_points.append((cx + radius * __import__("math").sin(rad), cy + radius * __import__("math").cos(rad)))
sd.line(arc_points, fill="#263746", width=7)
sd.line((cx, cy, cx, cy + radius + 40), fill="#8a959d", width=3)

phi = -12 * 3.141592653589793 / 180
foot = (cx + radius * __import__("math").sin(phi), cy + radius * __import__("math").cos(phi))
sd.line((cx, cy, foot[0], foot[1]), fill="#4a86c5", width=4)
sd.ellipse((foot[0] - 10, foot[1] - 10, foot[0] + 10, foot[1] + 10), fill="#d74b4b")

alpha = -12 * 3.141592653589793 / 180
theta = 20 * 3.141592653589793 / 180
length1, length2 = 190, 210
hip = (foot[0] + length1 * __import__("math").sin(alpha), foot[1] - length1 * __import__("math").cos(alpha))
head = (hip[0] + length2 * __import__("math").sin(theta), hip[1] - length2 * __import__("math").cos(theta))
sd.line((foot[0], foot[1], hip[0], hip[1]), fill="#2f8068", width=20)
sd.line((hip[0], hip[1], head[0], head[1]), fill="#725aa6", width=22)
sd.ellipse((hip[0] - 12, hip[1] - 12, hip[0] + 12, hip[1] + 12), fill="#f1a935")
sd.ellipse((head[0] - 20, head[1] - 20, head[0] + 20, head[1] + 20), fill="#725aa6")

sd.text((510, 145), "φ", fill="#2a6eae", font=FONT_BOLD)
sd.text((foot[0] - 85, foot[1] - 125), "α", fill="#216b58", font=FONT_BOLD)
sd.text((hip[0] + 65, hip[1] - 115), "θ", fill="#5f478e", font=FONT_BOLD)
sd.text((hip[0] + 70, hip[1] - 55), "δ=θ-α", fill="#a26714", font=FONT_SMALL)
sd.text((foot[0] - 135, foot[1] + 18), "발(줄 꼭짓점에 고정)", fill="#8b2d2d", font=FONT_SMALL)
sd.text((cx + 25, 330), "R", fill="#2a6eae", font=FONT_BOLD)
sd.rounded_rectangle((55, 585, 895, 695), radius=18, fill="#f3f6f8", outline="#9aabb5", width=2)
sd.text((82, 602), "φ, α, θ는 모두 연직 기준 오른쪽이 (+)", fill="#233b4d", font=FONT_SMALL)
sd.text((82, 645), "원호는 줄을 따라 미끄러지는 길이 아니라 발 궤적의 등가 표현", fill="#233b4d", font=FONT_SMALL)
schematic_path = OUT / "system_schematic.png"
schematic.save(schematic_path, quality=96)

collage(
    [schematic_path, numbered("image8")],
    ["(가) 처진 줄 위 상·하체 2링크 모델", "(나) 결합 운동방정식의 질량·중력 행렬"],
    "fig04_system_model.png",
    columns=2,
    cell_height=650,
)

collage(
    [numbered("image9"), numbered("image10"), numbered("image11"), numbered("image12")],
    ["① 기울어짐", "② 접기: 발을 넘겨 보냄", "③ 기다리기", "④ 펴기"],
    "fig05_fwe_sequence.png",
    columns=4,
    cell_width=510,
    cell_height=440,
)

collage(
    [numbered("image17"), numbered("image18"), numbered("image19")],
    ["후보 ① β=0: 몸만 수직", "후보 ② β=-φ: 줄과 몸이 나란함", "후보 ③ 그 사이의 일정한 비율"],
    "fig06_mode_thought_experiment.png",
    columns=3,
    cell_width=680,
    cell_height=600,
)

save_copy(numbered("image20"), "fig07_stable_unstable_modes.png")
save_copy(numbered("image22"), "fig08_pendulum_contrast.png")
save_copy(numbered("image29"), "fig09_epsilon_geometry.png")

collage(
    [numbered("image25"), numbered("image26"), numbered("image27"), numbered("image28")],
    ["① 초기 기울어짐", "② 접기량 계산", "③ 안정모드선 도달", "④ 안정모드선 위 운동"],
    "fig10_epsilon_sim_steps.png",
    columns=2,
    cell_width=1050,
    cell_height=540,
)

save_copy(DOCX_MEDIA / "rId35.png", "fig11_raw_vs_predicted.png")
save_copy(DOCX_MEDIA / "rId39.png", "fig12_single_fold_bounded.png")
save_copy(ROOT / "docs" / "예측점평면_S2_순간완전사이클.png", "fig13_full_cycle.png")
save_copy(ROOT / "docs" / "예측점평면_S3_유한시간접기.png", "fig14_finite_time.png")
save_copy(ROOT / "docs" / "예측점평면_S5_반복.png", "fig15_repeated_error.png")

collage(
    [numbered("image37"), numbered("image38"), numbered("image39"), numbered("image40")],
    ["(가) LQR 시뮬레이션", "(나) MuJoCo 전물리 모델", "(다) 유한시간 FWE", "(라) 증분 접기"],
    "fig16_simulation_evolution.png",
    columns=2,
    cell_width=960,
    cell_height=590,
)


chart = Image.new("RGB", (2200, 1050), "white")
draw = ImageDraw.Draw(chart)
title_font = ImageFont.truetype(str(MALGUN_BOLD), 56)
section_font = ImageFont.truetype(str(MALGUN_BOLD), 39)
axis_font = ImageFont.truetype(str(MALGUN), 28)
value_font = ImageFont.truetype(str(MALGUN_BOLD), 33)
note_font = ImageFont.truetype(str(MALGUN), 27)

title = "기존 FWE와 증분 접기의 전물리 시뮬레이션 생존 결과"
bbox = draw.textbbox((0, 0), title, font=title_font)
draw.text(((2200 - bbox[2]) / 2, 28), title, fill="#243746", font=title_font)

# Left panel: 120-second comparison.
left = (100, 150, 1290, 900)
draw.text((355, 155), "120초 시험 구간 비교", fill="#20364a", font=section_font)
plot_left, plot_top, plot_right, plot_bottom = 180, 250, 1200, 800
draw.line((plot_left, plot_top, plot_left, plot_bottom), fill="#555555", width=4)
draw.line((plot_left, plot_bottom, plot_right, plot_bottom), fill="#555555", width=4)
for tick in [0, 30, 60, 90, 120]:
    y = plot_bottom - int((plot_bottom - plot_top) * tick / 120)
    draw.line((plot_left, y, plot_right, y), fill="#dddddd", width=2)
    draw.text((105, y - 18), str(tick), fill="#555555", font=axis_font)
draw.text((40, 490), "생존시간 [s]", fill="#333333", font=axis_font)

labels = ["기존 FWE\n평균", "기존 FWE\n최대", "증분 접기\n20개 조건"]
values = [90, 120, 120]
colors = ["#7b8fa6", "#4d7590", "#2f8068"]
centers = [390, 690, 1010]
for center, label, value, color in zip(centers, labels, values, colors):
    bar_width = 150
    height = int((plot_bottom - plot_top) * value / 120)
    y = plot_bottom - height
    draw.rounded_rectangle((center - bar_width // 2, y, center + bar_width // 2, plot_bottom), radius=18, fill=color)
    text = f"{value} s"
    tb = draw.textbbox((0, 0), text, font=value_font)
    draw.text((center - tb[2] / 2, y - 48), text, fill="#273746", font=value_font)
    for line_index, line in enumerate(label.split("\n")):
        lb = draw.textbbox((0, 0), line, font=axis_font)
        draw.text((center - lb[2] / 2, 820 + line_index * 34), line, fill="#333333", font=axis_font)
draw.text((945, 280), "20/20 모두\n시험상한 도달", fill="white", font=note_font)

# Right panel: log-scale follow-up result.
draw.line((1370, 150, 1370, 930), fill="#d0d0d0", width=3)
draw.text((1515, 155), "상한 없는 후속 시뮬레이션", fill="#20364a", font=section_font)
draw.text((1510, 255), "27,000초까지 균형 유지", fill="#a25a14", font=value_font)
track = (1480, 400, 2070, 500)
draw.rounded_rectangle(track, radius=35, fill="#f2e1cc")
draw.rounded_rectangle((1480, 400, 1980, 500), radius=35, fill="#d88b35")
draw.text((1650, 426), "27,000 s", fill="white", font=value_font)
draw.text((1485, 570), "낙하가 아니라 계산 종료로 시험을 마침", fill="#6c3d10", font=note_font)
draw.text((1485, 625), "→ 전물리 시뮬레이션 안에서", fill="#8b2d2d", font=value_font)
draw.text((1485, 680), "   사실상 영구 생존", fill="#8b2d2d", font=value_font)
draw.rounded_rectangle((1450, 775, 2100, 905), radius=20, fill="#f5f7f8", outline="#a9b5bd", width=2)
draw.text((1480, 800), "※ 120초는 시험 상한이며,\n   실물 로봇의 성공 기록과는 구분한다.", fill="#45545f", font=note_font)
chart.save(OUT / "fig17_survival_comparison.png", quality=96)

collage(
    [numbered("image41"), numbered("image42")],
    ["(가) 로봇 전체 CAD 모델", "(나) 실물 로봇 ‘어름이’"],
    "fig18_robot_design_real.png",
    columns=2,
    cell_height=760,
)

collage(
    [numbered("image43"), numbered("image44"), numbered("image45")],
    ["(가) 전체 프레임", "(나) 상체", "(다) 하체·줄 결합부"],
    "fig19_robot_components.png",
    columns=3,
    cell_width=620,
    cell_height=720,
)

collage(
    [numbered("image46"), numbered("image47"), numbered("image48")],
    ["(가) 줄 위 폐루프 시험", "(나) 무게중심 위치 측정", "(다) 상·하체 비틀림 측정"],
    "fig20_robot_measurements.png",
    columns=3,
    cell_width=680,
    cell_height=650,
)

print(f"prepared {len(list(OUT.glob('fig*.png')))} report assets in {OUT}")
