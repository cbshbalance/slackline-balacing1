import sys
import os
import subprocess

def get_ffmpeg_path():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None

def download_video(url, output_format="%(title)s.%(ext)s"):
    if not url:
        print("[!] URL이 입력되지 않았습니다.")
        return

    ffmpeg_path = get_ffmpeg_path()
    cmd = [
        sys.executable, "-m", "yt_dlp",
        "--no-warnings"
    ]

    if ffmpeg_path and os.path.exists(ffmpeg_path):
        cmd.extend(["--ffmpeg-location", ffmpeg_path])

    cmd.extend([
        "-f", "bestvideo[ext=mp4]+bestaudio[ext=m4a]/bestvideo+bestaudio/best",
        "--merge-output-format", "mp4",
        "-o", output_format,
        url
    ])

    print(f"[*] 다운로드 시작: {url}")
    result = subprocess.run(cmd)

    if result.returncode == 0:
        print("\n[+] 다운로드가 성공적으로 완료되었습니다!")
    else:
        print(f"\n[-] 다운로드 중 오류가 발생했습니다 (코드: {result.returncode}).")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        target_url = sys.argv[1]
    else:
        target_url = input("유튜브 URL (또는 Shorts 링크)를 입력하세요: ").strip()

    if target_url:
        download_video(target_url)
