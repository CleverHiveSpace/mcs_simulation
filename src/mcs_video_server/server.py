from fastapi import FastAPI, HTTPException, Query, BackgroundTasks
from fastapi.responses import StreamingResponse
from datetime import datetime, timedelta
from typing import List, Tuple
import os
import subprocess
import tempfile
import uuid
import shutil
from pathlib import Path

app = FastAPI(title="Edge Recorder Video Service")

VIDEO_DIR = Path(os.getenv("VIDEO_DIR", "/tmp/edge_rec"))

FILENAME_PREFIX = "navcam_front_"
FILENAME_DATETIME_FMT = "%Y%m%d_%H%M"
FILENAME_SUFFIX = ".mp4"


def _parse_file_start(path: Path) -> datetime:
    try:
        ts_part = path.stem[len(FILENAME_PREFIX) :]
        return datetime.strptime(ts_part, FILENAME_DATETIME_FMT)
    except ValueError as exc:
        raise RuntimeError(f"Unexpected filename format: {path.name}") from exc


def _file_duration(path: Path) -> float:
    try:
        cmd = [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "format=duration",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            str(path),
        ]
        out = subprocess.check_output(cmd, text=True).strip()
        return float(out)
    except Exception:
        return 60.0


def _matching_clips(start: datetime, end: datetime) -> List[Tuple[Path, datetime, datetime]]:
    clips: List[Tuple[Path, datetime, datetime]] = []
    for p in sorted(VIDEO_DIR.glob(f"{FILENAME_PREFIX}*{FILENAME_SUFFIX}")):
        clip_start = _parse_file_start(p)
        duration = _file_duration(p)
        clip_end = clip_start + timedelta(seconds=duration)
        if clip_end < start or clip_start > end:
            continue
        clips.append((p, clip_start, clip_end))
    return clips


def _trim_and_concat(clips: List[Tuple[Path, datetime, datetime]], start: datetime, end: datetime) -> Path:
    workdir = Path(tempfile.mkdtemp(prefix="edge_trim_"))
    segments = []

    for idx, (path, clip_start, clip_end) in enumerate(clips):
        seg_path = workdir / f"seg_{idx:02d}.mp4"
        trim_start = max(0.0, (start - clip_start).total_seconds())
        trim_end = min((end - clip_start).total_seconds(), (clip_end - clip_start).total_seconds())
        trim_dur = trim_end - trim_start
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-ss",
            str(trim_start),
            "-i",
            str(path),
            "-t",
            str(trim_dur),
            "-c",
            "copy",
            str(seg_path),
        ]
        subprocess.check_call(cmd)
        segments.append(seg_path)

    if len(segments) == 1:
        return segments[0]

    list_file = workdir / "inputs.txt"
    with list_file.open("w") as fp:
        for seg in segments:
            fp.write(f"file '{seg.as_posix()}'\n")

    output_path = workdir / "output.mp4"
    subprocess.check_call([
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        str(list_file),
        "-c",
        "copy",
        str(output_path),
    ])

    return output_path


def _cleanup(path: Path):
    shutil.rmtree(path.parent, ignore_errors=True)


@app.get("/video", summary="Return camera footage within a time range", response_class=StreamingResponse)
async def get_video(
    background_tasks: BackgroundTasks,
    start: datetime = Query(..., description="ISO timestamp (UTC) marking beginning of clip"),
    end: datetime = Query(..., description="ISO timestamp (UTC) marking end of clip"),
):
    if end <= start:
        raise HTTPException(status_code=400, detail="'end' must be after 'start'")

    clips = _matching_clips(start, end)
    if not clips:
        raise HTTPException(status_code=404, detail="No footage available for requested range")

    try:
        result_path = _trim_and_concat(clips, start, end)
    except subprocess.CalledProcessError:
        raise HTTPException(status_code=500, detail="Error processing video")

    background_tasks.add_task(_cleanup, result_path)

    headers = {"Content-Disposition": f"attachment; filename=footage_{start.isoformat()}_{end.isoformat()}.mp4"}
    return StreamingResponse(result_path.open("rb"), media_type="video/mp4", headers=headers)
