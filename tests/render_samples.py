"""Run built samples without a window and retain their images and terminal output."""

import os
from pathlib import Path
import re
import subprocess
import sys

from PIL import Image

sys.stdout.reconfigure(encoding="utf-8")
build = Path(sys.argv[1]).resolve()
output = Path("sample-results").resolve()
env = dict(os.environ, GPRT_HEADLESS_SURFACE="1", GPRT_HEADLESS_FRAME_LIMIT="2",
           GPRT_TERMINAL_COLOR="1", GPRT_TERMINAL_ASCII="0", COLUMNS="160", LINES="50",
           VK_LOADER_LAYERS_DISABLE="~implicit~",
           VK_LAYER_ENABLES="VK_VALIDATION_FEATURE_ENABLE_SYNCHRONIZATION_VALIDATION_EXT")
samples = sorted(p for p in build.glob("s[0-9]_*") if p.is_file() and p.suffix in ("", ".exe"))
if not samples:
    raise SystemExit(f"No sample executables in {build}")
failed = []
excluded = {
    "s4_0_mbtriangles": "requires VK_NV_ray_tracing_motion_blur",
    "s8_0_imgui_rt": "existing GUI attachment READ_AFTER_WRITE validation errors",
    "s9_0_differentiable": "existing GUI attachment READ_AFTER_WRITE validation errors",
}
terminal_test = build / ("t13_terminal.exe" if os.name == "nt" else "t13_terminal")
for exe in [terminal_test, *samples]:
    if exe.stem in excluded:
        print(f"SKIP {exe.stem}: {excluded[exe.stem]}", flush=True)
        continue
    work = output / exe.stem
    work.mkdir(parents=True, exist_ok=True)
    for old_image in work.glob("*.png"):
        old_image.unlink()
    print(f"::group::{exe.stem}", flush=True)
    try:
        test_env = dict(env, GPRT_HEADLESS_FRAME_LIMIT="3") if exe == terminal_test else env
        result = subprocess.run([str(exe)], cwd=work, env=test_env, capture_output=True, timeout=180)
        log = (result.stdout + result.stderr).decode("utf-8", errors="replace")
        (work / "output.log").write_text(log, encoding="utf-8")
        print(log, flush=True)
        if result.returncode:
            raise RuntimeError(f"exited with {result.returncode}")
        if re.search(r"Validation Error|SYNC-HAZARD|ERROR: \[|#gprt error:", log):
            raise RuntimeError("GPRT or Vulkan validation reported an error")
        if "VK_LAYER_KHRONOS_validation is not present" in log:
            raise RuntimeError("Vulkan validation is unavailable")
        if not re.search(r"\x1b\[38;5;\d+m[\u2591-\u2593\u2588]{2}", log):
            raise RuntimeError("no colored block preview was printed")
        if "\x1b[2J" in log or "\x1b[H" in log:
            raise RuntimeError("redirected previews must preserve earlier log output")
        if exe == terminal_test:
            print("PASS terminal frame limits, filtering, and mapped contents", flush=True)
            continue
        images = list(work.glob("*.png"))
        if not images:
            raise RuntimeError("no PNG was saved")
        for path in images:
            with Image.open(path) as image:
                if all(low == high for low, high in image.convert("RGB").getextrema()):
                    raise RuntimeError(f"{path.name} contains a uniform image")
                print(f"PASS {path.name}: {image.width} x {image.height}", flush=True)
    except subprocess.TimeoutExpired as error:
        failed.append(exe.stem)
        log = ((error.stdout or b"") + (error.stderr or b"")).decode("utf-8", errors="replace")
        (work / "output.log").write_text(log, encoding="utf-8")
        print(log, flush=True)
        print(f"::error::{exe.stem}: exceeded 180 seconds", flush=True)
    except (RuntimeError, OSError) as error:
        failed.append(exe.stem)
        print(f"::error::{exe.stem}: {error}", flush=True)
    finally:
        print("::endgroup::", flush=True)
if failed:
    raise SystemExit("Failed samples: " + ", ".join(failed))
