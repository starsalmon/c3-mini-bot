Import("env")

import os
import subprocess
from datetime import datetime, timezone

proj = env["PROJECT_DIR"]
header = os.path.join(proj, "include", "bot_fw_version.h")


def resolve_git_hash() -> str:
    # 1) explicit env (tarball / CI): FW_GIT_HASH=abc123 pio run …
    v = os.environ.get("FW_GIT_HASH", "").strip()
    if v:
        return v
    # 2) one-line file in repo root (optional, for offline builds)
    fw_file = os.path.join(proj, "FW_VERSION")
    if os.path.isfile(fw_file):
        with open(fw_file, encoding="utf-8") as f:
            line = f.read().strip().split()
            if line:
                return line[0]
    # 3) git checkout
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=proj,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unknown"


git_hash = resolve_git_hash()
build_time = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

os.makedirs(os.path.dirname(header), exist_ok=True)
with open(header, "w", encoding="utf-8") as f:
    f.write("#pragma once\n")
    f.write(f'#define BOT_FW_GIT_HASH "{git_hash}"\n')
    f.write(f'#define BOT_FW_BUILD_TIME "{build_time}"\n')