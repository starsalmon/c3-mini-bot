Import("env")

import os
import subprocess
from datetime import datetime, timezone

proj = env["PROJECT_DIR"]
header = os.path.join(proj, "include", "bot_fw_version.h")

try:
    git_hash = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=proj,
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()
except Exception:
    git_hash = "unknown"

build_time = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

os.makedirs(os.path.dirname(header), exist_ok=True)
with open(header, "w", encoding="utf-8") as f:
    f.write("#pragma once\n")
    f.write(f'#define BOT_FW_GIT_HASH "{git_hash}"\n')
    f.write(f'#define BOT_FW_BUILD_TIME "{build_time}"\n')