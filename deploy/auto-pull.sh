#!/usr/bin/env bash
# Pull origin/main if there's a new commit and the working tree is clean.
# Triggered by launchd every minute. Safe to invoke manually.
set -euo pipefail

REPO=/Users/max_server/repo_claude/opti-route
cd "$REPO"

# Bail out if there are uncommitted local edits so we don't blow them away.
if ! git diff --quiet HEAD -- 2>/dev/null; then
  exit 0
fi
if ! git diff --cached --quiet 2>/dev/null; then
  exit 0
fi

git fetch origin main --quiet || exit 0

local_sha=$(git rev-parse HEAD)
remote_sha=$(git rev-parse origin/main)
if [[ "$local_sha" == "$remote_sha" ]]; then
  exit 0
fi

ts() { date '+%Y-%m-%d %H:%M:%S'; }
echo "$(ts)  pulling $remote_sha (was $local_sha)"
git reset --hard origin/main --quiet

# uvicorn --reload picks up backend/*.py changes by itself, but if the new
# commit touched the launchd plist or a runtime dependency, a kick is safer.
launchctl kickstart -k gui/$(id -u)/com.maxshih.opti-route.backend
echo "$(ts)  backend restarted"
