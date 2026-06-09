# Deployment — Mac Studio + Cloudflare Tunnel

Live at **https://opti-route.max-the-solution.com**

Same architecture pattern as goblins-nest:

```
Browser
   ↓ https
Cloudflare edge
   ↓ (Cloudflare Tunnel)
cloudflared (Mac Studio, launchd-managed)
   ↓ http://localhost:8765
uvicorn → FastAPI (Mac Studio, launchd-managed)
   ↓
serves /, /about, /static/*, /api/*
```

Both the API and the static frontend come out of the same FastAPI process,
so there's no CORS to configure and only one hostname to manage.

---

## Files in this folder

| File | Where it actually lives on the Mac |
|---|---|
| `cloudflared.yml` | `~/.cloudflared/config-opti-route.yml` |
| `com.maxshih.opti-route.backend.plist` | `~/Library/LaunchAgents/com.maxshih.opti-route.backend.plist` |
| `com.maxshih.opti-route.tunnel.plist` | `~/Library/LaunchAgents/com.maxshih.opti-route.tunnel.plist` |
| `com.maxshih.opti-route.auto-pull.plist` | `~/Library/LaunchAgents/com.maxshih.opti-route.auto-pull.plist` |
| `auto-pull.sh` | invoked from the auto-pull plist; stays in-repo |

The originals here are the source of truth; the deployed copies should be
kept in sync (or symlinked).

---

## One-time setup (already done)

```bash
# 1. Create the tunnel (cloudflared was already authed for max-the-solution.com
#    via goblins-nest, so no browser auth needed).
cloudflared tunnel create opti-route
# → outputs UUID 657baa2f-0322-466a-b6d4-8b6977ccaca5 and writes creds JSON

# 2. Route DNS to it.
cloudflared tunnel route dns opti-route opti-route.max-the-solution.com

# 3. Copy deploy/ files to their real locations and load.
cp deploy/cloudflared.yml                          ~/.cloudflared/config-opti-route.yml
cp deploy/com.maxshih.opti-route.backend.plist     ~/Library/LaunchAgents/
cp deploy/com.maxshih.opti-route.tunnel.plist      ~/Library/LaunchAgents/

launchctl load -w ~/Library/LaunchAgents/com.maxshih.opti-route.backend.plist
launchctl load -w ~/Library/LaunchAgents/com.maxshih.opti-route.tunnel.plist
```

The plists have `RunAtLoad=true` and `KeepAlive=true`, so:

- they start on every login / boot
- they restart automatically on crash (with a 10s throttle to avoid loops)

---

## Day-to-day operations

```bash
# status
launchctl list | grep opti-route

# tail logs
tail -f /Users/max_server/repo_claude/opti-route/logs/{backend,tunnel}.{out,err}.log

# restart the backend after a code change
launchctl kickstart -k gui/$(id -u)/com.maxshih.opti-route.backend

# restart the tunnel (rarely needed)
launchctl kickstart -k gui/$(id -u)/com.maxshih.opti-route.tunnel

# stop everything
launchctl unload ~/Library/LaunchAgents/com.maxshih.opti-route.backend.plist
launchctl unload ~/Library/LaunchAgents/com.maxshih.opti-route.tunnel.plist
```

---

## Smoke test

```bash
# local
curl -s -o /dev/null -w "%{http_code}\n" http://localhost:8765/api/scene
# public
curl -s -o /dev/null -w "%{http_code}\n" https://opti-route.max-the-solution.com/api/scene
```

Both should return `200`.

---

## Auto-deploy on `git push`

Two layered mechanisms keep production in sync with the repo with **zero
manual steps**:

### 1. uvicorn `--reload` (local edits)
The backend plist runs `uvicorn … --reload --reload-dir backend`.  Any
saved `.py` file under `backend/` triggers an in-process reload — visible
on the public URL within ~1 second.  Frontend files are served statically
by FastAPI so they're picked up on the next request, no reload required.

### 2. `auto-pull.sh` timer (remote pushes)
`deploy/auto-pull.sh` runs every 60 s under launchd
(`com.maxshih.opti-route.auto-pull`). Logic:

  - bail out if working tree has uncommitted edits (so it never blows
    away an in-progress local change)
  - `git fetch origin main`
  - if `HEAD != origin/main`: `git reset --hard origin/main` and
    `launchctl kickstart` the backend

Together they cover both workflows:

  - editing on the Mac Studio → uvicorn reloads instantly
  - pushing from elsewhere → poll picks it up within a minute

Logs at `logs/auto-pull.log`.

---

## Future work

- The frontend is currently coupled to the same hostname so it goes dark
  when the Mac Studio is off. If that ever becomes a problem, move
  `frontend/` to Cloudflare Pages (its own subdomain), keep the tunnel
  for `/api/*` only, and add `CORSMiddleware` in `backend/main.py`.
- A `static-no-backend` branch exists with a vanilla-JS port of the KSP
  solver (no MIP). It can be deployed straight to Cloudflare Pages as a
  cold-start-free fallback.
