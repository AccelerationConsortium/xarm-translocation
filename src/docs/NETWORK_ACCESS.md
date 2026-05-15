# Remote Network Access

This document describes how the xArm services are exposed over the Tailscale VPN so they can be reached from any machine on the network without touching the robot's own IP.

---

## Architecture

```
Remote machine (Tailscale peer)
        │
        │  http://100.64.254.16:<port>
        ▼
Lab PC  (100.64.254.16 — Tailscale IP)
  ├─ netsh portproxy  :18333 ──► 192.168.1.237:18333  (UFactory Studio)
  ├─ netsh portproxy  :8888  ──► 192.168.1.237:18333  (UFactory Studio alias)
  └─ pyxarm service   :8000                            (PyxArm REST / WebSocket API)
        │
        │  TCP 30000
        ▼
xArm controller  (192.168.1.237)
```

---

## UFactory Studio (official UFACTORY control UI)

### Access URLs

| URL | Notes |
|-----|-------|
| `http://100.64.254.16:18333` | Primary — same port as the robot uses |
| `http://100.64.254.16:8888` | Alias — convenient short port |

### How it works

Windows `netsh portproxy` (built-in TCP port forwarding via the IP Helper service) listens on **all interfaces** (`0.0.0.0`) and forwards connections to the robot controller at `192.168.1.237:18333`.

The UFactory Studio frontend is a React SPA. It initialises its WebSocket URL from `window.location.host`, so it automatically connects back to `100.64.254.16:<port>` without any hard-coded IP changes needed.

### Portproxy rules (active)

```powershell
# View active rules
netsh interface portproxy show v4tov4

# Current rules (set 2026-05-15):
# 0.0.0.0:18333  →  192.168.1.237:18333
# 0.0.0.0:8888   →  192.168.1.237:18333
```

### Setup — exact commands run (2026-05-15)

Run once as Administrator to create both forwarding aliases and their firewall guards:

```powershell
# 1. Port-forward :18333 → robot controller (same port, convenient alias)
netsh interface portproxy add v4tov4 `
    listenaddress=0.0.0.0 listenport=18333 `
    connectaddress=192.168.1.237 connectport=18333

# 2. Port-forward :8888 → robot controller (short-port alias)
netsh interface portproxy add v4tov4 `
    listenaddress=0.0.0.0 listenport=8888 `
    connectaddress=192.168.1.237 connectport=18333

# 3. Firewall — allow :18333 from Tailscale subnet only
netsh advfirewall firewall add rule `
    name="xArm UFactory UI :18333 (Tailscale only)" `
    dir=in action=allow protocol=TCP localport=18333 `
    remoteip=100.64.0.0/10

# 4. Firewall — allow :8888 from Tailscale subnet only
netsh advfirewall firewall add rule `
    name="xArm UFactory UI :8888 (Tailscale only)" `
    dir=in action=allow protocol=TCP localport=8888 `
    remoteip=100.64.0.0/10
```

### Firewall rules

Inbound access on both ports is restricted to the Tailscale subnet (`100.64.0.0/10`) only:

```
Rule: "xArm UFactory UI :18333 (Tailscale only)"
Rule: "xArm UFactory UI :8888 (Tailscale only)"
```

### Persistence

`netsh portproxy` rules are stored in the Windows registry under:

```
HKLM\SYSTEM\CurrentControlSet\Services\PortProxy\v4tov4\tcp
```

They are applied automatically by the **IP Helper** (`iphlpsvc`) service at boot. Because the rules are bound to `0.0.0.0` (not the Tailscale IP directly), they survive reboot regardless of the order in which Tailscale and IP Helper start.

### Management commands

```powershell
# View all active portproxy rules
netsh interface portproxy show v4tov4

# Add a new forwarding rule
netsh interface portproxy add v4tov4 `
    listenaddress=0.0.0.0 listenport=<PORT> `
    connectaddress=192.168.1.237 connectport=18333

# Remove a portproxy rule
netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=<PORT>

# Remove a firewall rule
netsh advfirewall firewall delete rule name="<RULE NAME>"

# List all xArm-related firewall rules
netsh advfirewall firewall show rule name=all | Select-String -Context 0,5 "xArm"
```

---

## PyxArm REST / WebSocket API

The `xarm` Windows service runs the PyxArm FastAPI server on port **8000** on all interfaces. It is accessible at:

- `http://100.64.254.16:8000` — REST API
- `ws://100.64.254.16:8000/ws` — WebSocket for real-time status updates
- `http://100.64.254.16:8000/web/` — PyxArm web control UI (served by the web server on port 6001, proxied via FastAPI)
- `http://100.64.254.16:8000/docs` — Interactive OpenAPI documentation

> **Note:** The pyxarm web UI is actually served by a lightweight HTTP server on port **6001**. If accessed directly via the browser (`http://100.64.254.16:6001`), it will proxy API calls to port 8000 automatically. The WebSocket connection bypasses the proxy and connects directly to port 8000.

---

## UFactory Studio — software stack (for reference)

Discovered by inspection of the running service:

| Layer | Technology |
|-------|-----------|
| Web server | Python **Tornado 4.5.3** |
| Frontend framework | **React** (Webpack bundle) |
| UI component library | **Ant Design v3.6.2** |
| HTTP client | **axios** |
| Real-time protocol | **WebSocket** (connects to `window.location.host`) |
| Robot control port | TCP **30000** (xArm SDK protocol, Tornado → robot) |
