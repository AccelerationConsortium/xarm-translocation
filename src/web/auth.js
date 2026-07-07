// Auth banner for the xArm web UI — SDL2 Auth (ac_auth) email-code login.
//
// Mirrors the LaAgenteAnalitica banner pattern: a slim top bar with an
// avatar circle (first letter of the signed-in email), the email, and a
// ghost sign-out button; signed out it offers the email → one-time-code
// flow. All calls go to THIS origin's /auth/* endpoints, which the device's
// FastAPI proxies to the auth sidecar (the sidecar has no CORS and a
// SameSite=Lax cookie, so the browser can't reach it directly).
//
// Identity here is advisory: it never gates the UI. Its one functional
// effect is that main.js stamps the signed-in email into /control/claim's
// `owner`, so claimed_by and the lab audit trail name a real person.
// main.js reads window.labAuth.identity and listens for 'labauth:change'.

(() => {
    // Base-path prefix the panel is served under: "" when hit directly
    // (…/web/…), or e.g. "/xarm5" when routed through the single Caddy edge
    // (…/xarm5/web/…). The API lives one level above /web on the same origin,
    // so /auth/* calls must carry this prefix or the edge routes them to the
    // dashboard. See docs/SINGLE_EDGE_SSO_PLAN.md.
    const _p = window.location.pathname;
    const _i = _p.indexOf('/web');
    const BASE_PATH = _i > 0 ? _p.slice(0, _i) : '';
    const API_BASE_URL = `${window.location.protocol}//${window.location.host}${BASE_PATH}`;

    window.labAuth = { enabled: false, identity: null };

    document.addEventListener('DOMContentLoaded', () => {
        const banner = document.getElementById('auth-banner');
        if (!banner) return;

        const avatar = document.getElementById('auth-avatar');
        const emailLabel = document.getElementById('auth-email');
        const signoutBtn = document.getElementById('auth-signout-btn');
        const signinBtn = document.getElementById('auth-signin-btn');
        const loginRow = document.getElementById('auth-login');
        const emailSelect = document.getElementById('auth-email-select');
        const requestBtn = document.getElementById('auth-request-btn');
        const codeInput = document.getElementById('auth-code-input');
        const verifyBtn = document.getElementById('auth-verify-btn');
        const cancelBtn = document.getElementById('auth-cancel-btn');
        const msg = document.getElementById('auth-msg');

        function setMsg(text, isError = false) {
            msg.textContent = text || '';
            msg.classList.toggle('is-error', Boolean(isError));
        }

        function setIdentity(identity) {
            window.labAuth.identity = identity;
            document.dispatchEvent(new CustomEvent('labauth:change', { detail: identity }));
            if (identity) {
                avatar.textContent = (identity.email || '?').charAt(0).toUpperCase();
                avatar.classList.remove('is-anon');
                emailLabel.textContent = identity.email;
                const viaEdge = identity.via === 'edge';
                emailLabel.title = viaEdge
                    ? `role: ${identity.role || 'user'} — signed in at the lab edge`
                    : `role: ${identity.role || 'user'}`;
                // Behind the edge the session lives at the edge, not this
                // panel, so a local sign-out would be a no-op — hide it.
                signoutBtn.hidden = viaEdge;
                signinBtn.hidden = true;
                loginRow.hidden = true;
            } else {
                avatar.textContent = '?';
                avatar.classList.add('is-anon');
                emailLabel.textContent = 'Not signed in';
                emailLabel.title = '';
                signoutBtn.hidden = true;
                signinBtn.hidden = !loginRow.hidden;
            }
        }

        async function refreshIdentity() {
            try {
                const r = await fetch(`${API_BASE_URL}/auth/me`);
                const data = await r.json();
                setIdentity(data.authenticated ? data.identity : null);
            } catch {
                setIdentity(null);
            }
        }

        function showLogin(show) {
            loginRow.hidden = !show;
            signinBtn.hidden = show || Boolean(window.labAuth.identity);
            codeInput.hidden = true;
            verifyBtn.hidden = true;
            requestBtn.disabled = false;
            codeInput.value = '';
            setMsg('');
        }

        async function loadUsers() {
            emailSelect.innerHTML = '';
            try {
                const r = await fetch(`${API_BASE_URL}/auth/users`);
                if (!r.ok) throw new Error();
                const { users } = await r.json();
                for (const u of users || []) {
                    const opt = document.createElement('option');
                    opt.value = u.email;
                    opt.textContent = u.email;
                    emailSelect.appendChild(opt);
                }
            } catch {
                setMsg('Could not load accounts.', true);
            }
        }

        async function requestCode() {
            const email = emailSelect.value;
            if (!email) return;
            requestBtn.disabled = true;
            setMsg('Sending…');
            try {
                const r = await fetch(`${API_BASE_URL}/auth/request-code`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ email }),
                });
                const data = await r.json().catch(() => ({}));
                if (r.ok) {
                    setMsg('Code emailed — check your inbox.');
                    codeInput.hidden = false;
                    verifyBtn.hidden = false;
                    codeInput.focus();
                } else {
                    setMsg(data.detail || `Failed (HTTP ${r.status}).`, true);
                }
            } catch {
                setMsg('Auth service unreachable.', true);
            } finally {
                requestBtn.disabled = false;
                requestBtn.textContent = 'Resend';
            }
        }

        async function verifyCode() {
            const email = emailSelect.value;
            const code = codeInput.value.trim();
            if (!email || !code) return;
            verifyBtn.disabled = true;
            try {
                const r = await fetch(`${API_BASE_URL}/auth/verify-code`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ email, code }),
                });
                const data = await r.json().catch(() => ({}));
                if (r.ok) {
                    showLogin(false);
                    await refreshIdentity();
                } else {
                    setMsg(data.detail || 'Invalid or expired code.', true);
                }
            } catch {
                setMsg('Auth service unreachable.', true);
            } finally {
                verifyBtn.disabled = false;
            }
        }

        signinBtn.addEventListener('click', async () => {
            showLogin(true);
            await loadUsers();
        });
        cancelBtn.addEventListener('click', () => {
            showLogin(false);
            requestBtn.textContent = 'Send code';
        });
        requestBtn.addEventListener('click', requestCode);
        verifyBtn.addEventListener('click', verifyCode);
        codeInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') verifyCode();
        });
        signoutBtn.addEventListener('click', async () => {
            // End any control session this browser holds BEFORE logging out,
            // so logout truly relinquishes the arm (main.js wires this up).
            try {
                if (window.labAuth.releaseClaimOnSignOut) {
                    await window.labAuth.releaseClaimOnSignOut();
                }
            } catch { /* release is best-effort; proceed to logout */ }
            try {
                await fetch(`${API_BASE_URL}/auth/logout`, { method: 'POST' });
            } catch { /* cookie clear is server-side; refresh regardless */ }
            requestBtn.textContent = 'Send code';
            await refreshIdentity();
        });

        // Bootstrap: only reveal the banner when the device has the auth
        // integration configured, so unconfigured/dev deployments see no chrome.
        (async () => {
            try {
                const r = await fetch(`${API_BASE_URL}/auth/config`);
                const { enabled } = await r.json();
                if (!enabled) return;
                window.labAuth.enabled = true;
                banner.hidden = false;
                signinBtn.hidden = false;
                await refreshIdentity();
            } catch { /* leave banner hidden */ }
        })();
    });
})();
