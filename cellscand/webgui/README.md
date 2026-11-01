# cellscan — board web GUI files

These files belong in the board GUI repo (`rfnm/webgui`), not in the app bundle. They are
staged here so the deliverable is self-contained; copy them into a webgui checkout to deploy.

| this file | webgui destination |
|---|---|
| `www/app_cellscan.js` | `www/app_cellscan.js` |
| `daemon/manifests/cellscan.app.json` | `daemon/manifests/cellscan.app.json` (copy of the app's `app.json`) |

Then add one line to `www/index.html`, with the other `app_*.js` panels, **before** `apps.js`:

```html
<script src="app_cellscan.js?v=app1"></script>
```

The panel self-registers onto `window.__RFNM_APP_MODULES` with `id:"cellscan"`; the manifest's
`gui.panel` selects it. No other GUI file changes — `apps.js` renders the card, the RF control
and the activate/stop lease buttons straight from appd's `/v1/state`, exactly as for every other
app. The panel appears under the app once it is the active radio owner, and drives the survey
through the app's own API (`/api/app/cellscan/scan`, `/cancel`, `/status`).
