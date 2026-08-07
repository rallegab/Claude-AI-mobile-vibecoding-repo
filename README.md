# Six-DOF Glider Simulator

A small sailplane, simulated in full six degrees of freedom (roll, pitch, yaw,
and 3D position), running entirely in the browser. No build step, no
installs — built with vanilla HTML/CSS/JS and Three.js for rendering.

## Running it

Any static file server works, since the app only uses `<script>` tags (no ES
modules, so no CORS restrictions on `file://`):

```
python3 -m http.server 8000
# or
npx http-server
```

Then open `http://localhost:8000` — or host the folder on GitHub Pages /
Netlify / any static host and open it from your phone's Chrome browser. On
Android Chrome, use the browser menu → **Add to Home Screen** for a
full-screen, app-like experience.

You can also just double-click `index.html` locally — it works over `file://`
too, since Three.js is vendored in `vendor/` rather than loaded from a CDN.

## Controls

- **Left stick** — elevator (pitch) and ailerons (roll)
- **Right stick** — rudder (yaw, left/right) and airbrake (drag the stick down)
- **LAUNCH** — release from an aerotow at altitude (also used to relaunch
  after landing or crashing)
- **CAM** — toggle chase camera / cockpit view
- **TILT** — steer by tilting your phone instead of the left stick (uses the
  device orientation sensor; requires a tap to grant permission on iOS)

The attitude indicator (top center) and HUD (airspeed, altitude, vertical
speed, heading, bank, pitch) update in real time. Watch the stall warning —
pull the nose up too far and the wing stalls, just like a real glider.

## How it works

`app.js` integrates rigid-body 6DOF equations of motion at a fixed 60 Hz
timestep (accumulator pattern, decoupled from render framerate): aerodynamic
lift/drag/side-force from angle of attack and sideslip, control-surface and
rate-damping moments, gravity, and quaternion-based orientation integration.
`vendor/three.min.js` handles only the 3D rendering — the physics has no
external dependencies.

## Project structure

- `index.html` — page shell, HUD markup, start overlay
- `style.css` — HUD, joystick, and attitude-indicator styling
- `app.js` — physics, controls (touch + tilt), Three.js scene, camera, HUD
- `vendor/three.min.js` — vendored Three.js (MIT licensed, see `vendor/THREE_LICENSE.txt`)
- `manifest.json`, `icon.svg` — PWA metadata for "Add to Home Screen"
