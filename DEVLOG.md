# FaceFall — Devlog

## 2026-07-04 — The cloth was invisible the whole time (plus: 128x128, adjustable everything)

Today's session started from one complaint that turned out to be three different bugs
wearing a trenchcoat: *"the cloth rendering is really poor, it barely renders the red
cloth, and it sort of clips into the model."* By the end of it the renderer got rebuilt,
the collision system grew a continuous-collision sweep, the resolution ceiling doubled to
128x128, and the settings menu picked up two new rows (cloth size and collider size).
Here's the honest play-by-play, including the part where I made the simulation explode.

---

### Bug 1: "barely renders the red cloth"

My first guess was a lighting problem or z-fighting. Wrong. The actual cause is my
favourite kind of bug: every individual line of code is correct, and the combination is
still broken.

`DrawCloth()` did this:

```c
rlDisableBackfaceCulling();
rlBegin(RL_TRIANGLES);
/* ...all the cloth vertices... */
rlEnd();
rlEnableBackfaceCulling();
```

Looks textbook. But rlgl is a *batched* immediate mode: `rlVertex3f` doesn't draw
anything, it just appends to a CPU-side vertex buffer that gets flushed to the GPU later —
in our case at `EndMode3D()`. Meanwhile `rlDisableBackfaceCulling()` is **not** batched;
in raylib 6.0 it's literally just `glDisable(GL_CULL_FACE)`, effective immediately
(I went and read `rlgl.h` to confirm — line ~1965, there's no flush in there).

So the actual execution order on the GPU was: culling off → *nothing drawn* → culling
back on → **now** the cloth draws, with culling enabled. And a flat cloth sheet built in
the XZ plane has its triangle winding facing *down*, away from the camera looking at it
from above. The whole sheet was backface-culled. What you saw was the occasional fold
whose winding happened to flip toward the camera — hence "barely renders." The cloth was
being simulated perfectly and drawn almost never.

The fix is to bracket the batch manually, flushing at both edges so the state change
applies to *our* geometry and only ours:

```c
rlDrawRenderBatchActive();      /* flush other stuff under the OLD state   */
rlDisableBackfaceCulling();
rlBegin(RL_TRIANGLES); ... rlEnd();
rlDrawRenderBatchActive();      /* draw the cloth NOW, while culling is off */
rlEnableBackfaceCulling();
```

Lesson burned into my brain: in rlgl, **state changes are immediate, geometry is
deferred**. If you touch GL state around batched draws, you flush on both sides or you
get lied to.

While I was in there I redid the shading. The old renderer computed one flat lambert
value per quad, which made the sheet look like a mosaic of tiles. Now there's a proper
per-vertex normal pass — every quad's face normal gets accumulated into its four corner
vertices, normalised, then lit per vertex (two-sided lambert + a hemispheric sky-fill
term so downward-facing folds don't go pitch black) and the GPU interpolates the colours.
Gouraud shading, 1971 called, it works great and needs zero shaders. The checkerboard got
toned way down (a 5% per-quad tint) because at high resolutions the old high-contrast
checker aliased into shimmer soup.

### Bug 2: "clips into the model"

Two separate causes here, and I only understood the second one after breaking things.

**Cause A — the shell was too thin for the mesh.** Collision is tested per *particle*,
but the rendered surface spans straight between particles. Put two particles legally
outside a curved surface (or straddling a sharp edge, like the star model's rim) and the
straight span between them can still cut *through* the surface. The fixed 0.035-unit
collision shell was much smaller than the inter-particle spacing (0.13 at 32x32), so
there was plenty of room for the interpolated surface to dip inside. Fix: the shell now
scales with particle spacing — `clamp(0.35 * spacing, 0.035, 0.15)` — recomputed on every
cloth rebuild. Fine cloth hugs tight, coarse cloth floats a bit higher, and the visible
surface stays out of the collider either way.

**Cause B — thin geometry is a liar.** This one was the real monster, and I only caught
it because I stress-tested at 128x128 against the star .obj. Watching the export:
the cloth hit the star, *tangled through it* — red visible on both sides of a solid
plate — and then the entire sheet slithered off onto the ground like the star wasn't
there. A towel draped over a thin wall should hang, not phase through.

What was happening: the strain limiter (and occasionally a fast substep) can move a
particle across a thin plate in a single update. The collision code then asks "what's the
nearest surface?" — and for a particle that's already crossed the midplane, the nearest
surface is the *far face*. So the resolver would helpfully push the particle out the
wrong side. Every "resolution" was actually a teleport through the model. Any purely
position-based, nearest-surface collision has this failure mode on thin geometry; no
amount of shell thickness fixes it, because the error isn't distance, it's *sidedness*.

The fix is continuous collision: each particle remembers `prevPos`, its position at the
start of the substep (which is post-collision, i.e. guaranteed legal). The mesh collider
now sweeps the `prevPos -> pos` segment against the triangles (Möller–Trumbore, with the
displacement vector un-normalised so the ray parameter *is* the segment parameter, plus a
hair of barycentric slack so particles can't slip through the mathematical seam between
two adjacent triangles). If the segment crossed a triangle, the particle never legally
got to where it is, full stop.

### The explosion (a.k.a. how I learned to stop worrying and revert)

My first swept-collision implementation resolved a crossing the "obvious" way: put the
particle back at the crossing point, offset along the normal by the shell thickness,
on the side it came from. Ran the 128x128 star export again and... t=3 looked *beautiful*
— cloth caught on the star points, draping around the plate on both sides. Then I scrubbed
to t=11 and the cloth was **gone**. Not slid off. Gone. Empty scene, star sitting there
alone like nothing happened.

The simulation had exploded into NaN and raylib just quietly declined to rasterise any
of it. Took me a minute of staring to figure out why, and it's a good one: near the rim
of the star the plate tapers — its local thickness drops below the collision shell. So
"crossing point + shell along the normal" can overshoot straight out the *opposite* face.
Next substep, the sweep sees THAT crossing and throws the particle back. Ping. Pong. Each
bounce teleports positions, springs get yanked, forces spike, and within a couple hundred
substeps the whole sheet is at coordinates that would embarrass an astronomer.

The fix is humbler and it's the classic one: on a sweep hit, just revert to `prevPos` and
kill the inward velocity component. `prevPos` cannot overshoot — it was legal last substep
by construction. It's technically "stickier" (a crossing particle loses its tangential
advance for that substep), but with contact friction in the mix you cannot see the
difference, and it is *unconditionally stable*: reverting can never add energy. New rule
for the wall: **collision responses that can place a particle somewhere new can pump
energy; responses that only restore an old position cannot.**

Re-ran the star test: cloth falls, catches on the points, wraps the plate, hangs there
through all 12 seconds. The blue star tips poke through where the fabric parts around
them, which is exactly what real cloth does.

### Resolution: up to 128x128, and why that needed a spatial grid

The old ceiling was 64x64. The new step table is 2, 4, 8, 16, 24, 32, 48, 64, 96, **128**
— that top end is 16,384 particles, ~97k springs, 32k rendered triangles. Two things had
to change to make it not-a-slideshow:

1. **Memory bounds**: `MAX_GRID` to 128; the `6*N*N` spring bound still holds (worked the
   math: structural 32,512 + shear 32,258 + bending 32,256 = 97,026 < 98,304). Static
   arrays, ~2.5 MB total, nothing dramatic.
2. **Mesh collision cost**: the collider loop was O(particles x triangles) per substep.
   Fine for 1k particles vs 40 triangles; death for 16k particles vs a real scanned .obj.
   The mesh collider now bakes a uniform grid (up to 32 cells per axis, CSR layout — one
   offsets array, one packed triangle-index array, built in two passes) and each particle
   only tests triangles registered in the handful of cells its swept box overlaps.
   A triangle spanning several cells gets tested more than once; harmless, both tests
   keep a min.

Result: the full 12-second 720p export at 128x128 with mesh collision + the sweep test
takes ~14 seconds wall clock. The simulation is still effectively real-time at maximum
everything, which frankly surprised me.

(Also snuck in `#ifndef` guards around `GRID_SIZE` and the default collider so I can
build headless stress-test binaries with `-DGRID_SIZE=128 -DDEFAULT_COLLIDER=COLLIDER_MESH`
and pipe them straight through `--render`. That's how every bug in this entry was
caught — extract frames from the mp4 with ffmpeg, eyeball, fix, repeat.)

### Adjustable cloth size and collider size

The settings menu grew from 6 rows to 8:

* **cloth size** — 1 to 10 world units (0.5 steps). Rebuilds the sheet live so the
  preview behind the menu stays honest.
* **collider size** — x0.3 to x3.0 (0.1 steps). Applies to all three colliders.

This forced a small refactor that was overdue anyway: the sphere/cube used to be
`static const` positions and radii. They're now tiny functions deriving placement from
the scale (cube rests exactly on the ground, sphere hovers its usual hair above it), and
the mesh collider got split into *load once* / *bake at scale* — changing the collider
size re-runs the world-space triangle bake and rebuilds the spatial grid. A shared
`ColliderTopY()` feeds two things that used to be hardcoded:

* **spawn height** — the cloth now spawns `max(6.5, colliderTop + 3)` so a x3 mesh can't
  swallow the spawn point;
* **camera auto-framing** — the fixed camera pulls back by
  `max(1, clothSize/4, colliderTop/2.2)` so nothing pokes out of frame. At default sizes
  that factor is exactly 1, i.e. the original framing is untouched. Free-cam is left
  alone, obviously.

### Housekeeping

`build.bat` had a full paragraph of gibberish prose appended after `endlocal` — no idea
where it came from, but cmd was dutifully trying to execute each sentence as a command
after every build. Deleted.

### Known limits (writing them down so future-me doesn't "discover" them)

* Collision is still particle-based plus a per-particle sweep. A *point* of the collider
  (like a star tip) can in principle spike through the middle of a quad without any
  particle noticing — you need edge/face collision or just more resolution. At 96+ it's
  effectively invisible.
* Coarse cloth (8x8 and below) visibly floats above the collider — that's the
  spacing-scaled shell doing its job, capped at 0.15 so it doesn't get comical. It's a
  trade: hover a little, or clip a lot.
* Cloth-vs-cloth self-collision remains completely unimplemented. Fold it hard enough
  and it will happily pass through itself. Some day.
* The mesh rebake on collider-size change is instant for the 40-tri star but will hitch
  for a heavy .obj if you hold the arrow key. Acceptable; noted.

**Tally for the day:** one rendering bug that was actually a GPU-state-ordering bug, one
clipping bug that was actually two bugs (shell too thin + sidedness on thin geometry),
one self-inflicted energy explosion, one spatial accelerator, two new settings, one
2x resolution ceiling raise, and a build script exorcism. The 128x128 red silk sheet
draping over the star and hanging off its points is genuinely pretty to watch.

---

## 2026-07-05 — GPU shading, real bending & self-collision, per-triangle wind, 4K export

A big feature session, rolled out in four phases: decouple the export resolution from the
window, move rendering onto a real GPU shader, replace the fake bending and add
self-collision, and swap the per-particle wind for a proper per-triangle aerodynamic model
(plus fabric presets and the UI to drive all of it). I also stripped the over-written
comments back to something a human would actually leave, and added a `--selftest` mode so
the new physics has a runnable check that doesn't need a window.

### Phase A — export resolution decoupled from the window

Export used to be `rlReadScreenPixels(SCREEN_W, SCREEN_H)` straight off the backbuffer, so
the video was whatever the 1280×720 window was. Now the window stays at 720p for
interactivity and the render path draws into an offscreen `RenderTexture2D` sized to the
chosen export resolution (720p / 1080p / 1440p / 4K), reads *that* back, and pipes it to
ffmpeg with a matching `-s WxH`. During a render the window shows a scaled blit of the
target so you still see progress.

The one thing I had to get right: `rlReadScreenPixels` calls `glReadPixels` on whatever
framebuffer is currently bound and then flips it to top-left origin. So the readback has to
happen *inside* `BeginTextureMode/EndTextureMode`, while the FBO is still bound — do it
after `EndTextureMode` and you read the window instead. The vertical flip is already
correct because `glReadPixels` on an FBO has the same bottom-left origin as the screen, so
no extra handling versus the old path. All four presets are 16:9, so the camera framing is
identical to the preview and there's no aspect distortion. (Trade-off: the RT isn't
multisampled, so 4K leans on resolution rather than MSAA for its edges. Fine.)

### Phase B — real lighting shader for cloth and collider

The old renderer was CPU Gouraud: accumulate face normals per vertex, compute a lambert
value on the CPU, feed vertex colours through rlgl's immediate-mode batch. It worked but it
couldn't do specular, rim, or anything view-dependent, and the collider was a flat unlit
blob.

The cloth is now a real indexed `Mesh` (positions + normals + uvs), re-uploaded when the
resolution changes and streamed every frame with `UpdateMeshBuffer`, drawn with a custom
GLSL 330 shader. The shader does diffuse + Blinn-Phong specular, a Fresnel-ish rim term for
silhouette pop, a procedural weave normal perturbation, and — for the cloth only — a
subsurface backlight so thin fabric glows when the light is behind it. Light direction,
camera position, and specular/roughness are uniforms. The same shader lights the collider
(sphere/cube are generated meshes drawn with a scale/translate transform; the .obj model
just gets its material's shader swapped), so everything in the scene is consistently lit.

Difficulties:

* **Immediate mode can't carry normals.** rlgl's batch only has position/uv/colour, so
  there was no way to feed per-vertex normals to a custom shader through `rlVertex3f`. That's
  what forced the move to an actual `Mesh` + `DrawMesh`. Once that was in place the old
  invisible-cloth flush dance disappeared too — `DrawMesh` issues its own draw call, it
  isn't deferred to `EndMode3D`, so the two-sidedness is just `rlDisableBackfaceCulling()`
  around the call plus flipping the normal toward the viewer in the fragment stage.
* **`viewPos` isn't auto-bound.** raylib sets `mvp`/`matModel`/`matNormal`/`colDiffuse` for
  you in `DrawMesh`, but the view position has to be pushed manually every frame via
  `SetShaderValue` — the specular and rim are both wrong (static) if you forget it.

### Phase C — dihedral bending + self-collision

The old "bending" was a linear spring from particle *i* to *i+2*. It resists the straight-
line distance shortening, which couples bending to in-plane stretch and, worse, only acts
along the two grid axes — so every crease wanted to line up with the grid.

Replaced it with a proper **dihedral bending constraint** (Müller et al., Position Based
Dynamics). At build time I generate a hinge for every interior shared edge of the triangle
mesh — three families: each quad diagonal, and each vertical and horizontal grid edge — with
the two "wing" vertices worked out from the triangulation. Each substep a couple of PBD
passes drive every fold angle back toward its (flat) rest angle. Because it resists the
*angle* and not a distance, folds can crease at any orientation without fighting stretch,
and the fabric stops looking like a bent wire grid.

Deriving the wing vertices for the two grid-edge families was the fiddly part: the quad
triangulation splits along the *i00–i11* diagonal, so the two triangles flanking a vertical
grid edge are asymmetric (one wing at row *j*, the other at *j+1*). That's fine for a
dihedral constraint — the rest angle is measured from the actual flat geometry per hinge, so
the asymmetry is baked into `phi0` and the constraint just holds it there.

**Self-collision** is a spatial hash over the particles: hash each particle into a grid cell
sized to the interaction radius, then for each particle test the 27 neighbouring cells and
push apart any pair closer than `2*radius`, split by inverse mass. The radius is kept under
one cell of rest spacing so resting neighbours never trigger it — only cloth that has
actually folded onto itself does, which means no topology bookkeeping to exclude structural
neighbours. To bound the cost it runs once per *frame* (two relaxation iterations), not per
substep, followed by a final collider pass to keep the just-separated sheet out of the
object. That's the ceiling: it's O(n) and fine to ~96², heavier at 128².

### Phase D — per-triangle aerodynamics, fabric presets, UI

Wind used to be a constant per-particle force with a sine ripple. Now every render triangle
gets an aerodynamic force from the airflow across it: relative airflow = wind velocity minus
the triangle's velocity, and the force is a normal (form-drag/pressure) term scaled by how
broadside the face is to the flow, plus a tangential skin-drag term, distributed to the
three vertices. A face edge-on to the wind barely feels it; a broadside face catches it —
which is exactly what produces billow and lift, and it also gives physically-shaped air
resistance when there's no wind at all (a flat sheet falling gets a parachute effect the old
`-c*v` drag couldn't). The residual linear drag is kept tiny just for numerical calm.

Four **fabric presets** (silk / cotton / denim / leather) bundle stiffness, bend, damping
and friction. The settings screen grew from 8 rows to 16: preset, the four physics params
individually (so you can tweak after picking a preset), self-collision toggle, specular,
roughness, and export resolution.

### The behavioural surprise: cloth pouring off the sphere

First full render with the new physics: the cloth draped over the sphere beautifully at
mid-fall, then by t≈10s slid completely off and crumpled on the ground next to it. The
self-test proved it wasn't an energy explosion (it settles into a crumpled heap with real
self-collision volume, not NaNs) — it's the *unstable-equilibrium* problem. A flat sheet
dropped dead-centre on a sphere is balanced on a knife edge; the old stiff bend springs made
the cloth rigid enough to balance there, but the new soft angle-based bending lets it
conform and pour off one side, which is what real light fabric actually does. Friction is
applied per substep so it's not the culprit — gravity just wins the torque battle.

The fix was tuning, not code: I raised the default presets' bend and friction so cotton
catches and holds a proper hood-shaped drape (and still slides down to pool at the base over
time, which looks right), while silk stays soft enough to pour off for anyone who wants that.
The lesson worth writing down: **with realistic soft bending, "cloth on a sphere" is a
genuinely unstable configuration — the fabric stiffness/friction preset, not a bug, decides
whether it holds.**

### Housekeeping — comments and a self-test

Stripped the multi-paragraph teaching-tone comments back to terse notes that state the
non-obvious gotcha and nothing else. Kept the ones that actually encode hard-won knowledge
(the rlgl flush ordering, popen binary mode, the thin-geometry sidedness revert).

Added `facefall --selftest`: a pure-CPU, no-GL run that asserts the three new systems do
what they claim — bending reduces total fold deviation across all hinges (23.2 → 0.42),
self-collision separates two overlapping particles to the minimum distance, and aero exerts
a downwind force on a still triangle. It reuses the real functions, so it's a real check, and
it runs in one gcc invocation with no framework.

### Verification

`--selftest` green; full 12s 1080p render of the default (cotton / sphere / 32²) exports a
clean 2 MB mp4 in ~18s wall time, frames pulled with ffmpeg confirm: cloth falls, drapes
over the sphere as a wrinkled hood with specular sheen along the creases, self-collision
keeps the folds voluminous, and the collider is properly lit. Builds with `-Wall -Wextra`,
zero warnings, all GPU resources unloaded cleanly at exit.

### Known limits

* Self-collision is per-frame O(n) hash — fine to ~96², a hitch at 128².
* The export render target isn't multisampled; 4K relies on resolution for edge quality.
* Cloth-on-sphere stability is preset-dependent (see above); it's physics, not a bug.
* Dihedral bending near a fully-folded 0° crease loses authority (the `sin(phi)` factor
  goes to zero) — a known degeneracy of angle-based bending, invisible in practice.

### Correction, same day — reverted the physics changes (they made it worse)

Shipped the four phases, then watched a real drape and the physics was worse, not better:
the cloth clipped through itself, and it would randomly get flung. The additions from Phase
C/D were the cause, and the root problem was mixing paradigms:

* **Self-collision** ran once per frame with a small radius. It was too weak to actually
  stop self-intersection (so it still clipped), and when it *did* act on a deep overlap it
  shoved particles apart in position space with no velocity feedback — the springs then saw
  a suddenly-stretched edge and snapped, flinging the sheet. Position corrections that don't
  update velocity fight a force-based integrator.
* **Dihedral bending** replaced the stiff bend springs with soft, position-based angle
  constraints. That made the drape floppy instead of the satisfying stiffer hang, and being
  position-based it also fought the force-based springs and added jitter.
* **Per-triangle aero** changed the fall/settle feel away from what worked.

So I reverted the physics to the proven force-based model: structural + shear + **bend
springs** (bend stiffness = a fraction of k, driven by the fabric preset), per-particle
gusting wind, linear air drag, symplectic Euler, strain limiting as the only position pass.
No self-collision, no angle-based bending. The lesson: don't bolt position-based constraints
onto a force-based integrator without velocity reconciliation, and don't ship a physics
change off a single pretty mid-fall frame — watch the whole settle.

**Kept** (unrelated to the instability, and working well): the GPU cloth/collider shader
(diffuse + specular + rim + weave + subsurface), the offscreen 1080p/1440p/4K export target,
the fabric presets (now mapping to the bend-spring model), and the settings UI. `--selftest`
now runs a stability check instead — 120 frames of drape onto the sphere, asserting positions
stay finite and bounded (no fling) and structural springs stay within the strain limit.

Verified: default cotton/sphere render drapes symmetrically over the ball and *stays* there,
stable through all 12 s; self-test green (drape settles to y in [0.83, 2.21], worst stretch
1.082 vs 1.080 limit); builds with `-Wall -Wextra`, zero warnings.

---

## 2026-07-05 — Making it actually bounce (three failed attempts, one physics lesson, 120 fps)

Today's brief sounded simple: *"properly simulate real falling and physics — even the cloth
bounces when it hits the object. Computational cost is fine, render can take longer, higher
FPS too."* What followed was the most instructive physics debugging session of this project.
Also: last session's lesson — position corrections need velocity reconciliation — turned out
to be the literal key that made everything work this time.

### Attempt 1: just reflect the velocity (fails: lattice ringing)

The obvious restitution implementation: on contact, reflect the inward normal velocity scaled
by `e`, with a Box2D-style velocity threshold so resting contact doesn't jitter. Built it,
selftest said rebound = **+0.007 units**. Even at `e = 1.0`: +0.029. Effectively nothing.

A per-frame trace of the centre particle showed why. The reflection *works* — one frame after
impact the particle is moving up at 6.17 m/s. Two frames later it's at −3.06. Then +4.47,
−0.25, +2.69… flipping sign every ~2.5 frames. That's the spring natural frequency
(ω = √(k/m) = 300 rad/s ≈ 48 Hz). On a curved collider, neighbours strike at slightly
different instants, so each particle's reflection just rings *its own lattice node* against
its still-seated neighbours until the springs eat the energy. Incoherent bounces thermalise.

### Attempt 2: bank the impulses and smear them (fails: it's a cushion, not a bounce)

Inspired by Bridson et al. 2002 (impulse-based cloth contacts): absorb the impact, bank the
restitution impulse, then pay it out smeared over a 5×5 grid neighbourhood so the patch
rebounds coherently. Result: **+0.005** — *worse*. The smeared upward velocity slows
neighbours that haven't landed yet, so their own impacts bank less, geometrically decaying.
The mechanism converts bounce into soft-landing. Diffusion is damping. Deleted it.

### Attempt 3: penalty layer (works, after a sizing bug)

The right tool turned out to be the oldest one in the book — the penalty method (Terzopoulos
et al. 1987). The collision shell becomes a spring-damper layer: `kc` sized so a reference
impact is arrested inside the shell, damping ratio derived from restitution via the
logarithmic decrement (ζ = −ln e / √(π² + ln²e)), applied as velocity impulses which is
identical to forces under semi-implicit Euler. Resting contact becomes a static equilibrium
(the cloth floats mg/kc — a fraction of a millimetre — into the shell), so the jitter
threshold could be deleted entirely.

First version still didn't bounce: I sized the spring to arrest CONTACT_VREF within the
*whole* shell, but the stopping distance is v/ω = 5.5 cm and the backstop projection sits
at a quarter shell = 4.6 cm available. Every impact punched through to the backstop, which
absorbs inelastically. Resized to arrest within *half* the shell; StepFrame now counts the
contact ω in its substep budget so the layer never outruns the integrator.

### The phantom velocity, or: last session's lesson collects its debt

Next trace was the best one: after impact the centre particle's velocity reads +2 to +3 m/s
*for twenty straight frames* while its position doesn't move a millimetre. Strain limiting
(a position-space clamp) was pinning the particle to its seated neighbours every substep
while its velocity kept insisting it was flying upward — and contact damping happily
computed forces from that phantom velocity. Exactly the position/velocity divergence that
blew up last session's self-collision experiment, wearing a different hat.

Fix: the limiter now applies the matching velocity impulse (dv = dx/h) with every positional
correction — the same bookkeeping PBD does when it rebuilds velocities from corrected
positions. The recoil momentum stops evaporating and instead travels down the sheet as a
visible wave. In the render you can see the skirt *flare upward* off the sphere at t≈1.3 s.

### The honest physics limit

With everything fixed, the sphere-top rebound ceiling is still only ~3 cm even at e = 1.0.
That's not a bug — an inextensible sheet draped over a ball physically cannot trampoline:
the hanging skirt (78% of the mass, outside the sphere's silhouette) anchors the contact
patch through the strain limit almost immediately. Flat impacts are a different story: the
cube's whole top face strikes coherently while the skirt is still airborne, and the sheet
rebounds **+8.3 cm** — plainly visible. The selftest now has two phases: sphere (assert
contact recoil velocity > 0.8 m/s, then a sane settle) and cube (assert visible positional
rebound > 5 cm). Test what the physics promises, not what you wish it promised.

### Aerodynamics, this time for real

Last session's per-triangle aero got reverted for changing the feel; today it went in
properly: flat-plate pressure drag per render triangle (F = ½ρCd·A·|v|² projected on the
face normal, split over the three corners), with the coefficient normalised by total cloth
mass so the same AERO_DRAG value behaves identically at every grid resolution — a flat
sheet's terminal velocity is √(g/AERO_DRAG) regardless of N. Wind became a velocity field
feeding the same model, so it only pushes on faces that actually face it. The falling sheet
now *parachutes* with a billowed canopy (t≈0.95 s in the render) instead of sinking like a
plate through honey. The old linear drag survives only as a small numerical-calm term.

### Self-collision, second attempt, welcome back

Same idea as the reverted one — particle-particle separation on a Teschner spatial hash —
but this time the positional push comes with the closing-velocity kill along the pair axis,
so it cooperates with the force integrator instead of fighting it (the whole reason it
failed before). Folds now stack in layers on top of the sphere instead of raining through
each other. It runs every substep and is the most expensive thing in the sim; there's a
settings-menu toggle for it.

### 120 fps

FRAME_DT is now locked to RENDER_FPS = 120: the export is high-speed video and the
integration step halved for free. The 60 Hz interactive preview steps the sim twice per
displayed frame to stay real-time. Render takes roughly twice as long. Worth it — the
impact wave reads beautifully at 120.

### Repo prep

Wrote a README (build instructions, what's simulated, credits — raylib, ffmpeg, Ericson,
Möller–Trumbore, Provot, Teschner, Terzopoulos, and the AI-assistance disclosure), added a
.gitignore (tools/, build+render output, .claude/), and initialised the git repo pointed at
github.com/pefia/FaceFall.

**Verified:** two-phase selftest green (sphere recoil +2.85 m/s; drape settles y ∈
[0.156, 2.201], worst stretch exactly at the 1.080 limit; cube rebound +0.083); full
12 s / 120 fps / 1080p render via `--render` + frame extraction — parachute at 0.95 s,
impact splash at 1.15 s, skirt flare (the bounce wave) at 1.30 s, layered folds at 1.5 s,
clean anchored drape at 11.5 s, zero clipping; scratch build `-DGRID_SIZE=128
-DDEFAULT_COLLIDER=COLLIDER_MESH` compiles with `-Wall -Wextra`, zero warnings, selftest
green.

**Known limits:** the sphere-top bounce is physically capped (see above) — if you want
drama, drop it on the cube or crank restitution and watch the skirt; self-collision is
particle-based, so extremely sharp folds thinner than the particle spacing can still kiss;
friction is the old aggressive per-substep retention model, which reads more like felt than
silk on steep surfaces.
