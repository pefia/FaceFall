# FaceFall

A 3D cloth simulator in C99. It drops a square of fabric onto a sphere, a cube, or any .obj model you give it, and you can grab it, tear it, throw a ball at it, watch it all in a live preview, or export the whole thing as an mp4 (up to 4K at 120 fps).

![what it does](https://github.com/pefia/FaceFall)

## What it actually simulates

The cloth is an N x N grid of point masses connected by springs:

- structural springs between grid neighbours carry the weight
- shear springs across the diagonals stop it collapsing sideways
- bending springs (skip one particle) resist folding
- strain limiting (Provot 1995) keeps any spring from stretching past 108%, which is what makes it behave like fabric instead of rubber
- self collision on a spatial hash (Teschner et al. 2003), so when the cloth folds over itself the layers stack instead of passing through each other
- per triangle aerodynamics, a flat plate drag model, so a falling sheet parachutes and flutters instead of sinking like it is underwater. Wind is a velocity field that only pushes on faces that actually face it
- contact is a penalty layer (the old Terzopoulos 1987 idea): the collision shell around each object acts like a spring and damper, so the cloth decelerates into the surface over several substeps and settles into a real static equilibrium instead of getting its velocity chopped at a hard boundary. The damping ratio comes from a restitution parameter, which the fabric presets keep near zero because real cloth lands dead. The slider goes up to 1 if you ever want to watch a rubber sheet trampoline off the cube
- collision against arbitrary meshes uses closest point on triangle (from Ericson's Real Time Collision Detection book) plus a swept segment test (Moller-Trumbore) so fast particles cannot tunnel through thin geometry
- optional tearing: a spring yanked past its threshold in a single step snaps, and the crack severs the shear and bending springs crossing it, so pieces genuinely come apart instead of dangling from invisible diagonals. A curtain hanging still never tears, the static load lives in the strain limiter; it takes a snag, a yank, or a ball
- a throwable ball with two way momentum exchange, using the same penalty contact as everything else, so draped cloth can catch and cushion it, and a pinned sheet with tearing on gets punched straight through

A note from getting this wrong first: I spent a while trying to make the cloth visibly bounce off the sphere before doing the momentum accounting and accepting that an inextensible sheet draped over a ball physically cannot trampoline, the hanging skirt anchors it almost instantly. The selftest now enforces realism in both directions: with default fabric the sheet must NOT bounce off the sphere, and with restitution cranked to 0.6 it must rebound visibly off the cube's flat top, which proves the contact layer stores and returns energy correctly.

The integrator is semi implicit Euler with automatic substepping: the step count is chosen from the stiffest thing in the system (springs or the contact layer) so it stays stable when you crank the sliders.

## Building

Windows: run `build.bat`. It expects the toolchain in `tools/`:

- [w64devkit](https://github.com/skeeto/w64devkit) unzipped to `tools/w64devkit`
- [raylib 6.0](https://github.com/raysan5/raylib/releases) (win64 mingw build) unzipped to `tools/raylib-6.0_win64_mingw-w64`
- [ffmpeg](https://ffmpeg.org) at `tools/ffmpeg/bin/ffmpeg.exe` (or just have it on PATH), only needed for video export

Linux: `cc src/*.c -o facefall -O2 -lraylib -lm` with raylib installed.

## Running

- `facefall` opens the settings menu. Arrow keys to change things, P for a live preview, ENTER to render and save the mp4
- `facefall --render` renders with the current defaults and exits, no interaction needed
- `facefall --selftest` runs the physics checks on the CPU and exits

In the preview: R resets the cloth, SPACE pauses, N steps one frame while paused, comma and period are slow motion and fast forward (1/8x to 2x), T is a tension heatmap (blue relaxed, red at the strain limit), left mouse grabs and drags the cloth (scroll reels it in and out), right mouse throws a ball at it, W is wireframe, F is a free fly camera, V exports video, H hides the help.

Things you can change in the menu: cloth resolution (up to 128x128), cloth size, which collider and its size, fabric preset (silk, cotton, denim, leather), stiffness, bend, damping, friction, restitution, self collision on or off, gravity, wind, the surface shine, export resolution, pinning the top edge if you want a curtain instead of a drop, and tearing (off by default, the number is how hard a yank has to be).

If you put an `assets/model.obj` in the project it will collide the cloth against that when you pick the mesh collider. If there is no file it generates a torus so the option always works.

## How the video export works

Every frame gets rendered offscreen at the export resolution, read back from the GPU, and the raw pixels get piped straight into ffmpeg's stdin, which encodes H.264 on the fly. No temp frames on disk. 120 fps output, so the render takes a while, that is the cost of the motion being smooth.

## Credits and honesty

Written in C99 on top of [raylib](https://www.raylib.com). Video encoding by [ffmpeg](https://ffmpeg.org). The physics techniques come from published work: Provot's strain limiting, Teschner's spatial hashing, Ericson's closest point on triangle, Moller-Trumbore intersection, and the penalty contact model going back to Terzopoulos et al. 1987.

I built this with AI assistance (Claude Code) doing a lot of debugging and helping implement the .obj loading. The development story, including the bugs and the dead ends, is in the project devlog.
