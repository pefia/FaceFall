# FaceFall

A 3D cloth simulator in one C file. It drops a square of fabric onto a sphere, a cube, or any .obj model you give it, and you can watch it in a live preview or export the whole thing as an mp4 (up to 4K at 120 fps). Made for Hack Club Stardance.

## I RECCOMEND 64X64 AND UP FOR REALISM AND TESTING, 128X128 BEING THE BEST

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

A note from getting this wrong first: I spent a while trying to make the cloth visibly bounce off the sphere before doing the momentum accounting and accepting that an inextensible sheet draped over a ball physically cannot trampoline, the hanging skirt anchors it almost instantly. The selftest now enforces realism in both directions: with default fabric the sheet must NOT bounce off the sphere, and with restitution cranked to 0.6 it must rebound visibly off the cube's flat top, which proves the contact layer stores and returns energy correctly.

The integrator is semi implicit Euler with automatic substepping: the step count is chosen from the stiffest thing in the system (springs or the contact layer) so it stays stable when you crank the sliders.

## Building

Windows: Install the zip, trun `build.bat`, then open the newly .exe file. It expects the toolchain in `tools/` which is now included, but is 100MB+. It includes:

- [w64devkit](https://github.com/skeeto/w64devkit) unzipped to `tools/w64devkit`
- [raylib 6.0](https://github.com/raysan5/raylib/releases) (win64 mingw build) unzipped to `tools/raylib-6.0_win64_mingw-w64`
- [ffmpeg](https://ffmpeg.org) at `tools/ffmpeg/bin/ffmpeg.exe` (or just have it on PATH), only needed for video export


## Running

- `facefall` opens the settings menu. Arrow keys to change things, P for a live preview, ENTER to render and save the mp4
- `facefall --render` renders with the current defaults and exits, no interaction needed
- `facefall --selftest` runs the physics checks on the CPU and exits

In the preview: R resets the cloth, SPACE pauses, W is wireframe, F is a free fly camera, V exports video, H hides the help.

Things you can change in the menu: cloth resolution (up to 128x128), cloth size, which collider and its size, fabric preset (silk, cotton, denim, leather), stiffness, bend, damping, friction, restitution, self collision on or off, gravity, wind, the surface shine, export resolution, and pinning the top edge if you want a curtain instead of a drop.

If you put an `assets/model.obj` in the project it will collide the cloth against that when you pick the mesh collider. If there is no file it generates a torus so the option always works.

## How the video export works

Every frame gets rendered offscreen at the export resolution, read back from the GPU, and the raw pixels get piped straight into ffmpeg's stdin, which encodes H.264 on the fly. No temp frames on disk. 120 fps output, so the render takes a while, that is the cost of the motion being smooth.

## Credits and honesty

Written in C99 on top of [raylib](https://www.raylib.com). Video encoding by [ffmpeg](https://ffmpeg.org). The physics techniques come from published work credited in the source comments: Provot's strain limiting, Teschner's spatial hashing, Ericson's closest point on triangle, Moller-Trumbore intersection, and the penalty contact model going back to Terzopoulos et al. 1987.

I built this with AI assistance (Claude Code) doing a lot of debugging and helping implement the .obj loading. The development story, including the bugs and the dead ends, is in my Stardance devlogs.
