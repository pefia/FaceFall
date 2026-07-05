/*******************************************************************************
*   FaceFall — 3D mass-spring cloth simulator (raylib, C99, single file)
*
*   N x N cloth (structural/shear/bending springs, strain limiting, particle
*   self-collision on a spatial hash, per-triangle flat-plate aerodynamics,
*   impact restitution) dropped onto a sphere / cube / .obj collider, shaded
*   on the GPU and exported to mp4 via ffmpeg at up to 4K / 120 fps.
*
*   Build:  build.bat            (Windows, bundled w64devkit + raylib)
*           cc facefall.c -o facefall -O2 -lraylib -lm    (Linux)
*   Run:    facefall             interactive
*           facefall --render    render + export, then exit
*           facefall --selftest  run physics self-checks, then exit
*******************************************************************************/

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

/* Windows _popen needs BINARY mode ("wb"): text mode turns every 0x0A in the
 * raw pixel stream into 0x0D 0x0A and shears the video. */
#ifdef _WIN32
    #define POPEN(cmd, mode)  _popen(cmd, mode)
    #define PCLOSE(fp)        _pclose(fp)
    #define POPEN_WRITE_MODE  "wb"
    #define DEVNULL           "NUL"
#else
    #define POPEN(cmd, mode)  popen(cmd, mode)
    #define PCLOSE(fp)        pclose(fp)
    #define POPEN_WRITE_MODE  "w"
    #define DEVNULL           "/dev/null"
#endif

/* ============================================================================
 *  TUNABLES
 * ========================================================================= */

#define SCREEN_W            1280     /* interactive preview window            */
#define SCREEN_H            720

/* Export resolutions, decoupled from the preview window. All 16:9 so the
 * camera framing is identical to the preview. */
typedef struct { const char *name; int w, h; } ExportRes;
static const ExportRes EXPORT_RES[] = {
    { "720p",  1280, 720  },
    { "1080p", 1920, 1080 },
    { "1440p", 2560, 1440 },
    { "4K",    3840, 2160 },
};
#define NUM_EXPORT_RES (int)(sizeof(EXPORT_RES)/sizeof(EXPORT_RES[0]))

/* Default cloth resolution; re-tessellated at runtime through RESOLUTION_STEPS.
 * (#ifndef so test builds can override with -DGRID_SIZE=128.) */
#ifndef GRID_SIZE
#define GRID_SIZE           32
#endif
#define MAX_GRID            128                   /* hard upper bound        */
#define MAX_PARTICLES       (MAX_GRID * MAX_GRID)
#define MAX_SPRINGS         (6 * MAX_GRID * MAX_GRID)   /* struct+shear+slack */

#define DEFAULT_CLOTH_SIZE  4.0f     /* world-space size of the cloth sheet  */
#define MIN_CLOTH_SIZE      1.0f
#define MAX_CLOTH_SIZE      10.0f
#define CLOTH_SPAWN_Y       6.5f     /* spawn height above the collider      */
#define CLOTH_THICKNESS     0.035f   /* minimum collision shell per particle */

/* Constant per-particle mass (not total/N^2): keeps the integrator's
 * stability budget the same across resolutions. */
#define PARTICLE_MASS       0.05f

/* Physics defaults. Stiffness / bend / damping / friction are also live
 * fabric-preset parameters (see g_* below and FABRICS[]). */
#define DEFAULT_STIFFNESS   4500.0f  /* Hooke spring constant k (N/m)        */
#define DEFAULT_BEND        0.35f    /* bend springs as a fraction of k      */
#define DEFAULT_DAMPING     1.2f     /* damping along the spring axis        */
#define DEFAULT_FRICTION    0.20f    /* tangential velocity kept on contact  */
#define DEFAULT_GRAVITY     9.81f

/* Impact response. Restitution is the fraction of the normal impact speed
 * returned on a bounce; impacts slower than BOUNCE_THRESHOLD are absorbed
 * outright so resting contact cannot jitter forever — the velocity-threshold
 * trick every rigid-body engine uses (cf. Box2D's b2_velocityThreshold). */
/* (#ifndef so scratch builds can sweep these, like GRID_SIZE.) */
#ifndef DEFAULT_RESTITUTION
#define DEFAULT_RESTITUTION 0.30f
#endif

/* Contact-spring sizing: the penalty layer is stiff enough to arrest a
 * CONTACT_VREF impact within HALF the collision shell, comfortably clear of
 * the quarter-shell backstop (stopping distance is v/omega, so omega must be
 * at least v / (shell/2)). See ContactPenalty. */
#define CONTACT_VREF        7.0f     /* m/s */
#define CONTACT_OMEGA       (2.0f * CONTACT_VREF / g_collisionShell)

/* Aerodynamics. AERO_DRAG is normalised against total cloth mass inside
 * PhysicsSubstep, so the same value gives the same behaviour at every grid
 * resolution: a flat sheet's terminal velocity is sqrt(g / AERO_DRAG).
 * AIR_DRAG is a small residual per-particle drag for numerical calm. */
#define AERO_DRAG           0.18f    /* flat-plate pressure-drag coefficient */
#define AIR_DRAG            0.06f    /* linear drag, F = -c*v                */

#ifndef MAX_STRETCH
#define MAX_STRETCH         1.08f    /* springs may not exceed 108% rest len */
#endif
#define STRAIN_ITERATIONS   3        /* strain-limiting passes per substep   */

#define FRAME_DT            (1.0f / (float)RENDER_FPS)  /* timestep = video rate */

#define GROUND_Y            0.0f
#define STICK_SPEED         0.30f    /* tangential speed below which a contact
                                        sticks (static friction) so a hanging
                                        skirt doesn't drag the sheet off      */

/* Render/export settings. The sim's fixed timestep is locked to RENDER_FPS,
 * so raising it both smooths the video and halves the integration step; the
 * 60 Hz interactive preview steps the sim RENDER_FPS/PREVIEW_FPS times per
 * displayed frame to stay real-time. */
#define RENDER_SECONDS      12
#define RENDER_FPS          120
#define PREVIEW_FPS         60
#define RENDER_TOTAL_FRAMES (RENDER_SECONDS * RENDER_FPS)
#define OUTPUT_FILENAME     "facefall_output.mp4"

/* Places we look for an ffmpeg binary, in order. */
static const char *FFMPEG_CANDIDATES[] = {
    "ffmpeg",                                  /* on PATH                    */
    "tools\\ffmpeg\\bin\\ffmpeg.exe",          /* bundled with this project  */
    "tools/ffmpeg/bin/ffmpeg",
};

static const int RESOLUTION_STEPS[] = { 2, 4, 8, 16, 24, 32, 48, 64, 96, 128 };
#define NUM_RESOLUTION_STEPS (int)(sizeof(RESOLUTION_STEPS)/sizeof(RESOLUTION_STEPS[0]))

/* ============================================================================
 *  TYPES
 * ========================================================================= */

typedef enum { SPRING_STRUCTURAL, SPRING_SHEAR, SPRING_BENDING } SpringType;

typedef struct Particle {
    Vector3 pos;
    Vector3 prevPos;  /* last known legal (post-collision) position; the mesh
                         collider sweeps prevPos->pos so thin geometry can't be
                         tunnelled through in one substep */
    Vector3 vel;
    Vector3 force;    /* accumulator, cleared every substep */
    float   invMass;  /* 0 => pinned/immovable              */
} Particle;

typedef struct Spring {
    int        a, b;      /* particle indices                   */
    float      restLen;   /* length at which the spring is calm */
    SpringType type;
} Spring;

/* A fabric preset: one row of feel-defining parameters. */
typedef struct Fabric {
    const char *name;
    float stiffness;      /* stretch resistance (N/m)              */
    float bend;           /* bend-spring stiffness, fraction of k  */
    float damping;        /* spring-axis damping                   */
    float friction;       /* contact tangential retention          */
    float restitution;    /* impact bounciness [0,1]               */
} Fabric;

static const Fabric FABRICS[] = {
    /*  name       stiffness  bend   damping  friction  restitution */
    { "silk",       4000.0f, 0.15f,  1.0f,    0.15f,    0.10f },
    { "cotton",     4500.0f, 0.35f,  1.2f,    0.25f,    0.30f },
    { "denim",      7000.0f, 0.60f,  1.8f,    0.40f,    0.45f },
    { "leather",   10000.0f, 0.85f,  2.5f,    0.55f,    0.55f },
};
#define NUM_FABRICS (int)(sizeof(FABRICS)/sizeof(FABRICS[0]))

typedef enum { COLLIDER_SPHERE = 0, COLLIDER_CUBE, COLLIDER_MESH, COLLIDER_COUNT } ColliderKind;

typedef enum { MODE_SETTINGS, MODE_PREVIEW, MODE_RENDER } AppMode;

/* Loaded .obj baked into world-space triangles, plus a uniform CSR grid so a
 * particle only tests triangles in the few cells around it (broad phase). */
typedef struct MeshCollider {
    Vector3    *tri;        /* 3 vertices per triangle, flat array */
    int         triCount;
    BoundingBox bounds;     /* world-space AABB, inflated for the broad-phase */
    Model       model;      /* GPU copy for drawing                          */
    bool        fromFile;   /* true if assets/model.obj was actually found   */
    int        *gridStart;  /* CSR offsets, gridNx*gridNy*gridNz + 1 entries */
    int        *gridItems;  /* triangle indices, indexed via gridStart       */
    int         gridNx, gridNy, gridNz;
    Vector3     gridMin;    /* tight AABB min = grid origin                  */
    Vector3     cellSize;   /* per-axis cell dimensions (always > 0)         */
} MeshCollider;

/* ============================================================================
 *  GLOBAL SIMULATION STATE
 * ========================================================================= */

static Particle g_particles[MAX_PARTICLES];
static Spring   g_springs[MAX_SPRINGS];
static int      g_gridN       = GRID_SIZE;
static int      g_springCount = 0;

static float g_stiffness   = DEFAULT_STIFFNESS;
static float g_bend        = DEFAULT_BEND;         /* bend stiffness [0,1]   */
static float g_damping     = DEFAULT_DAMPING;
static float g_friction    = DEFAULT_FRICTION;
static float g_restitution = DEFAULT_RESTITUTION;
static float g_gravity     = DEFAULT_GRAVITY;
static float g_windPower   = 0.0f;
static bool  g_selfCollide = true;                 /* cloth-on-cloth contact */
static bool  g_pinTopEdge  = false;
static bool  g_paused     = false;
static bool  g_wireframe  = false;
static bool  g_showHelp   = true;
static int   g_fabric     = 1;                     /* index into FABRICS[]   */

/* Surface/lighting parameters fed to the cloth shader. */
static float g_specular  = 0.5f;                   /* specular intensity     */
static float g_roughness = 0.45f;                  /* 0 sharp .. 1 broad     */

static float g_clothSize     = DEFAULT_CLOTH_SIZE; /* world units per side   */
static float g_colliderScale = 1.0f;               /* x0.3 .. x3.0           */

/* Collision shell actually used at runtime. It is CLOTH_THICKNESS at fine
 * resolutions but grows with particle spacing on coarse cloth — see
 * BuildCloth for the reasoning. */
static float g_collisionShell = CLOTH_THICKNESS;

/* (#ifndef so test builds can boot straight into e.g. COLLIDER_MESH.) */
#ifndef DEFAULT_COLLIDER
#define DEFAULT_COLLIDER COLLIDER_SPHERE
#endif
static ColliderKind g_collider = DEFAULT_COLLIDER;
static MeshCollider g_meshCollider = { 0 };

/* Collider placement, derived from the user-adjustable scale. All shapes
 * share the X/Z origin so switching between them feels seamless; the sphere
 * hovers a hair above the ground, the cube rests exactly on it. */
static float   SphereRadius(void) { return 1.00f * g_colliderScale; }
static Vector3 SphereCenter(void) { return (Vector3){ 0.0f, SphereRadius() + 0.15f, 0.0f }; }
static Vector3 CubeHalf(void)     { float h = 0.85f * g_colliderScale; return (Vector3){ h, h, h }; }
static Vector3 CubeCenter(void)   { return (Vector3){ 0.0f, CubeHalf().y, 0.0f }; }

/* Highest point of the current collider — used to place the cloth spawn and
 * to pull the camera back when the scene grows. */
static float ColliderTopY(void)
{
    switch (g_collider)
    {
        case COLLIDER_SPHERE: return SphereCenter().y + SphereRadius();
        case COLLIDER_CUBE:   return CubeCenter().y + CubeHalf().y;
        case COLLIDER_MESH:   return g_meshCollider.bounds.max.y;
        default:              return 0.0f;
    }
}

static float g_simTime = 0.0f;

/* GPU resources for the shaded cloth (built in InitClothMesh / BuildCloth). */
static Shader          g_shader        = { 0 };
static Mesh            g_clothMesh      = { 0 };
static Material        g_clothMat       = { 0 };
static bool            g_clothMeshReady = false;
static int   g_locLightDir, g_locViewPos, g_locSpec, g_locRough;

/* Export state */
static AppMode g_mode = MODE_SETTINGS;
static FILE   *g_ffmpegPipe   = NULL;
static int     g_renderFrame  = 0;
static bool    g_ffmpegOk     = false;
static char    g_ffmpegPath[260] = "ffmpeg";
static char    g_statusMsg[256]  = "";
static double  g_statusUntil     = 0.0;
static int     g_exportRes    = 1;               /* index into EXPORT_RES[]  */
static RenderTexture2D g_exportRT = { 0 };       /* offscreen render target  */

/* ============================================================================
 *  SMALL HELPERS
 * ========================================================================= */

static void SetStatus(const char *msg)
{
    strncpy(g_statusMsg, msg, sizeof(g_statusMsg) - 1);
    g_statusMsg[sizeof(g_statusMsg) - 1] = '\0';
    g_statusUntil = GetTime() + 4.0;
}

static inline int PIndex(int i, int j) { return j * g_gridN + i; }

/* ============================================================================
 *  CLOTH CONSTRUCTION
 * ========================================================================= */

static void RebuildClothMesh(void);   /* GPU mesh (re)alloc, defined below */

/* Add one spring; rest length is the current (flat, unstretched) separation. */
static void AddSpring(int a, int b, SpringType type)
{
    if (g_springCount >= MAX_SPRINGS) return;
    Spring *s = &g_springs[g_springCount++];
    s->a = a;
    s->b = b;
    s->restLen = Vector3Distance(g_particles[a].pos, g_particles[b].pos);
    s->type = type;
}



/* (Re)build the cloth: an N x N flat sheet above the collider with structural,
 * shear and bending springs.
 *   STRUCTURAL - grid neighbours; carry weight, resist stretch.
 *   SHEAR      - quad diagonals; resist in-plane collapse to a parallelogram.
 *   BENDING    - skip-one-particle springs; resist folding (softer than k). */
static void BuildCloth(int n)
{
    g_gridN = n;
    g_springCount = 0;

    const float spacing = g_clothSize / (float)(n - 1);
    const float half    = g_clothSize * 0.5f;

    /* Collision shell scales with spacing: collision is per-particle but the
     * rendered surface spans straight between particles, so on coarse cloth the
     * span dips into a curved collider. Push particles out ~a third of a cell;
     * capped so a 2x2 sheet doesn't hover comically high. */
    g_collisionShell = Clamp(0.35f * spacing, CLOTH_THICKNESS, 0.15f);

    const float spawnY = fmaxf(CLOTH_SPAWN_Y, ColliderTopY() + 3.0f);

    for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++)
    {
        Particle *p = &g_particles[PIndex(i, j)];
        p->pos = (Vector3){ -half + i * spacing, spawnY, -half + j * spacing };
        p->prevPos = p->pos;
        p->vel = (Vector3){ 0 };
        p->force = (Vector3){ 0 };
        p->invMass = 1.0f / PARTICLE_MASS;
        if (g_pinTopEdge && j == 0) p->invMass = 0.0f;   /* hanging-curtain */
    }

    for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++)
    {
        if (i < n - 1) AddSpring(PIndex(i, j), PIndex(i + 1, j), SPRING_STRUCTURAL);
        if (j < n - 1) AddSpring(PIndex(i, j), PIndex(i, j + 1), SPRING_STRUCTURAL);
        if (i < n - 1 && j < n - 1)
        {
            AddSpring(PIndex(i, j),     PIndex(i + 1, j + 1), SPRING_SHEAR);
            AddSpring(PIndex(i + 1, j), PIndex(i, j + 1),     SPRING_SHEAR);
        }
        if (i < n - 2) AddSpring(PIndex(i, j), PIndex(i + 2, j), SPRING_BENDING);
        if (j < n - 2) AddSpring(PIndex(i, j), PIndex(i, j + 2), SPRING_BENDING);
    }

    RebuildClothMesh();
    g_simTime = 0.0f;
}

/* ============================================================================
 *  COLLISION RESOLUTION
 *
 *  Contact is a penalty layer: the collision shell around each solid acts as
 *  a spring-damper (the classic penalty method — Terzopoulos, Platt, Barr &
 *  Fleischer, "Elastically Deformable Models", SIGGRAPH 1987). A particle
 *  entering the shell is decelerated over several substeps, the layer
 *  stores that energy elastically, and the damping ratio decides how much
 *  comes back — that IS the fabric's restitution. This matters for cloth:
 *  hard per-particle velocity reflection cannot make a sheet bounce, because
 *  neighbours strike a curved surface at different instants and each
 *  reflection incoherently rings its own lattice node against still-seated
 *  neighbours until the springs eat it (verified with a velocity trace).
 *  The penalty layer loads the whole contact patch together and launches it
 *  back as a coherent unit.
 *
 *  Each resolver:
 *    1. Detect: is the particle inside the shell zone around the solid?
 *    2. Penalty: apply the spring-damper impulse along the outward normal,
 *       plus tangential friction while in contact.
 *    3. Backstop: if the particle got deeper than a quarter-shell from the
 *       surface (fast hit, or swept through thin geometry), project it back
 *       out position-wise and absorb the remaining inward velocity.
 * ========================================================================= */

/* Backstop response: discard any remaining inward normal velocity, apply
 * tangential friction (sticking below STICK_SPEED so a hanging skirt doesn't
 * drag the sheet off). Only reached on deep/fast contact. */
static void ContactAbsorb(Particle *p, Vector3 normal)
{
    float vn = Vector3DotProduct(p->vel, normal);
    Vector3 vNormal  = Vector3Scale(normal, vn);
    Vector3 vTangent = Vector3Subtract(p->vel, vNormal);
    if (vn < 0.0f) vNormal = (Vector3){ 0 };          /* moving inward: stop */
    vTangent = Vector3Scale(vTangent, g_friction);
    if (Vector3LengthSqr(vTangent) < STICK_SPEED * STICK_SPEED)
        vTangent = (Vector3){ 0 };                    /* static friction     */
    p->vel = Vector3Add(vNormal, vTangent);
}

/* Penalty spring-damper, applied as a velocity impulse (identical to adding
 * the force next substep, since integration is semi-implicit anyway).
 *
 *   stiffness  kc = m·CONTACT_OMEGA² — arrests a CONTACT_VREF impact within
 *               half the shell (stopping distance v/omega). StepFrame counts
 *               CONTACT_OMEGA in its substep budget, so h·omega stays <= 0.7
 *               at every resolution.
 *   damping    cd = 2ζ·sqrt(kc·m), with the damping ratio ζ derived from
 *               the fabric's restitution e through the logarithmic
 *               decrement of a damped oscillator: ζ = -ln e / √(π²+ln²e).
 *               e→1 rebounds elastically, e→0 lands dead.
 *
 *  Resting contact is a plain static equilibrium of this spring (the cloth
 *  floats mg/kc — a millimetre or two — inside the shell), so unlike
 *  reflection-based restitution there is no jitter and no need for a
 *  velocity threshold. Friction runs whenever the layer is compressed. */
static void ContactPenalty(Particle *p, Vector3 normal, float depth, float h)
{
    if (depth <= 0.0f || p->invMass == 0.0f) return;

    const float kc = PARTICLE_MASS * CONTACT_OMEGA * CONTACT_OMEGA;
    float e    = (g_restitution < 0.01f) ? 0.01f : g_restitution;
    float lnE  = logf(e);
    float zeta = -lnE / sqrtf(PI * PI + lnE * lnE);
    const float cd = 2.0f * zeta * sqrtf(kc * PARTICLE_MASS);

    float vn = Vector3DotProduct(p->vel, normal);
    float fn = kc * depth - cd * vn;
    if (fn > 0.0f)                                    /* no adhesion         */
    {
        p->vel = Vector3Add(p->vel, Vector3Scale(normal, fn * p->invMass * h));

        /* friction while the layer carries load */
        vn = Vector3DotProduct(p->vel, normal);
        Vector3 vNormal  = Vector3Scale(normal, vn);
        Vector3 vTangent = Vector3Subtract(p->vel, vNormal);
        vTangent = Vector3Scale(vTangent, g_friction);
        if (Vector3LengthSqr(vTangent) < STICK_SPEED * STICK_SPEED)
            vTangent = (Vector3){ 0 };
        p->vel = Vector3Add(vNormal, vTangent);
    }
}

/* --- 1. Sphere ------------------------------------------------------------ */
static void CollideSphere(Particle *p, float h)
{
    Vector3 centre = SphereCenter();
    Vector3 d = Vector3Subtract(p->pos, centre);
    float dist  = Vector3Length(d);
    float outer = SphereRadius() + g_collisionShell;
    if (dist >= outer || dist < 1e-6f) return;

    Vector3 n = Vector3Scale(d, 1.0f / dist);         /* outward unit normal */
    float backstop = SphereRadius() + 0.25f * g_collisionShell;
    if (dist < backstop)
    {
        p->pos = Vector3Add(centre, Vector3Scale(n, backstop));
        ContactAbsorb(p, n);
        dist = backstop;
    }
    ContactPenalty(p, n, outer - dist, h);
}

/* --- 2. Cube: axis-aligned box. Near-surface contact runs the penalty
 * layer; fully-inside ejects through the nearest face to the backstop. */
static void CollideCube(Particle *p, float h)
{
    Vector3 centre = CubeCenter(), halfExt = CubeHalf();
    Vector3 lo = Vector3Subtract(centre, halfExt);
    Vector3 hi = Vector3Add(centre, halfExt);

    bool inside = (p->pos.x > lo.x && p->pos.x < hi.x &&
                   p->pos.y > lo.y && p->pos.y < hi.y &&
                   p->pos.z > lo.z && p->pos.z < hi.z);

    if (!inside)
    {
        Vector3 closest = {
            Clamp(p->pos.x, lo.x, hi.x),
            Clamp(p->pos.y, lo.y, hi.y),
            Clamp(p->pos.z, lo.z, hi.z)
        };
        Vector3 d = Vector3Subtract(p->pos, closest);
        float dist = Vector3Length(d);
        if (dist >= g_collisionShell || dist < 1e-6f) return;

        Vector3 n = Vector3Scale(d, 1.0f / dist);
        float backstop = 0.25f * g_collisionShell;
        if (dist < backstop)
        {
            p->pos = Vector3Add(closest, Vector3Scale(n, backstop));
            ContactAbsorb(p, n);
            dist = backstop;
        }
        ContactPenalty(p, n, g_collisionShell - dist, h);
    }
    else
    {
        /* Distance from the particle to each of the six faces; the smallest
         * one is the cheapest exit route. */
        float dx1 = p->pos.x - lo.x, dx2 = hi.x - p->pos.x;
        float dy1 = p->pos.y - lo.y, dy2 = hi.y - p->pos.y;
        float dz1 = p->pos.z - lo.z, dz2 = hi.z - p->pos.z;

        float best = dx1; Vector3 n = { -1, 0, 0 };
        if (dx2 < best) { best = dx2; n = (Vector3){ 1, 0, 0 }; }
        if (dy1 < best) { best = dy1; n = (Vector3){ 0,-1, 0 }; }
        if (dy2 < best) { best = dy2; n = (Vector3){ 0, 1, 0 }; }
        if (dz1 < best) { best = dz1; n = (Vector3){ 0, 0,-1 }; }
        if (dz2 < best) { best = dz2; n = (Vector3){ 0, 0, 1 }; }

        p->pos = Vector3Add(p->pos, Vector3Scale(n, best + 0.25f * g_collisionShell));
        ContactAbsorb(p, n);
        ContactPenalty(p, n, 0.75f * g_collisionShell, h);
    }
}

/* --- 3. Arbitrary mesh (.obj) -----------------------------------------------
 * Closest point on a triangle to a point, from Ericson's "Real-Time Collision
 * Detection" (the Voronoi-region walk). Returns the closest point; barycentric
 * logic is inlined for speed. */
static Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
{
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ac = Vector3Subtract(c, a);
    Vector3 ap = Vector3Subtract(p, a);

    float d1 = Vector3DotProduct(ab, ap);
    float d2 = Vector3DotProduct(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;                    /* vertex A   */

    Vector3 bp = Vector3Subtract(p, b);
    float d3 = Vector3DotProduct(ab, bp);
    float d4 = Vector3DotProduct(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;                      /* vertex B   */

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)                /* edge AB    */
        return Vector3Add(a, Vector3Scale(ab, d1 / (d1 - d3)));

    Vector3 cp = Vector3Subtract(p, c);
    float d5 = Vector3DotProduct(ab, cp);
    float d6 = Vector3DotProduct(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;                      /* vertex C   */

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)                /* edge AC    */
        return Vector3Add(a, Vector3Scale(ac, d2 / (d2 - d6)));

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)  /* edge BC    */
        return Vector3Add(b, Vector3Scale(Vector3Subtract(c, b),
                                          (d4 - d3) / ((d4 - d3) + (d5 - d6))));

    /* Interior of the face */
    float denom = 1.0f / (va + vb + vc);
    return Vector3Add(a, Vector3Add(Vector3Scale(ab, vb * denom),
                                    Vector3Scale(ac, vc * denom)));
}

/* Map a world coordinate onto a grid axis, clamped to valid cells. Out-of-
 * range positions land in the border cell, which is fine: the CSR lists in
 * those cells already contain everything near the mesh boundary. */
static inline int GridCoord(float v, float origin, float cell, int n)
{
    int c = (int)((v - origin) / cell);
    if (c < 0) c = 0;
    if (c > n - 1) c = n - 1;
    return c;
}

/* Geometric (winding-derived) unit normal of triangle t. */
static Vector3 TriNormal(int t)
{
    Vector3 *tri = &g_meshCollider.tri[t * 3];
    Vector3 cr = Vector3CrossProduct(Vector3Subtract(tri[1], tri[0]),
                                     Vector3Subtract(tri[2], tri[0]));
    return (Vector3LengthSqr(cr) > 1e-12f) ? Vector3Normalize(cr)
                                           : (Vector3){ 0, 1, 0 };
}

/* Möller–Trumbore, segment flavour: does o + t*d (t in [0,1]) cross the
 * triangle? d is the UN-normalised displacement, so t parameterises the
 * segment directly. The barycentric bounds get a hair of slack so a particle
 * can't slip through the mathematical seam between two adjacent triangles. */
static bool SegmentHitsTriangle(Vector3 o, Vector3 d,
                                Vector3 a, Vector3 b, Vector3 c, float *tOut)
{
    Vector3 e1 = Vector3Subtract(b, a);
    Vector3 e2 = Vector3Subtract(c, a);
    Vector3 pv = Vector3CrossProduct(d, e2);
    float det = Vector3DotProduct(e1, pv);
    if (fabsf(det) < 1e-12f) return false;            /* parallel            */

    float inv = 1.0f / det;
    Vector3 tv = Vector3Subtract(o, a);
    float u = Vector3DotProduct(tv, pv) * inv;
    if (u < -0.001f || u > 1.001f) return false;

    Vector3 qv = Vector3CrossProduct(tv, e1);
    float v = Vector3DotProduct(d, qv) * inv;
    if (v < -0.001f || u + v > 1.001f) return false;

    float t = Vector3DotProduct(e2, qv) * inv;
    if (t < 0.0f || t > 1.0f) return false;

    *tOut = t;
    return true;
}

/* Keep a particle out of the loaded mesh. One walk over the uniform grid does
 * two tests: a SWEEP of the prevPos->pos segment (reverts to the last legal
 * position on a crossing — required for thin geometry a particle can tunnel
 * through in one substep), and a PROXIMITY push-out along the nearest
 * triangle's normal for resting contact. Sidedness trusts the .obj winding. */
static void CollideMesh(Particle *p, float h)
{
    (void)h;
    Vector3 disp = Vector3Subtract(p->pos, p->prevPos);
    float travel = Vector3Length(disp);
    float reach  = g_collisionShell + travel;

    /* Cheap broad-phase: skip everything outside the inflated AABB. This
     * rejects the vast majority of the particles each substep. */
    BoundingBox bb = g_meshCollider.bounds;
    if (p->pos.x < bb.min.x - reach || p->pos.x > bb.max.x + reach ||
        p->pos.y < bb.min.y - reach || p->pos.y > bb.max.y + reach ||
        p->pos.z < bb.min.z - reach || p->pos.z > bb.max.z + reach) return;

    float bestDistSq = reach * reach;
    Vector3 bestPoint = { 0 };
    int bestTri = -1;

    float bestT = 2.0f;                               /* earliest sweep hit  */
    int sweepTri = -1;

    /* Narrow phase over the cells the swept box overlaps. A triangle that
     * spans several of them gets tested more than once — harmless, both
     * tests keep a min. */
    const MeshCollider *mc = &g_meshCollider;
    Vector3 lo = Vector3Min(p->pos, p->prevPos);
    Vector3 hi = Vector3Max(p->pos, p->prevPos);
    int cx0 = GridCoord(lo.x - g_collisionShell, mc->gridMin.x, mc->cellSize.x, mc->gridNx);
    int cx1 = GridCoord(hi.x + g_collisionShell, mc->gridMin.x, mc->cellSize.x, mc->gridNx);
    int cy0 = GridCoord(lo.y - g_collisionShell, mc->gridMin.y, mc->cellSize.y, mc->gridNy);
    int cy1 = GridCoord(hi.y + g_collisionShell, mc->gridMin.y, mc->cellSize.y, mc->gridNy);
    int cz0 = GridCoord(lo.z - g_collisionShell, mc->gridMin.z, mc->cellSize.z, mc->gridNz);
    int cz1 = GridCoord(hi.z + g_collisionShell, mc->gridMin.z, mc->cellSize.z, mc->gridNz);

    for (int cz = cz0; cz <= cz1; cz++)
    for (int cy = cy0; cy <= cy1; cy++)
    for (int cx = cx0; cx <= cx1; cx++)
    {
        int cell = (cz * mc->gridNy + cy) * mc->gridNx + cx;
        for (int k = mc->gridStart[cell]; k < mc->gridStart[cell + 1]; k++)
        {
            int t = mc->gridItems[k];
            Vector3 *tri = &mc->tri[t * 3];

            if (travel > 1e-8f)
            {
                float tc;
                if (SegmentHitsTriangle(p->prevPos, disp, tri[0], tri[1], tri[2], &tc)
                    && tc < bestT)
                {
                    bestT = tc;
                    sweepTri = t;
                }
            }

            Vector3 q = ClosestPointOnTriangle(p->pos, tri[0], tri[1], tri[2]);
            float dsq = Vector3DistanceSqr(p->pos, q);
            if (dsq < bestDistSq)
            {
                bestDistSq = dsq;
                bestPoint = q;
                bestTri = t;
            }
        }
    }

    if (sweepTri >= 0)
    {
        /* Crossed a surface: revert to the last legal position (prevPos can't
         * overshoot) and kill the inward velocity. A normal-offset placement
         * would overshoot the opposite face on thin features and pump energy.
         * No penalty here — the proximity pass owns it from next substep. */
        Vector3 faceN = TriNormal(sweepTri);
        Vector3 n = (Vector3DotProduct(disp, faceN) < 0.0f) ? faceN
                                                            : Vector3Negate(faceN);
        (void)bestT;
        p->pos = p->prevPos;
        ContactAbsorb(p, n);
        return;
    }

    if (bestTri < 0) return;

    Vector3 faceN = TriNormal(bestTri);
    Vector3 d = Vector3Subtract(p->pos, bestPoint);
    float dist = sqrtf(bestDistSq);
    float backstop = 0.25f * g_collisionShell;

    if (Vector3DotProduct(d, faceN) < 0.0f)
    {
        /* Behind the surface (e.g. spawned inside): eject out the front. */
        p->pos = Vector3Add(bestPoint, Vector3Scale(faceN, backstop));
        ContactAbsorb(p, faceN);
        ContactPenalty(p, faceN, g_collisionShell - backstop, h);
    }
    else if (dist < g_collisionShell)
    {
        Vector3 n = (dist > 1e-5f) ? Vector3Scale(d, 1.0f / dist) : faceN;
        if (dist < backstop)
        {
            p->pos = Vector3Add(bestPoint, Vector3Scale(n, backstop));
            ContactAbsorb(p, n);
            dist = backstop;
        }
        ContactPenalty(p, n, g_collisionShell - dist, h);
    }
}

/* Infinite ground plane at y = GROUND_Y, so the cloth's skirt has somewhere
 * to land (and bounce). */
static void CollideGround(Particle *p, float h)
{
    float y = p->pos.y - GROUND_Y;
    if (y >= g_collisionShell) return;

    Vector3 up = { 0, 1, 0 };
    float backstop = 0.25f * g_collisionShell;
    if (y < backstop)
    {
        p->pos.y = GROUND_Y + backstop;
        ContactAbsorb(p, up);
        y = backstop;
    }
    ContactPenalty(p, up, g_collisionShell - y, h);
}

/* ============================================================================
 *  MESH COLLIDER LOADING
 * ========================================================================= */

/* (Re)bake the already-loaded model into world-space triangles at the
 * current collider scale, then build the uniform grid over them. Called once
 * at startup and again whenever the user changes the collider size. The mesh
 * is normalised — centred, uniformly scaled to a ~2.3 * scale footprint, and
 * rested on the ground. */
static void BakeMeshCollider(void)
{
    Model *model = &g_meshCollider.model;
    if (model->meshCount == 0) return;             /* nothing loaded (yet)   */

    /* Work out the model's own bounding box so we can normalise its size. */
    BoundingBox raw = GetMeshBoundingBox(model->meshes[0]);
    Vector3 size   = Vector3Subtract(raw.max, raw.min);
    Vector3 centre = Vector3Scale(Vector3Add(raw.min, raw.max), 0.5f);
    float footprint = fmaxf(size.x, size.z);
    if (footprint < 1e-6f) footprint = 1.0f;
    float scale = 2.3f * g_colliderScale / footprint;

    /* Bake every triangle into world space. raylib keeps a CPU-side copy of
     * mesh.vertices after upload, which is exactly what we need. Indexed and
     * non-indexed meshes are both handled. */
    Mesh mesh = model->meshes[0];
    int triCount = (mesh.indices != NULL) ? mesh.triangleCount
                                          : mesh.vertexCount / 3;

    MemFree(g_meshCollider.tri);                   /* MemFree(NULL) is fine  */
    MemFree(g_meshCollider.gridStart);
    MemFree(g_meshCollider.gridItems);

    g_meshCollider.tri = (Vector3 *)MemAlloc(sizeof(Vector3) * 3 * triCount);
    g_meshCollider.triCount = triCount;

    for (int t = 0; t < triCount; t++)
    {
        for (int k = 0; k < 3; k++)
        {
            int vi = (mesh.indices != NULL) ? mesh.indices[t * 3 + k] : (t * 3 + k);
            Vector3 v = { mesh.vertices[vi * 3 + 0],
                          mesh.vertices[vi * 3 + 1],
                          mesh.vertices[vi * 3 + 2] };
            /* centre -> scale -> lift so the lowest point sits on the ground */
            v = Vector3Scale(Vector3Subtract(v, centre), scale);
            v.y += (size.y * 0.5f) * scale;
            g_meshCollider.tri[t * 3 + k] = v;
        }
    }

    /* World AABB. Inflated by a constant that covers the largest possible
     * collision shell (0.15) plus margin, so the broad-phase never rejects a
     * particle the narrow phase would have caught. */
    Vector3 mn = g_meshCollider.tri[0], mx = g_meshCollider.tri[0];
    for (int i = 1; i < triCount * 3; i++)
    {
        mn = Vector3Min(mn, g_meshCollider.tri[i]);
        mx = Vector3Max(mx, g_meshCollider.tri[i]);
    }
    g_meshCollider.bounds.min = Vector3SubtractValue(mn, 0.35f);
    g_meshCollider.bounds.max = Vector3AddValue(mx, 0.35f);

    /* --- Uniform grid build (two-pass CSR) ------------------------------ */
    {
        Vector3 ext = Vector3Subtract(mx, mn);
        float target = fmaxf(fmaxf(ext.x, ext.y), ext.z) / 16.0f;
        if (target < 1e-4f) target = 1.0f;

        int nx = (int)ceilf(ext.x / target); nx = (nx < 1) ? 1 : (nx > 32) ? 32 : nx;
        int ny = (int)ceilf(ext.y / target); ny = (ny < 1) ? 1 : (ny > 32) ? 32 : ny;
        int nz = (int)ceilf(ext.z / target); nz = (nz < 1) ? 1 : (nz > 32) ? 32 : nz;

        Vector3 cs = { (ext.x > 1e-6f) ? ext.x / nx : 1.0f,
                       (ext.y > 1e-6f) ? ext.y / ny : 1.0f,
                       (ext.z > 1e-6f) ? ext.z / nz : 1.0f };

        g_meshCollider.gridNx = nx;
        g_meshCollider.gridNy = ny;
        g_meshCollider.gridNz = nz;
        g_meshCollider.gridMin = mn;
        g_meshCollider.cellSize = cs;

        int cells = nx * ny * nz;
        int *start = (int *)MemAlloc(sizeof(int) * (cells + 1));
        int *count = (int *)MemAlloc(sizeof(int) * cells);

        /* Pass 1: how many triangles land in each cell (a triangle registers
         * in every cell its AABB overlaps). Pass 2: fill the item lists. */
        for (int pass = 0; pass < 2; pass++)
        {
            for (int t = 0; t < triCount; t++)
            {
                Vector3 *tri = &g_meshCollider.tri[t * 3];
                Vector3 tmn = Vector3Min(tri[0], Vector3Min(tri[1], tri[2]));
                Vector3 tmx = Vector3Max(tri[0], Vector3Max(tri[1], tri[2]));
                int x0 = GridCoord(tmn.x, mn.x, cs.x, nx), x1 = GridCoord(tmx.x, mn.x, cs.x, nx);
                int y0 = GridCoord(tmn.y, mn.y, cs.y, ny), y1 = GridCoord(tmx.y, mn.y, cs.y, ny);
                int z0 = GridCoord(tmn.z, mn.z, cs.z, nz), z1 = GridCoord(tmx.z, mn.z, cs.z, nz);

                for (int cz = z0; cz <= z1; cz++)
                for (int cy = y0; cy <= y1; cy++)
                for (int cx = x0; cx <= x1; cx++)
                {
                    int cell = (cz * ny + cy) * nx + cx;
                    if (pass == 0) count[cell]++;
                    else g_meshCollider.gridItems[start[cell] + count[cell]++] = t;
                }
            }
            if (pass == 0)
            {
                start[0] = 0;
                for (int c = 0; c < cells; c++) start[c + 1] = start[c] + count[c];
                g_meshCollider.gridItems = (int *)MemAlloc(sizeof(int) * start[cells]);
                memset(count, 0, sizeof(int) * cells);   /* reuse as cursor  */
            }
        }
        g_meshCollider.gridStart = start;
        MemFree(count);
    }

    /* Keep the model for drawing, transformed the same way we baked the
     * triangles so what you see is what the cloth collides with. */
    model->transform = MatrixMultiply(
        MatrixMultiply(MatrixTranslate(-centre.x, -centre.y, -centre.z),
                       MatrixScale(scale, scale, scale)),
        MatrixTranslate(0, (size.y * 0.5f) * scale, 0));

    TraceLog(LOG_INFO, "FACEFALL: mesh collider baked (%d triangles, x%.1f, grid %dx%dx%d)",
             triCount, g_colliderScale, g_meshCollider.gridNx,
             g_meshCollider.gridNy, g_meshCollider.gridNz);
}

/* Load assets/model.obj, or fall back to a procedurally generated torus if
 * the file is missing (so the demo never breaks), then bake it at the
 * current scale. */
static void LoadMeshCollider(void)
{
    Model model;
    bool fromFile = false;

    if (FileExists("assets/model.obj"))
    {
        model = LoadModel("assets/model.obj");
        if (model.meshCount > 0 && model.meshes[0].vertexCount > 0) fromFile = true;
        else TraceLog(LOG_WARNING, "FACEFALL: assets/model.obj loaded but empty, using torus");
    }
    if (!fromFile)
    {
        model = LoadModelFromMesh(GenMeshTorus(0.45f, 2.0f, 24, 32));
        TraceLog(LOG_WARNING, "FACEFALL: assets/model.obj not found, generated torus instead");
    }

    g_meshCollider.model = model;
    g_meshCollider.fromFile = fromFile;
    BakeMeshCollider();
}

/* ============================================================================
 *  PHYSICS STEP
 * ========================================================================= */

static void ApplyStrainLimiting(float h);


/* Push all particles out of the active collider and the ground. */
static void ResolveColliders(float h)
{
    const int n2 = g_gridN * g_gridN;
    for (int i = 0; i < n2; i++)
    {
        Particle *p = &g_particles[i];
        if (p->invMass == 0.0f) continue;
        switch (g_collider)
        {
            case COLLIDER_SPHERE: CollideSphere(p, h); break;
            case COLLIDER_CUBE:   CollideCube(p, h);   break;
            case COLLIDER_MESH:   CollideMesh(p, h);   break;
            default: break;
        }
        CollideGround(p, h);
    }
}

/* ============================================================================
 *  SELF-COLLISION
 *
 *  Particle-particle separation on a uniform spatial hash (Teschner et al.,
 *  "Optimized Spatial Hashing for Collision Detection of Deformable
 *  Objects", VMV 2003 — same magic primes). Any two particles that are not
 *  near-neighbours in the grid are kept at least a cloth-thickness apart:
 *  the overlap is corrected in position space split by inverse mass, and the
 *  closing velocity along the pair axis is removed, so when the sheet folds
 *  or bounces its layers stack on each other instead of raining through.
 *  This is the expensive part of the sim (every pair candidate, every
 *  substep) and can be toggled in the settings menu.
 * ========================================================================= */

#define SELF_HASH_SIZE 32768                        /* power of two          */
static int g_selfHead[SELF_HASH_SIZE];
static int g_selfNext[MAX_PARTICLES];

static inline unsigned SelfHash(int x, int y, int z)
{
    return (((unsigned)x * 73856093u) ^ ((unsigned)y * 19349663u)
          ^ ((unsigned)z * 83492791u)) & (SELF_HASH_SIZE - 1);
}

static void ResolveSelfCollision(void)
{
    const int   n       = g_gridN, n2 = n * n;
    const float spacing = g_clothSize / (float)(n - 1);
    const float minDist = 0.75f * spacing;          /* cloth-on-cloth shell  */
    const float cell    = minDist;                  /* 27 cells cover a pair */

    memset(g_selfHead, -1, sizeof(g_selfHead));
    for (int i = 0; i < n2; i++)
    {
        Vector3 p = g_particles[i].pos;
        unsigned hsh = SelfHash((int)floorf(p.x / cell),
                                (int)floorf(p.y / cell),
                                (int)floorf(p.z / cell));
        g_selfNext[i] = g_selfHead[hsh];
        g_selfHead[hsh] = i;
    }

    for (int a = 0; a < n2; a++)
    {
        Particle *pa = &g_particles[a];
        int ax = (int)floorf(pa->pos.x / cell);
        int ay = (int)floorf(pa->pos.y / cell);
        int az = (int)floorf(pa->pos.z / cell);
        int ai = a % n, aj = a / n;

        for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
        for (int b = g_selfHead[SelfHash(ax + dx, ay + dy, az + dz)];
             b >= 0; b = g_selfNext[b])
        {
            if (b <= a) continue;                   /* each pair once        */

            /* Immediate grid neighbours are held together by springs at less
             * than minDist by construction; separating them would fight the
             * spring network, so they are exempt. */
            int bi = b % n, bj = b / n;
            if (abs(ai - bi) <= 1 && abs(aj - bj) <= 1) continue;

            Particle *pb = &g_particles[b];
            Vector3 d = Vector3Subtract(pb->pos, pa->pos);
            float distSq = Vector3LengthSqr(d);
            if (distSq >= minDist * minDist || distSq < 1e-12f) continue;

            float wa = pa->invMass, wb = pb->invMass, wSum = wa + wb;
            if (wSum <= 0.0f) continue;

            float dist = sqrtf(distSq);
            Vector3 nrm = Vector3Scale(d, 1.0f / dist);
            float push = (minDist - dist) / wSum;
            pa->pos = Vector3Subtract(pa->pos, Vector3Scale(nrm, push * wa));
            pb->pos = Vector3Add(pb->pos, Vector3Scale(nrm, push * wb));

            /* Kill the closing velocity so the layers rest instead of
             * re-penetrating next substep (inelastic layer contact). */
            float relV = Vector3DotProduct(Vector3Subtract(pb->vel, pa->vel), nrm);
            if (relV < 0.0f)
            {
                float imp = relV / wSum;
                pa->vel = Vector3Add(pa->vel, Vector3Scale(nrm, imp * wa));
                pb->vel = Vector3Subtract(pb->vel, Vector3Scale(nrm, imp * wb));
            }
        }
    }
}

/* Wind sampled as a velocity field: a gusting magnitude with a spatial
 * ripple phase so the sheet luffs instead of translating rigidly. Feeds the
 * per-triangle aerodynamics as the free-stream air velocity. */
static Vector3 WindVelocity(Vector3 at)
{
    if (g_windPower == 0.0f) return (Vector3){ 0 };
    float gust = g_windPower * (0.65f + 0.35f * sinf(g_simTime * 1.7f)
                                      + 0.20f * sinf(g_simTime * 4.3f));
    float ripple = sinf(at.z * 2.1f + g_simTime * 3.0f) * 0.3f + 1.0f;
    return (Vector3){ gust * ripple, 0.0f, gust * -0.35f * ripple };
}

/* One substep, semi-implicit (symplectic) Euler: accumulate F(x,v), advance v
 * with F, then advance x with the NEW v. Using the updated velocity is what
 * keeps oscillating springs from pumping energy every step. */
static void PhysicsSubstep(float h)
{
    const int n2 = g_gridN * g_gridN;

    /* External forces: gravity plus a small residual linear drag. The real
     * air resistance is per-triangle, below. */
    for (int i = 0; i < n2; i++)
    {
        Particle *p = &g_particles[i];
        p->force = (Vector3){ 0, -g_gravity * PARTICLE_MASS, 0 };
        p->force = Vector3Add(p->force, Vector3Scale(p->vel, -AIR_DRAG * PARTICLE_MASS));
    }

    /* Per-triangle aerodynamics: flat-plate pressure drag, the standard
     * quadratic drag law F = ½ρ·Cd·A·|v|² projected onto the face normal and
     * split evenly over the triangle's three particles. A sheet falling
     * face-on parachutes hard, falling edge-on it slices through, and wind
     * only pushes on surfaces that actually face it — this is what makes the
     * fall flutter and billow instead of looking underwater. The combined
     * coefficient is normalised by total cloth mass (PARTICLE_MASS·N²) so
     * the SAME AERO_DRAG behaves identically at every grid resolution. */
    {
        const int n = g_gridN;
        const float cAero = AERO_DRAG * PARTICLE_MASS * (float)n2
                          / (g_clothSize * g_clothSize);
        for (int j = 0; j < n - 1; j++)
        for (int i = 0; i < n - 1; i++)
        {
            /* two triangles per grid quad, same split as the render mesh */
            const int q[4] = { PIndex(i, j),     PIndex(i + 1, j),
                               PIndex(i + 1, j + 1), PIndex(i, j + 1) };
            for (int t = 0; t < 2; t++)
            {
                Particle *p0 = &g_particles[q[0]];
                Particle *p1 = &g_particles[q[t + 1]];
                Particle *p2 = &g_particles[q[t + 2]];

                Vector3 nA = Vector3Scale(Vector3CrossProduct(
                                 Vector3Subtract(p1->pos, p0->pos),
                                 Vector3Subtract(p2->pos, p0->pos)), 0.5f);
                float area = Vector3Length(nA);     /* |nA| = triangle area  */
                if (area < 1e-9f) continue;

                Vector3 centroid = Vector3Scale(
                    Vector3Add(Vector3Add(p0->pos, p1->pos), p2->pos), 1.0f / 3.0f);
                Vector3 vRel = Vector3Scale(
                    Vector3Add(Vector3Add(p0->vel, p1->vel), p2->vel), 1.0f / 3.0f);
                vRel = Vector3Subtract(vRel, WindVelocity(centroid));
                float speed = Vector3Length(vRel);
                if (speed < 1e-6f) continue;

                /* sign of (v·n̂) flips with the winding, so the force always
                 * opposes motion through the face — winding-independent */
                Vector3 nHat = Vector3Scale(nA, 1.0f / area);
                float vn = Vector3DotProduct(vRel, nHat);
                Vector3 f = Vector3Scale(nHat, -cAero * area * speed * vn / 3.0f);

                p0->force = Vector3Add(p0->force, f);
                p1->force = Vector3Add(p1->force, f);
                p2->force = Vector3Add(p2->force, f);
            }
        }
    }

    /* Internal springs: Hooke's law with damping projected onto the spring
     * axis only (damping the full relative velocity would fight rigid rotation
     * and look like the fabric is swimming through honey). */
    for (int s = 0; s < g_springCount; s++)
    {
        Spring *sp = &g_springs[s];
        Particle *pa = &g_particles[sp->a];
        Particle *pb = &g_particles[sp->b];

        Vector3 d = Vector3Subtract(pb->pos, pa->pos);
        float len = Vector3Length(d);
        if (len < 1e-7f) continue;
        Vector3 dhat = Vector3Scale(d, 1.0f / len);

        float stretch = len - sp->restLen;
        float relVel  = Vector3DotProduct(Vector3Subtract(pb->vel, pa->vel), dhat);
        float k = (sp->type == SPRING_BENDING) ? g_stiffness * g_bend : g_stiffness;
        float mag = k * stretch + g_damping * relVel;   /* pulls a,b together */
        Vector3 f = Vector3Scale(dhat, mag);

        pa->force = Vector3Add(pa->force, f);
        pb->force = Vector3Subtract(pb->force, f);
    }

    /* Integrate. */
    for (int i = 0; i < n2; i++)
    {
        Particle *p = &g_particles[i];
        if (p->invMass == 0.0f) continue;
        p->prevPos = p->pos;      /* last legal position, for the mesh sweep */
        p->vel = Vector3Add(p->vel, Vector3Scale(p->force, p->invMass * h));
        p->pos = Vector3Add(p->pos, Vector3Scale(p->vel, h));
    }

    /* Positional constraints run before collision so the frame never ends with
     * cloth inside the collider and friction sees every source of motion.
     * Self-collision goes between them: strain limiting may re-bunch layers,
     * and the solid collider must have the last word on interpenetration. */
    ApplyStrainLimiting(h);
    if (g_selfCollide) ResolveSelfCollision();
    ResolveColliders(h);
}

/* Strain limiting (Provot 1995): pull the endpoints of any structural spring
 * stretched past MAX_STRETCH back together in position space. Lets us run
 * believable fabric at a stiffness the integrator can afford; a few
 * Gauss-Seidel passes converge plenty for visuals.
 *
 * Every positional correction also applies the matching velocity impulse
 * (dv = dx/h — the same bookkeeping PBD does when it rebuilds velocity from
 * corrected positions). Without it, position and velocity drift apart: a
 * particle recoiling off the collider gets its position clamped back to its
 * seated neighbours while its velocity keeps insisting it is moving up at
 * 3 m/s (watched it happen in a trace), contact damping then reads that
 * phantom velocity, and the recoil momentum simply evaporates instead of
 * travelling down the sheet as a wave. */
static void ApplyStrainLimiting(float h)
{
    const float invH = 1.0f / h;
    for (int it = 0; it < STRAIN_ITERATIONS; it++)
    {
        for (int s = 0; s < g_springCount; s++)
        {
            Spring *sp = &g_springs[s];
            if (sp->type != SPRING_STRUCTURAL) continue;

            Particle *pa = &g_particles[sp->a];
            Particle *pb = &g_particles[sp->b];

            Vector3 d = Vector3Subtract(pb->pos, pa->pos);
            float len = Vector3Length(d);
            float maxLen = sp->restLen * MAX_STRETCH;
            if (len <= maxLen || len < 1e-7f) continue;

            /* Split the correction according to inverse mass, so pinned
             * particles (invMass = 0) never move. */
            float wa = pa->invMass, wb = pb->invMass;
            float wSum = wa + wb;
            if (wSum <= 0.0f) continue;

            Vector3 corr = Vector3Scale(d, (len - maxLen) / (len * wSum));
            pa->pos = Vector3Add(pa->pos, Vector3Scale(corr, wa));
            pb->pos = Vector3Subtract(pb->pos, Vector3Scale(corr, wb));
            pa->vel = Vector3Add(pa->vel, Vector3Scale(corr, wa * invH));
            pb->vel = Vector3Subtract(pb->vel, Vector3Scale(corr, wb * invH));
        }
    }
}



/* Advance the sim one video frame using enough substeps to keep the
 * symplectic-Euler stability bound h*omega <= 0.7 for the stiffest thing in
 * the system — the springs or the contact penalty layer, whichever rings
 * faster (raising stiffness buys more substeps instead of exploding). */
static void StepFrame(void)
{
    float omega = sqrtf(g_stiffness / PARTICLE_MASS);
    if (CONTACT_OMEGA > omega) omega = CONTACT_OMEGA;
    int substeps = (int)ceilf(FRAME_DT * omega / 0.7f);
    if (substeps < 4)   substeps = 4;
    if (substeps > 120) substeps = 120;

    float h = FRAME_DT / (float)substeps;
    for (int s = 0; s < substeps; s++)
    {
        PhysicsSubstep(h);
        g_simTime += h;
    }
}

/* ============================================================================
 *  FFMPEG EXPORT
 *
 *  Each frame we render the scene to an offscreen RenderTexture at the chosen
 *  export resolution, read back its raw RGBA, and fwrite() it into ffmpeg's
 *  stdin. ffmpeg is told the stream geometry up front (-f rawvideo -pix_fmt
 *  rgba -s WxH), so it just consumes W*H*4 bytes per frame and encodes H.264.
 *  The preview window stays at 1280x720 regardless of export size.
 * ========================================================================= */

/* Probe for a usable ffmpeg binary once at startup ("ffmpeg -version" exits 0
 * iff it runs; the redirect keeps its banner off our console). */
static void DetectFFmpeg(void)
{
    for (int i = 0; i < (int)(sizeof(FFMPEG_CANDIDATES)/sizeof(FFMPEG_CANDIDATES[0])); i++)
    {
        char probe[512];
        snprintf(probe, sizeof(probe), "\"%s\" -version > %s 2>&1",
                 FFMPEG_CANDIDATES[i], DEVNULL);
        if (system(probe) == 0)
        {
            snprintf(g_ffmpegPath, sizeof(g_ffmpegPath), "%s", FFMPEG_CANDIDATES[i]);
            g_ffmpegOk = true;
            TraceLog(LOG_INFO, "FACEFALL: ffmpeg found: %s", g_ffmpegPath);
            return;
        }
    }
    g_ffmpegOk = false;
    TraceLog(LOG_WARNING,
             "FACEFALL: ffmpeg not found — video export disabled. "
             "Install it from https://ffmpeg.org or drop ffmpeg.exe into tools/ffmpeg/bin/");
}

static void BeginRender(void)
{
    if (!g_ffmpegOk)
    {
        SetStatus("ffmpeg not found - export unavailable (see console)");
        return;
    }

    int ew = EXPORT_RES[g_exportRes].w, eh = EXPORT_RES[g_exportRes].h;

    /* Offscreen target at export resolution, independent of the window.
     * The whole command gets an EXTRA outer pair of quotes: _popen hands it to
     * `cmd /c`, which strips the first and last quote when it sees more than
     * two — the outer pair exists purely to be sacrificed to that rule. */
    g_exportRT = LoadRenderTexture(ew, eh);
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
             "\"\"%s\" -y -loglevel error -f rawvideo -pix_fmt rgba -s %dx%d "
             "-framerate %d -i - -c:v libx264 -preset medium -crf 18 "
             "-pix_fmt yuv420p \"%s\"\"",
             g_ffmpegPath, ew, eh, RENDER_FPS, OUTPUT_FILENAME);

    g_ffmpegPipe = POPEN(cmd, POPEN_WRITE_MODE);
    if (g_ffmpegPipe == NULL)
    {
        UnloadRenderTexture(g_exportRT);
        g_exportRT = (RenderTexture2D){ 0 };
        SetStatus("popen() failed - could not start ffmpeg");
        return;
    }

    /* Deterministic export: restart the sim from frame zero. */
    BuildCloth(g_gridN);
    g_renderFrame = 0;
    g_mode = MODE_RENDER;
    g_paused = false;
    SetStatus(TextFormat("Rendering %s... (V or ESC cancels)", EXPORT_RES[g_exportRes].name));
}

static void EndRender(bool completed)
{
    if (g_ffmpegPipe != NULL)
    {
        /* pclose flushes the pipe, waits for ffmpeg to finish, and reaps it.
         * Skipping this truncates the moov atom and the file won't play. */
        PCLOSE(g_ffmpegPipe);
        g_ffmpegPipe = NULL;
    }
    if (g_exportRT.id != 0)
    {
        UnloadRenderTexture(g_exportRT);
        g_exportRT = (RenderTexture2D){ 0 };
    }
    g_mode = MODE_SETTINGS;
    SetStatus(completed ? "Saved " OUTPUT_FILENAME
                        : "Export cancelled (partial file written)");
}

/* Read the export render target back and ship it down the pipe. Called while
 * g_exportRT is still bound, so rlReadScreenPixels reads the RT (not the
 * window) and flips it to top-left origin, ready to stream as-is. */
static bool PipeFrame(int w, int h)
{
    unsigned char *pixels = rlReadScreenPixels(w, h);
    size_t frameBytes = (size_t)w * h * 4;
    size_t written = fwrite(pixels, 1, frameBytes, g_ffmpegPipe);
    RL_FREE(pixels);

    if (written != frameBytes)
    {
        TraceLog(LOG_ERROR, "FACEFALL: ffmpeg pipe broke after %d frames", g_renderFrame);
        return false;
    }
    return true;
}

/* ============================================================================
 *  RENDERING
 * ========================================================================= */

/* GLSL 330 lighting shader shared by cloth and collider. Diffuse + Blinn-Phong
 * specular, a rim term for silhouette pop, and (cloth only) a procedural weave
 * normal perturbation plus a subsurface backlight so thin fabric glows when lit
 * from behind. The surface is two-sided: the normal is flipped toward the
 * viewer in the fragment stage so folds are never black. */
static const char *CLOTH_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform mat4 matNormal;\n"
    "out vec3 fragPos; out vec2 fragTexCoord; out vec3 fragNormal; out vec4 fragColor;\n"
    "void main(){\n"
    "  fragPos = vec3(matModel*vec4(vertexPosition,1.0));\n"
    "  fragTexCoord = vertexTexCoord;\n"
    "  fragNormal = normalize(vec3(matNormal*vec4(vertexNormal,0.0)));\n"
    "  fragColor = vertexColor;\n"
    "  gl_Position = mvp*vec4(vertexPosition,1.0);\n"
    "}\n";

static const char *CLOTH_FS =
    "#version 330\n"
    "in vec3 fragPos; in vec2 fragTexCoord; in vec3 fragNormal; in vec4 fragColor;\n"
    "uniform vec4 colDiffuse; uniform vec3 lightDir; uniform vec3 viewPos;\n"
    "uniform float uSpecular; uniform float uRoughness; uniform float uCloth;\n"
    "out vec4 finalColor;\n"
    "void main(){\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  vec3 V = normalize(viewPos - fragPos);\n"
    "  if (dot(N,V) < 0.0) N = -N;                 // two-sided\n"
    "  vec3 L = normalize(-lightDir);\n"
    "  if (uCloth > 0.5) {                          // procedural weave bump\n"
    "    vec3 T = normalize(cross(N, vec3(0.0,1.0,0.0)) + vec3(1e-4));\n"
    "    vec3 B = cross(N, T);\n"
    "    N = normalize(N + T*cos(fragTexCoord.x*140.0)*0.04\n"
    "                    + B*cos(fragTexCoord.y*140.0)*0.04);\n"
    "  }\n"
    "  float NdotL = max(dot(N,L), 0.0);\n"
    "  float wrap = max((NdotL + 0.3)/1.3, 0.0);    // soft wrap for fabric\n"
    "  vec3 base = (colDiffuse*fragColor).rgb;\n"
    "  vec3 col = base*0.25 + base*wrap*0.9;\n"
    "  vec3 H = normalize(L+V);\n"
    "  float shin = mix(120.0, 8.0, clamp(uRoughness,0.0,1.0));\n"
    "  col += vec3(pow(max(dot(N,H),0.0), shin) * uSpecular * NdotL);\n"
    "  float rim = pow(1.0 - max(dot(N,V),0.0), 3.0);\n"
    "  col += vec3(0.5,0.6,0.8)*rim*0.6;\n"
    "  if (uCloth > 0.5) col += base*pow(max(dot(-N,L),0.0),2.0)*0.4;   // backlight\n"
    "  finalColor = vec4(col, 1.0);\n"
    "}\n";

static Mesh     g_sphereMesh, g_cubeMesh;
static Material g_colliderMat;
static int      g_locCloth;

static void InitShaders(void)
{
    g_shader = LoadShaderFromMemory(CLOTH_VS, CLOTH_FS);
    g_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(g_shader, "viewPos");
    g_locLightDir = GetShaderLocation(g_shader, "lightDir");
    g_locViewPos  = g_shader.locs[SHADER_LOC_VECTOR_VIEW];
    g_locSpec     = GetShaderLocation(g_shader, "uSpecular");
    g_locRough    = GetShaderLocation(g_shader, "uRoughness");
    g_locCloth    = GetShaderLocation(g_shader, "uCloth");

    g_clothMat = LoadMaterialDefault();
    g_clothMat.shader = g_shader;
    g_clothMat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 226, 56, 82, 255 };

    g_colliderMat = LoadMaterialDefault();
    g_colliderMat.shader = g_shader;
    g_colliderMat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 90, 140, 220, 255 };

    g_sphereMesh = GenMeshSphere(1.0f, 24, 32);
    g_cubeMesh   = GenMeshCube(1.0f, 1.0f, 1.0f);
}

/* Write current particle positions and smoothed vertex normals into the cloth
 * mesh's CPU arrays (normals = normalised sum of surrounding face normals). */
static void FillClothVertices(void)
{
    const int n = g_gridN;
    float *V = g_clothMesh.vertices, *No = g_clothMesh.normals;

    for (int v = 0; v < n * n; v++)
    {
        Vector3 p = g_particles[v].pos;
        V[v*3+0] = p.x; V[v*3+1] = p.y; V[v*3+2] = p.z;
        No[v*3+0] = No[v*3+1] = No[v*3+2] = 0.0f;
    }
    for (int j = 0; j < n - 1; j++)
    for (int i = 0; i < n - 1; i++)
    {
        int i00 = PIndex(i, j), i10 = PIndex(i+1, j), i01 = PIndex(i, j+1), i11 = PIndex(i+1, j+1);
        Vector3 fn = Vector3CrossProduct(
            Vector3Subtract(g_particles[i10].pos, g_particles[i00].pos),
            Vector3Subtract(g_particles[i01].pos, g_particles[i00].pos));
        int c[4] = { i00, i10, i01, i11 };
        for (int k = 0; k < 4; k++)
        { No[c[k]*3+0]+=fn.x; No[c[k]*3+1]+=fn.y; No[c[k]*3+2]+=fn.z; }
    }
    for (int v = 0; v < n * n; v++)
    {
        Vector3 nn = { No[v*3+0], No[v*3+1], No[v*3+2] };
        float lsq = Vector3LengthSqr(nn);
        nn = (lsq > 1e-12f) ? Vector3Scale(nn, 1.0f/sqrtf(lsq)) : (Vector3){ 0, 1, 0 };
        No[v*3+0] = nn.x; No[v*3+1] = nn.y; No[v*3+2] = nn.z;
    }
}

/* (Re)allocate the cloth GPU mesh when the resolution changes. Static topology
 * (indices, uvs) is set once; positions/normals stream every frame. */
static void RebuildClothMesh(void)
{
    if (g_shader.id == 0) return;               /* shaders not up yet */
    const int n = g_gridN;
    const int verts = n * n, tris = 2 * (n-1) * (n-1);
    if (g_clothMeshReady && g_clothMesh.vertexCount == verts) { FillClothVertices(); return; }

    if (g_clothMeshReady) UnloadMesh(g_clothMesh);

    Mesh m = { 0 };
    m.vertexCount = verts;
    m.triangleCount = tris;
    m.vertices  = (float *)MemAlloc(sizeof(float) * 3 * verts);
    m.normals   = (float *)MemAlloc(sizeof(float) * 3 * verts);
    m.texcoords = (float *)MemAlloc(sizeof(float) * 2 * verts);
    m.indices   = (unsigned short *)MemAlloc(sizeof(unsigned short) * 3 * tris);

    for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++)
    {
        int v = j * n + i;
        m.texcoords[v*2+0] = (float)i / (n - 1);
        m.texcoords[v*2+1] = (float)j / (n - 1);
    }
    int t = 0;
    for (int j = 0; j < n - 1; j++)
    for (int i = 0; i < n - 1; i++)
    {
        unsigned short i00 = j*n+i, i10 = j*n+i+1, i01 = (j+1)*n+i, i11 = (j+1)*n+i+1;
        m.indices[t++] = i00; m.indices[t++] = i10; m.indices[t++] = i11;
        m.indices[t++] = i00; m.indices[t++] = i11; m.indices[t++] = i01;
    }

    g_clothMesh = m;
    FillClothVertices();
    UploadMesh(&g_clothMesh, true);             /* dynamic: streamed each frame */
    g_clothMeshReady = true;
}

static void DrawCloth(Camera3D camera)
{
    FillClothVertices();
    UpdateMeshBuffer(g_clothMesh, 0, g_clothMesh.vertices, sizeof(float)*3*g_clothMesh.vertexCount, 0);
    UpdateMeshBuffer(g_clothMesh, 2, g_clothMesh.normals,  sizeof(float)*3*g_clothMesh.vertexCount, 0);

    Vector3 lightDir = Vector3Normalize((Vector3){ 0.5f, -1.0f, 0.35f });
    float cloth = 1.0f;
    SetShaderValue(g_shader, g_locLightDir, &lightDir,        SHADER_UNIFORM_VEC3);
    SetShaderValue(g_shader, g_locViewPos,  &camera.position, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_shader, g_locSpec,     &g_specular,      SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_shader, g_locRough,    &g_roughness,     SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_shader, g_locCloth,    &cloth,           SHADER_UNIFORM_FLOAT);

    rlDisableBackfaceCulling();                 /* cloth is two-sided */
    DrawMesh(g_clothMesh, g_clothMat, MatrixIdentity());
    rlEnableBackfaceCulling();

    if (g_wireframe)
        for (int s = 0; s < g_springCount; s++)
        {
            if (g_springs[s].type != SPRING_STRUCTURAL) continue;
            DrawLine3D(g_particles[g_springs[s].a].pos,
                       g_particles[g_springs[s].b].pos,
                       (Color){ 255, 255, 255, 90 });
        }
}

static void DrawCollider(void)
{
    float notCloth = 0.0f, spec = 0.35f, rough = 0.55f;
    SetShaderValue(g_shader, g_locCloth, &notCloth, SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_shader, g_locSpec,  &spec,     SHADER_UNIFORM_FLOAT);
    SetShaderValue(g_shader, g_locRough, &rough,    SHADER_UNIFORM_FLOAT);

    switch (g_collider)
    {
        case COLLIDER_SPHERE:
        {
            float r = SphereRadius(); Vector3 c = SphereCenter();
            DrawMesh(g_sphereMesh, g_colliderMat,
                     MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(c.x, c.y, c.z)));
            break;
        }
        case COLLIDER_CUBE:
        {
            Vector3 h = CubeHalf(), c = CubeCenter();
            DrawMesh(g_cubeMesh, g_colliderMat,
                     MatrixMultiply(MatrixScale(h.x*2, h.y*2, h.z*2),
                                    MatrixTranslate(c.x, c.y, c.z)));
            break;
        }
        case COLLIDER_MESH:
            g_meshCollider.model.materials[0].shader = g_shader;
            DrawModel(g_meshCollider.model, Vector3Zero(), 1.0f, (Color){ 90, 140, 220, 255 });
            break;
        default: break;
    }
}

/* Draw the whole 3D scene (grid, collider, cloth). Called once per frame,
 * either straight to the window or into the export render target. */
static void DrawScene(Camera3D camera, float camScale)
{
    DrawGrid(20, 0.5f * camScale);
    DrawCollider();
    DrawCloth(camera);
}

/* Apply a fabric preset to the live physics parameters. */
static void ApplyFabric(int idx)
{
    g_fabric      = idx;
    g_stiffness   = FABRICS[idx].stiffness;
    g_bend        = FABRICS[idx].bend;
    g_damping     = FABRICS[idx].damping;
    g_friction    = FABRICS[idx].friction;
    g_restitution = FABRICS[idx].restitution;
}

static void DrawHUD(bool freeCam)
{
    const char *colliderNames[] = { "SPHERE", "CUBE",
        g_meshCollider.fromFile ? "MESH (assets/model.obj)" : "MESH (generated torus)" };

    DrawRectangle(8, 8, 380, g_showHelp ? 172 : 78, Fade(BLACK, 0.55f));
    DrawText("FaceFall - cloth simulator", 16, 14, 20, RAYWHITE);
    DrawText(TextFormat("cloth %dx%d (%d faces, %.1f units)   collider: %s x%.1f",
                        g_gridN, g_gridN, (g_gridN-1)*(g_gridN-1), g_clothSize,
                        colliderNames[g_collider], g_colliderScale),
             16, 40, 10, RAYWHITE);
    DrawText(TextFormat("%s  k=%.0f  bend=%.2f  rest=%.2f  wind=%.1f  %s%s%s",
                        FABRICS[g_fabric].name, g_stiffness, g_bend, g_restitution,
                        g_windPower, g_selfCollide ? "" : "NO-SELFCOL ",
                        g_paused ? "PAUSED " : "", g_pinTopEdge ? "PINNED" : ""),
             16, 56, 10, RAYWHITE);

    if (g_showHelp)
    {
        int y = 76;
        const char *lines[] = {
            "R       reset cloth",
            "SPACE   pause      W  wireframe",
            "F       free camera (WASD+mouse)",
            "V       export video (ffmpeg)",
            "ESC     back to settings",
            "H       hide help",
        };
        for (int i = 0; i < (int)(sizeof(lines)/sizeof(lines[0])); i++, y += 15)
            DrawText(lines[i], 16, y, 10, (Color){ 200, 200, 200, 255 });
    }

    if (!g_ffmpegOk)
        DrawText("ffmpeg NOT FOUND - export disabled", 16, SCREEN_H - 24, 10,
                 (Color){ 255, 120, 100, 255 });

    if (freeCam)
        DrawText("FREE CAM: WASD + mouse (F to release)", SCREEN_W - 260, 10, 10, YELLOW);

    if (g_mode == MODE_RENDER)
    {
        float t = (float)g_renderFrame / RENDER_TOTAL_FRAMES;
        DrawRectangle(0, SCREEN_H - 8, (int)(SCREEN_W * t), 8, RED);
        DrawText(TextFormat("RENDERING %d / %d", g_renderFrame, RENDER_TOTAL_FRAMES),
                 SCREEN_W - 200, SCREEN_H - 30, 20, RED);
    }

    DrawFPS(SCREEN_W - 90, SCREEN_H - 60);
}

/* The settings screen shown at startup (and after every export). The scene
 * sits frozen behind it so you can see what you are configuring. */
#define SETTINGS_ROWS 17

static void DrawSettings(int row)
{
    const char *colliderNames[] = { "sphere", "cube",
        g_meshCollider.fromFile ? "mesh (assets/model.obj)" : "mesh (torus)" };
    const char *labels[SETTINGS_ROWS] = {
        "cloth resolution", "cloth size", "collider", "collider size",
        "fabric preset", "stiffness", "bend", "damping", "friction",
        "restitution", "self collision",
        "gravity", "wind", "specular", "roughness",
        "export res", "pin top edge"
    };

    const int w = 500, h = 494;
    const int px = SCREEN_W / 2 - w / 2, py = SCREEN_H / 2 - h / 2;

    DrawRectangle(px, py, w, h, Fade(BLACK, 0.82f));
    DrawRectangleLines(px, py, w, h, Fade(RAYWHITE, 0.35f));
    DrawText("FACEFALL - made for Stardance", px + 20, py + 14, 20, RAYWHITE);

    int y = py + 46;
    for (int i = 0; i < SETTINGS_ROWS; i++, y += 23)
    {
        const char *val =
            (i == 0)  ? TextFormat("%d x %d", g_gridN, g_gridN) :
            (i == 1)  ? TextFormat("%.1f units", g_clothSize) :
            (i == 2)  ? colliderNames[g_collider] :
            (i == 3)  ? TextFormat("x%.1f", g_colliderScale) :
            (i == 4)  ? FABRICS[g_fabric].name :
            (i == 5)  ? TextFormat("%.0f", g_stiffness) :
            (i == 6)  ? TextFormat("%.2f", g_bend) :
            (i == 7)  ? TextFormat("%.1f", g_damping) :
            (i == 8)  ? TextFormat("%.2f", g_friction) :
            (i == 9)  ? TextFormat("%.2f", g_restitution) :
            (i == 10) ? (g_selfCollide ? "on" : "off") :
            (i == 11) ? TextFormat("%.1f", g_gravity) :
            (i == 12) ? TextFormat("%.1f", g_windPower) :
            (i == 13) ? TextFormat("%.2f", g_specular) :
            (i == 14) ? TextFormat("%.2f", g_roughness) :
            (i == 15) ? EXPORT_RES[g_exportRes].name :
                        (g_pinTopEdge ? "yes" : "no");
        Color c = (i == row) ? YELLOW : (Color){ 205, 205, 205, 255 };
        if (i == row) DrawText(">", px + 20, y, 16, c);
        DrawText(labels[i], px + 40, y, 16, c);
        DrawText(val, px + 300, y, 16, c);
    }

    DrawText("UP/DOWN select    LEFT/RIGHT change", px + 20, y + 6, 10,
             (Color){ 170, 170, 170, 255 });
    DrawText("ENTER render + save mp4    P live preview    ESC quit",
             px + 20, y + 20, 10, (Color){ 170, 170, 170, 255 });
    if (!g_ffmpegOk)
        DrawText("ffmpeg NOT FOUND - render disabled", px + 20, y + 34, 10,
                 (Color){ 255, 120, 100, 255 });
}

/* ============================================================================
 *  MAIN
 * ========================================================================= */

static int RunSelfTest(void);

int main(int argc, char **argv)
{
    bool autoRender = (argc > 1 && strcmp(argv[1], "--render") == 0);
    if (argc > 1 && strcmp(argv[1], "--selftest") == 0) return RunSelfTest();

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_W, SCREEN_H, "FaceFall - cloth simulator");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);    /* ESC navigates modes; only the X quits        */

    DetectFFmpeg();
    InitShaders();           /* GL context up: build shader, materials, meshes */
    ApplyFabric(g_fabric);
    LoadMeshCollider();      /* needs the GL context, so must follow InitWindow */
    BuildCloth(GRID_SIZE);

    /* Fixed camera framing the whole drop; F toggles a free fly camera in
     * preview. Nothing pans on its own. */
    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 8.5f, 6.0f, 8.5f };
    camera.target     = (Vector3){ 0.0f, 2.0f, 0.0f };
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy       = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    bool freeCam = false;

    int resStep = 5;         /* index into RESOLUTION_STEPS -> 32            */
    for (int i = 0; i < NUM_RESOLUTION_STEPS; i++)
        if (RESOLUTION_STEPS[i] == GRID_SIZE) resStep = i;

    int  menuRow = 0;
    bool quit = false;

    if (autoRender) BeginRender();

    while (!WindowShouldClose() && !quit)
    {
        /* ------------------------------ input ---------------------------- */
        if (g_mode == MODE_SETTINGS)
        {
            if (IsKeyPressed(KEY_DOWN)) menuRow = (menuRow + 1) % SETTINGS_ROWS;
            if (IsKeyPressed(KEY_UP))   menuRow = (menuRow + SETTINGS_ROWS - 1) % SETTINGS_ROWS;

            int dir = (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT)) ? 1
                    : (IsKeyPressed(KEY_LEFT)  || IsKeyPressedRepeat(KEY_LEFT))  ? -1 : 0;
            if (dir != 0) switch (menuRow)
            {
                case 0:
                    resStep += dir;
                    if (resStep < 0) resStep = 0;
                    if (resStep > NUM_RESOLUTION_STEPS - 1) resStep = NUM_RESOLUTION_STEPS - 1;
                    BuildCloth(RESOLUTION_STEPS[resStep]);
                    break;
                case 1:
                    g_clothSize = Clamp(g_clothSize + dir * 0.5f, MIN_CLOTH_SIZE, MAX_CLOTH_SIZE);
                    BuildCloth(g_gridN);
                    break;
                case 2:
                    g_collider = (ColliderKind)(((int)g_collider + dir + COLLIDER_COUNT) % COLLIDER_COUNT);
                    BuildCloth(g_gridN);   /* spawn height tracks the collider */
                    break;
                case 3:
                    g_colliderScale = Clamp(g_colliderScale + dir * 0.1f, 0.3f, 3.0f);
                    BakeMeshCollider();
                    BuildCloth(g_gridN);
                    break;
                case 4:  ApplyFabric((g_fabric + dir + NUM_FABRICS) % NUM_FABRICS);        break;
                case 5:  g_stiffness = Clamp(g_stiffness + dir * 500.0f, 300.0f, 20000.0f); break;
                case 6:  g_bend      = Clamp(g_bend      + dir * 0.05f,  0.0f,    1.0f);     break;
                case 7:  g_damping   = Clamp(g_damping   + dir * 0.2f,   0.0f,   10.0f);     break;
                case 8:  g_friction  = Clamp(g_friction  + dir * 0.05f,  0.0f,    1.0f);     break;
                case 9:  g_restitution = Clamp(g_restitution + dir * 0.05f, 0.0f, 1.0f);     break;
                case 10: g_selfCollide = !g_selfCollide;                                     break;
                case 11: g_gravity   = Clamp(g_gravity   + dir * 0.5f,   0.0f,   30.0f);     break;
                case 12: g_windPower = Clamp(g_windPower + dir * 0.5f,   0.0f,   12.0f);     break;
                case 13: g_specular  = Clamp(g_specular  + dir * 0.05f,  0.0f,    1.0f);     break;
                case 14: g_roughness = Clamp(g_roughness + dir * 0.05f,  0.0f,    1.0f);     break;
                case 15: g_exportRes = (int)Clamp((float)(g_exportRes + dir), 0.0f, (float)(NUM_EXPORT_RES-1)); break;
                case 16: g_pinTopEdge = !g_pinTopEdge; BuildCloth(g_gridN);                  break;
            }

            if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) BeginRender();
            if (IsKeyPressed(KEY_P)) { BuildCloth(g_gridN); g_paused = false; g_mode = MODE_PREVIEW; }
            if (IsKeyPressed(KEY_ESCAPE)) quit = true;
        }
        else if (g_mode == MODE_PREVIEW)
        {
            if (IsKeyPressed(KEY_R)) BuildCloth(g_gridN);
            if (IsKeyPressed(KEY_W) && freeCam == false) g_wireframe = !g_wireframe;
            if (IsKeyPressed(KEY_SPACE)) g_paused = !g_paused;
            if (IsKeyPressed(KEY_H)) g_showHelp = !g_showHelp;
            if (IsKeyPressed(KEY_F))
            {
                freeCam = !freeCam;
                if (freeCam) DisableCursor(); else EnableCursor();
            }
            if (IsKeyPressed(KEY_V)) BeginRender();
            if (IsKeyPressed(KEY_ESCAPE))
            {
                if (freeCam) { freeCam = false; EnableCursor(); }
                g_mode = MODE_SETTINGS;
            }

            if (freeCam) UpdateCamera(&camera, CAMERA_FREE);
        }
        else /* MODE_RENDER: only allow cancelling */
        {
            if (IsKeyPressed(KEY_V) || IsKeyPressed(KEY_ESCAPE)) EndRender(false);
        }

        /* ------------------------------ simulate ------------------------- */
        /* The sim's fixed step is 1/RENDER_FPS. The exporter advances one
         * step per video frame; the 60 Hz preview window takes
         * RENDER_FPS/PREVIEW_FPS steps per displayed frame to stay real-time. */
        if (g_mode != MODE_SETTINGS && !g_paused)
        {
            int steps = (g_mode == MODE_PREVIEW) ? RENDER_FPS / PREVIEW_FPS : 1;
            for (int s = 0; s < steps; s++) StepFrame();
        }

        /* Auto-framing: pull the fixed camera back as the cloth or collider
         * grows so nothing pokes out of frame. At the default sizes this is
         * exactly the original framing (camScale == 1). Free-cam owns the
         * camera while it is on. */
        float camScale = fmaxf(1.0f, fmaxf(g_clothSize / 4.0f, ColliderTopY() / 2.2f));
        if (!freeCam)
        {
            camera.position = (Vector3){ 8.5f * camScale, 6.0f * camScale, 8.5f * camScale };
            camera.target   = (Vector3){ 0.0f, 2.0f * camScale, 0.0f };
        }

        /* ------------------------------ draw ------------------------------ */
        const Color bg = { 24, 26, 34, 255 };
        bool pipeOk = true;
        BeginDrawing();
            ClearBackground(bg);

            if (g_mode == MODE_RENDER)
            {
                /* Render the clean scene into the offscreen target at export
                 * resolution, read it back before the HUD, then blit a scaled
                 * copy into the window so the user sees progress. */
                int ew = EXPORT_RES[g_exportRes].w, eh = EXPORT_RES[g_exportRes].h;
                BeginTextureMode(g_exportRT);
                    ClearBackground(bg);
                    BeginMode3D(camera);
                        DrawScene(camera, camScale);
                    EndMode3D();
                    rlDrawRenderBatchActive();       /* flush geometry to the FBO */
                    pipeOk = PipeFrame(ew, eh);      /* read RT while still bound */
                EndTextureMode();

                DrawTexturePro(g_exportRT.texture,
                    (Rectangle){ 0, 0, (float)ew, -(float)eh },   /* flip Y */
                    (Rectangle){ 0, 0, SCREEN_W, SCREEN_H },
                    (Vector2){ 0, 0 }, 0.0f, WHITE);
            }
            else
            {
                BeginMode3D(camera);
                    DrawScene(camera, camScale);
                EndMode3D();
            }

            if (g_mode == MODE_SETTINGS) DrawSettings(menuRow);
            else                         DrawHUD(freeCam);

            if (GetTime() < g_statusUntil)
                DrawText(g_statusMsg, SCREEN_W / 2 - MeasureText(g_statusMsg, 20) / 2,
                         SCREEN_H - 48, 20, YELLOW);
        EndDrawing();

        /* ------------------------------ export --------------------------- */
        if (g_mode == MODE_RENDER)
        {
            if (!pipeOk)
            {
                EndRender(false);
                if (autoRender) break;    /* --render: don't idle on failure */
            }
            else if (++g_renderFrame >= RENDER_TOTAL_FRAMES)
            {
                EndRender(true);
                if (autoRender) break;    /* --render: exit when done        */
            }
        }
    }

    if (g_ffmpegPipe != NULL) EndRender(false);   /* window closed mid-export */

    if (g_clothMeshReady) UnloadMesh(g_clothMesh);
    UnloadMesh(g_sphereMesh);
    UnloadMesh(g_cubeMesh);
    UnloadShader(g_shader);
    UnloadModel(g_meshCollider.model);
    MemFree(g_meshCollider.tri);
    MemFree(g_meshCollider.gridStart);
    MemFree(g_meshCollider.gridItems);
    CloseWindow();
    return 0;
}

/* ============================================================================
 *  SELF-TEST  (facefall --selftest) — pure-CPU physics checks, no GL.
 * ========================================================================= */

/* Physics checks, two phases. Phase 1 drops the cloth on the SPHERE: the
 * hanging skirt anchors a draped sheet almost immediately (inextensible
 * fabric cannot trampoline off a ball — measured ceiling ~3cm even at e=1),
 * so there we assert the contact-layer RECOIL: the centre particle must come
 * back off the surface with real upward velocity. Then the drape must settle
 * finite and bounded with structural springs inside the strain limit.
 * Phase 2 drops the cloth on the CUBE, where the whole flat top strikes
 * coherently while the skirt is still airborne: there the sheet must rebound
 * VISIBLY in position, not just velocity. Pure CPU: RebuildClothMesh no-ops
 * without a GL context. */
static int RunSelfTest(void)
{
    g_pinTopEdge = false;
    g_windPower  = 0.0f;
    g_collider   = COLLIDER_SPHERE;
    BuildCloth(24);

    const int   centre   = PIndex(g_gridN / 2, g_gridN / 2);
    const float contactY = SphereCenter().y + SphereRadius()
                         + g_collisionShell + 0.02f;
    bool  touched = false;
    float vyRecoil = -1e9f;

    for (int frame = 0; frame < 3 * RENDER_FPS; frame++)
    {
        StepFrame();
        float y = g_particles[centre].pos.y;
#ifdef TRACE_CENTRE
        printf("f=%d y=%.4f vy=%.3f\n", frame, y, g_particles[centre].vel.y);
#endif
        if (!touched) touched = (y <= contactY);
        else if (g_particles[centre].vel.y > vyRecoil)
            vyRecoil = g_particles[centre].vel.y;
    }
    printf("[selftest] sphere: contact recoil vy peak = %+.2f m/s\n", vyRecoil);
    assert(touched && "cloth never reached the sphere");
    assert(vyRecoil > 0.8f && "no contact recoil - restitution broken");

    int n2 = g_gridN * g_gridN;
    float maxY = -1e9f, minY = 1e9f;
    for (int i = 0; i < n2; i++)
    {
        Vector3 pos = g_particles[i].pos;
        assert(isfinite(pos.x) && isfinite(pos.y) && isfinite(pos.z) && "position non-finite");
        assert(fabsf(pos.x) < 100.0f && fabsf(pos.y) < 100.0f && fabsf(pos.z) < 100.0f && "cloth flung");
        if (pos.y > maxY) maxY = pos.y;
        if (pos.y < minY) minY = pos.y;
    }
    printf("[selftest] drape settled: y in [%.3f, %.3f]\n", minY, maxY);
    assert(minY >= -0.05f && "cloth fell through the ground");

    float worst = 0.0f;
    for (int s = 0; s < g_springCount; s++)
    {
        Spring *sp = &g_springs[s];
        if (sp->type != SPRING_STRUCTURAL) continue;
        float ratio = Vector3Distance(g_particles[sp->a].pos, g_particles[sp->b].pos) / sp->restLen;
        if (ratio > worst) worst = ratio;
    }
    printf("[selftest] worst structural stretch: %.3f (limit %.3f)\n", worst, MAX_STRETCH);
    assert(worst < MAX_STRETCH + 0.05f && "strain limiting failed");

    /* Phase 2: flat coherent impact on the cube top must bounce for real. */
    g_collider = COLLIDER_CUBE;
    BuildCloth(24);
    const float cubeContactY = CubeCenter().y + CubeHalf().y
                             + g_collisionShell + 0.02f;
    touched = false;
    float touchY = 0.0f, reboundPeak = -1e9f;

    for (int frame = 0; frame < 2 * RENDER_FPS; frame++)
    {
        StepFrame();
        float y = g_particles[centre].pos.y;
        if (!touched) { if (y <= cubeContactY) { touched = true; touchY = y; } }
        else if (y > reboundPeak) reboundPeak = y;
    }
    printf("[selftest] cube: impact y=%.3f, rebound peak y=%.3f (+%.3f)\n",
           touchY, reboundPeak, reboundPeak - touchY);
    assert(touched && "cloth never reached the cube");
    assert(reboundPeak > touchY + 0.05f && "flat impact did not bounce");

    printf("[selftest] all checks passed\n");
    return 0;
}



