#ifndef FACEFALL_H
#define FACEFALL_H

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#ifdef _WIN32
    #define popen_wrap(cmd, mode)  _popen(cmd, mode)
    #define pclose_wrap(fp)        _pclose(fp)
    #define POPEN_WRITE_MODE       "wb"
    #define DEVNULL                "NUL"
#else
    #define popen_wrap(cmd, mode)  popen(cmd, mode)
    #define pclose_wrap(fp)        pclose(fp)
    #define POPEN_WRITE_MODE       "w"
    #define DEVNULL                "/dev/null"
#endif

#define SCREEN_W           1280
#define SCREEN_H           720

#ifndef GRID_SIZE
#define GRID_SIZE          32
#endif
#define MAX_GRID           128
#define MAX_PARTICLES      (MAX_GRID * MAX_GRID)
#define MAX_SPRINGS        (6 * MAX_GRID * MAX_GRID)

#define DEFAULT_CLOTH_SIZE 4.0f
#define MIN_CLOTH_SIZE     1.0f
#define MAX_CLOTH_SIZE     10.0f
#define CLOTH_SPAWN_Y      6.5f
#define CLOTH_THICKNESS    0.035f
#define PARTICLE_MASS      0.05f

#define DEFAULT_STIFFNESS  4500.0f
#define DEFAULT_BEND       0.35f
#define DEFAULT_DAMPING    1.2f
#define DEFAULT_FRICTION   0.20f
#define DEFAULT_GRAVITY    9.81f

#ifndef DEFAULT_RESTITUTION
#define DEFAULT_RESTITUTION 0.05f
#endif

#define CONTACT_VREF       7.0f
#define AERO_DRAG          0.18f
#define AIR_DRAG           0.06f

#ifndef MAX_STRETCH
#define MAX_STRETCH        1.08f
#endif
#define STRAIN_ITERATIONS  3

#define FRAME_DT           (1.0f / (float)RENDER_FPS)
#define GROUND_Y           0.0f
#define STICK_SPEED        0.30f

#define TEAR_SOFT_MULT     1.5f
#define BALL_RADIUS        0.30f
#define BALL_MASS          2.0f
#define BALL_SPEED         14.0f
#define GRAB_MAX_SPEED     25.0f

#define RENDER_SECONDS     12
#define RENDER_FPS         120
#define PREVIEW_FPS        60
#define RENDER_TOTAL_FRAMES (RENDER_SECONDS * RENDER_FPS)
#define OUTPUT_FILENAME    "facefall_output.mp4"

#define SELF_HASH_SIZE     32768

typedef enum {
    SPRING_STRUCTURAL,
    SPRING_SHEAR,
    SPRING_BENDING
} SpringType;

typedef struct {
    Vector3 pos;
    Vector3 prev_pos;
    Vector3 vel;
    Vector3 force;
    float inv_mass;
} Particle;

typedef struct {
    int a, b;
    float rest_len;
    SpringType type;
    bool alive;
} Spring;

typedef struct {
    const char *name;
    float stiffness;
    float bend;
    float damping;
    float friction;
    float restitution;
} Fabric;

typedef enum {
    COLLIDER_SPHERE = 0,
    COLLIDER_CUBE,
    COLLIDER_MESH,
    COLLIDER_COUNT
} ColliderKind;

typedef enum {
    MODE_SETTINGS,
    MODE_PREVIEW,
    MODE_RENDER
} AppMode;

typedef struct {
    Vector3 *tri;
    int tri_count;
    BoundingBox bounds;
    Model model;
    bool from_file;
    int *grid_start;
    int *grid_items;
    int grid_nx, grid_ny, grid_nz;
    Vector3 grid_min;
    Vector3 cell_size;
} MeshCollider;

typedef struct {
    const char *name;
    int w, h;
} ExportRes;

typedef struct {
    Particle particles[MAX_PARTICLES];
    Spring springs[MAX_SPRINGS];
    int grid_n;
    int spring_count;
    int spring_right[MAX_PARTICLES];
    int spring_down[MAX_PARTICLES];
    int shear_main[MAX_PARTICLES];
    int shear_anti[MAX_PARTICLES];
    int bend_right[MAX_PARTICLES];
    int bend_down[MAX_PARTICLES];

    float stiffness;
    float bend;
    float damping;
    float friction;
    float restitution;
    float gravity;
    float wind_power;
    float tear_ratio;
    bool self_collide;
    bool pin_top_edge;
    bool paused;
    float cloth_size;
    float collider_scale;
    float collision_shell;
    ColliderKind collider;
    MeshCollider mesh_collider;
    float sim_time;
    float time_scale;
    float step_accum;
    bool step_once;
    int grab_idx;
    float grab_dist;
    Vector3 ball_pos;
    Vector3 ball_vel;
    bool ball_active;

    float specular;
    float roughness;
    int fabric;
    bool wireframe;
    bool show_help;
    bool heatmap;
    bool topo_dirty;

    Shader shader;
    Mesh cloth_mesh;
    Material cloth_mat;
    Material collider_mat;
    Mesh sphere_mesh;
    Mesh cube_mesh;
    bool cloth_mesh_ready;
    int loc_light_dir, loc_view_pos, loc_spec, loc_rough, loc_cloth;

    AppMode mode;
    FILE *ffmpeg_pipe;
    int render_frame;
    bool ffmpeg_ok;
    char ffmpeg_path[260];
    char status_msg[256];
    double status_until;
    int export_res;
    RenderTexture2D export_rt;
} SimState;

extern const Fabric FABRICS[];
extern const int NUM_FABRICS;
extern const ExportRes EXPORT_RES[];
extern const int NUM_EXPORT_RES;
extern const int RESOLUTION_STEPS[];
extern const int NUM_RESOLUTION_STEPS;
extern const char *FFMPEG_CANDIDATES[];
extern const int NUM_FFMPEG_CANDIDATES;

static inline int particle_idx(int i, int j, int n) { return j * n + i; }
static inline float sphere_radius(const SimState *s) { return 1.00f * s->collider_scale; }
static inline Vector3 sphere_center(const SimState *s) { return (Vector3){ 0.0f, sphere_radius(s) + 0.15f, 0.0f }; }
static inline Vector3 cube_half(const SimState *s) { float h = 0.85f * s->collider_scale; return (Vector3){ h, h, h }; }
static inline Vector3 cube_center(const SimState *s) { return (Vector3){ 0.0f, cube_half(s).y, 0.0f }; }

static inline bool quad_alive(const SimState *s, int i, int j)
{
    int n = s->grid_n;
    int r0 = s->spring_right[particle_idx(i, j, n)];
    int r1 = s->spring_right[particle_idx(i, j + 1, n)];
    int d0 = s->spring_down[particle_idx(i, j, n)];
    int d1 = s->spring_down[particle_idx(i + 1, j, n)];
    return r0 >= 0 && r1 >= 0 && d0 >= 0 && d1 >= 0 &&
           s->springs[r0].alive && s->springs[r1].alive &&
           s->springs[d0].alive && s->springs[d1].alive;
}

float collider_top_y(const SimState *s);
void set_status(SimState *s, const char *msg);

void physics_init(SimState *s);
void build_cloth(SimState *s, int n);
void step_frame(SimState *s);
void apply_fabric(SimState *s, int idx);
void release_grab(SimState *s);

void init_mesh_collider(SimState *s);
void bake_mesh_collider(SimState *s);
void resolve_colliders(SimState *s, float h);
void mesh_push_ball(SimState *s, Vector3 *pos, Vector3 *vel, float radius);

bool renderer_init(SimState *s);
void renderer_cleanup(SimState *s);
void draw_scene(SimState *s, Camera3D camera, float cam_scale);
void rebuild_cloth_mesh(SimState *s);

void detect_ffmpeg(SimState *s);
void begin_render(SimState *s);
void end_render(SimState *s, bool completed);
bool pipe_frame(SimState *s, int w, int h);

void draw_hud(SimState *s, bool free_cam);
void draw_settings(SimState *s, int row);
void handle_input_settings(SimState *s, int *row, int *res_step, bool *quit);
void handle_input_preview(SimState *s, bool *free_cam);
void handle_input_render(SimState *s);
void handle_mouse_preview(SimState *s, Camera3D camera, bool free_cam, float cam_scale);

int run_selftest(void);

#endif
