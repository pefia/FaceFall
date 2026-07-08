#include "facefall.h"

const Fabric FABRICS[] = {
    { "silk",    4000.0f, 0.15f, 1.0f, 0.15f, 0.02f },
    { "cotton",  4500.0f, 0.35f, 1.2f, 0.25f, 0.05f },
    { "denim",   7000.0f, 0.60f, 1.8f, 0.40f, 0.08f },
    { "leather", 10000.0f, 0.85f, 2.5f, 0.55f, 0.12f },
};
const int NUM_FABRICS = sizeof(FABRICS) / sizeof(FABRICS[0]);

const ExportRes EXPORT_RES[] = {
    { "720p",  1280, 720 },
    { "1080p", 1920, 1080 },
    { "1440p", 2560, 1440 },
    { "4K",    3840, 2160 },
};
const int NUM_EXPORT_RES = sizeof(EXPORT_RES) / sizeof(EXPORT_RES[0]);

const int RESOLUTION_STEPS[] = { 2, 4, 8, 16, 24, 32, 48, 64, 96, 128 };
const int NUM_RESOLUTION_STEPS = sizeof(RESOLUTION_STEPS) / sizeof(RESOLUTION_STEPS[0]);

const char *FFMPEG_CANDIDATES[] = {
    "ffmpeg",
    "tools\\ffmpeg\\bin\\ffmpeg.exe",
    "tools/ffmpeg/bin/ffmpeg",
};
const int NUM_FFMPEG_CANDIDATES = sizeof(FFMPEG_CANDIDATES) / sizeof(FFMPEG_CANDIDATES[0]);

float collider_top_y(const SimState *s)
{
    switch (s->collider) {
    case COLLIDER_SPHERE: return sphere_center(s).y + sphere_radius(s);
    case COLLIDER_CUBE:   return cube_center(s).y + cube_half(s).y;
    case COLLIDER_MESH:   return s->mesh_collider.bounds.max.y;
    default:              return 0.0f;
    }
}

void set_status(SimState *s, const char *msg)
{
    strncpy(s->status_msg, msg, sizeof(s->status_msg) - 1);
    s->status_msg[sizeof(s->status_msg) - 1] = '\0';
    s->status_until = GetTime() + 4.0;
}

void apply_fabric(SimState *s, int idx)
{
    s->fabric = idx;
    s->stiffness   = FABRICS[idx].stiffness;
    s->bend        = FABRICS[idx].bend;
    s->damping     = FABRICS[idx].damping;
    s->friction    = FABRICS[idx].friction;
    s->restitution = FABRICS[idx].restitution;
}

static int add_spring(SimState *s, int a, int b, SpringType type)
{
    if (s->spring_count >= MAX_SPRINGS) return -1;
    Spring *sp = &s->springs[s->spring_count];
    sp->a = a;
    sp->b = b;
    sp->rest_len = Vector3Distance(s->particles[a].pos, s->particles[b].pos);
    sp->type = type;
    sp->alive = true;
    return s->spring_count++;
}

void build_cloth(SimState *s, int n)
{
    s->grid_n = n;
    s->spring_count = 0;
    memset(s->spring_right, -1, sizeof(s->spring_right));
    memset(s->spring_down,  -1, sizeof(s->spring_down));
    memset(s->shear_main,   -1, sizeof(s->shear_main));
    memset(s->shear_anti,   -1, sizeof(s->shear_anti));
    memset(s->bend_right,   -1, sizeof(s->bend_right));
    memset(s->bend_down,    -1, sizeof(s->bend_down));
    s->grab_idx = -1;
    s->ball_active = false;
    s->topo_dirty = true;

    float spacing = s->cloth_size / (float)(n - 1);
    float half = s->cloth_size * 0.5f;
    s->collision_shell = Clamp(0.35f * spacing, CLOTH_THICKNESS, 0.15f);
    float spawn_y = fmaxf(CLOTH_SPAWN_Y, collider_top_y(s) + 3.0f);

    for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++) {
        Particle *p = &s->particles[particle_idx(i, j, n)];
        p->pos = (Vector3){ -half + i * spacing, spawn_y, -half + j * spacing };
        p->prev_pos = p->pos;
        p->vel = (Vector3){ 0 };
        p->force = (Vector3){ 0 };
        p->inv_mass = 1.0f / PARTICLE_MASS;
        if (s->pin_top_edge && j == 0) p->inv_mass = 0.0f;
    }

    for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++) {
        int p = particle_idx(i, j, n);
        if (i < n - 1) s->spring_right[p] = add_spring(s, p, particle_idx(i + 1, j, n), SPRING_STRUCTURAL);
        if (j < n - 1) s->spring_down[p]  = add_spring(s, p, particle_idx(i, j + 1, n), SPRING_STRUCTURAL);
        if (i < n - 1 && j < n - 1) {
            s->shear_main[p] = add_spring(s, p, particle_idx(i + 1, j + 1, n), SPRING_SHEAR);
            s->shear_anti[p] = add_spring(s, particle_idx(i + 1, j, n), particle_idx(i, j + 1, n), SPRING_SHEAR);
        }
        if (i < n - 2) s->bend_right[p] = add_spring(s, p, particle_idx(i + 2, j, n), SPRING_BENDING);
        if (j < n - 2) s->bend_down[p]  = add_spring(s, p, particle_idx(i, j + 2, n), SPRING_BENDING);
    }

    rebuild_cloth_mesh(s);
    s->sim_time = 0.0f;
}

void physics_init(SimState *s)
{
    memset(s, 0, sizeof(*s));
    s->stiffness = DEFAULT_STIFFNESS;
    s->bend = DEFAULT_BEND;
    s->damping = DEFAULT_DAMPING;
    s->friction = DEFAULT_FRICTION;
    s->restitution = DEFAULT_RESTITUTION;
    s->gravity = DEFAULT_GRAVITY;
    s->cloth_size = DEFAULT_CLOTH_SIZE;
    s->specular = 0.5f;
    s->roughness = 0.45f;
    s->collider_scale = 1.0f;
    s->self_collide = true;
    s->show_help = true;
    s->fabric = 1;
    s->export_res = 1;
    s->collision_shell = CLOTH_THICKNESS;
    s->mode = MODE_SETTINGS;
    s->time_scale = 1.0f;
    s->grab_idx = -1;
    apply_fabric(s, s->fabric);
}

void release_grab(SimState *s)
{
    if (s->grab_idx < 0) return;
    Particle *p = &s->particles[s->grab_idx];
    p->inv_mass = (s->pin_top_edge && s->grab_idx < s->grid_n) ? 0.0f : 1.0f / PARTICLE_MASS;
    s->grab_idx = -1;
}

static void apply_strain_limiting(SimState *s, float h)
{
    float inv_h = 1.0f / h;
    for (int it = 0; it < STRAIN_ITERATIONS; it++) {
        for (int i = 0; i < s->spring_count; i++) {
            Spring *sp = &s->springs[i];
            if (sp->type != SPRING_STRUCTURAL || !sp->alive) continue;
            Particle *pa = &s->particles[sp->a];
            Particle *pb = &s->particles[sp->b];
            Vector3 d = Vector3Subtract(pb->pos, pa->pos);
            float len = Vector3Length(d);
            float max_len = sp->rest_len * MAX_STRETCH;
            if (len <= max_len || len < 1e-7f) continue;
            float wa = pa->inv_mass, wb = pb->inv_mass, wsum = wa + wb;
            if (wsum <= 0.0f) continue;
            Vector3 corr = Vector3Scale(d, (len - max_len) / (len * wsum));
            pa->pos = Vector3Add(pa->pos, Vector3Scale(corr, wa));
            pb->pos = Vector3Subtract(pb->pos, Vector3Scale(corr, wb));
            pa->vel = Vector3Add(pa->vel, Vector3Scale(corr, wa * inv_h));
            pb->vel = Vector3Subtract(pb->vel, Vector3Scale(corr, wb * inv_h));
        }
    }
}

static int self_hash_head[SELF_HASH_SIZE];
static int self_hash_next[MAX_PARTICLES];

static inline unsigned self_hash(int x, int y, int z)
{
    return (((unsigned)x * 73856093u) ^ ((unsigned)y * 19349663u) ^ ((unsigned)z * 83492791u)) & (SELF_HASH_SIZE - 1);
}

static void resolve_self_collision(SimState *s)
{
    int n = s->grid_n, n2 = n * n;
    float spacing = s->cloth_size / (float)(n - 1);
    float min_dist = 0.75f * spacing;
    float cell = min_dist;

    memset(self_hash_head, -1, sizeof(self_hash_head));
    for (int i = 0; i < n2; i++) {
        Vector3 p = s->particles[i].pos;
        unsigned h = self_hash((int)floorf(p.x / cell), (int)floorf(p.y / cell), (int)floorf(p.z / cell));
        self_hash_next[i] = self_hash_head[h];
        self_hash_head[h] = i;
    }

    for (int a = 0; a < n2; a++) {
        Particle *pa = &s->particles[a];
        int ax = (int)floorf(pa->pos.x / cell);
        int ay = (int)floorf(pa->pos.y / cell);
        int az = (int)floorf(pa->pos.z / cell);
        int ai = a % n, aj = a / n;

        for (int dz = -1; dz <= 1; dz++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
        for (int b = self_hash_head[self_hash(ax + dx, ay + dy, az + dz)]; b >= 0; b = self_hash_next[b]) {
            if (b <= a) continue;
            int bi = b % n, bj = b / n;
            if (abs(ai - bi) <= 1 && abs(aj - bj) <= 1) continue;
            Particle *pb = &s->particles[b];
            Vector3 d = Vector3Subtract(pb->pos, pa->pos);
            float dist_sq = Vector3LengthSqr(d);
            if (dist_sq >= min_dist * min_dist || dist_sq < 1e-12f) continue;
            float wa = pa->inv_mass, wb = pb->inv_mass, wsum = wa + wb;
            if (wsum <= 0.0f) continue;
            float dist = sqrtf(dist_sq);
            Vector3 nrm = Vector3Scale(d, 1.0f / dist);
            float push = (min_dist - dist) / wsum;
            pa->pos = Vector3Subtract(pa->pos, Vector3Scale(nrm, push * wa));
            pb->pos = Vector3Add(pb->pos, Vector3Scale(nrm, push * wb));
            float rel_v = Vector3DotProduct(Vector3Subtract(pb->vel, pa->vel), nrm);
            if (rel_v < 0.0f) {
                float imp = rel_v / wsum;
                pa->vel = Vector3Add(pa->vel, Vector3Scale(nrm, imp * wa));
                pb->vel = Vector3Subtract(pb->vel, Vector3Scale(nrm, imp * wb));
            }
        }
    }
}

static void kill_spring(SimState *s, int idx)
{
    if (idx < 0 || !s->springs[idx].alive) return;
    s->springs[idx].alive = false;
    s->topo_dirty = true;
}

static void apply_tearing(SimState *s)
{
    if (s->tear_ratio <= 0.0f) return;
    int n = s->grid_n;
    for (int k = 0; k < s->spring_count; k++) {
        Spring *sp = &s->springs[k];
        if (!sp->alive) continue;
        float limit = sp->rest_len * s->tear_ratio *
                      ((sp->type == SPRING_STRUCTURAL) ? 1.0f : TEAR_SOFT_MULT);
        if (Vector3DistanceSqr(s->particles[sp->a].pos, s->particles[sp->b].pos) <= limit * limit)
            continue;

        sp->alive = false;
        s->topo_dirty = true;
        if (sp->type != SPRING_STRUCTURAL) continue;

        int a = sp->a, i = a % n, j = a / n;
        if (sp->b == a + 1) {
            if (j > 0) {
                kill_spring(s, s->shear_main[particle_idx(i, j - 1, n)]);
                kill_spring(s, s->shear_anti[particle_idx(i, j - 1, n)]);
            }
            kill_spring(s, s->shear_main[a]);
            kill_spring(s, s->shear_anti[a]);
            if (i > 0) kill_spring(s, s->bend_right[particle_idx(i - 1, j, n)]);
            kill_spring(s, s->bend_right[a]);
        } else {
            if (i > 0) {
                kill_spring(s, s->shear_main[particle_idx(i - 1, j, n)]);
                kill_spring(s, s->shear_anti[particle_idx(i - 1, j, n)]);
            }
            kill_spring(s, s->shear_main[a]);
            kill_spring(s, s->shear_anti[a]);
            if (j > 0) kill_spring(s, s->bend_down[particle_idx(i, j - 1, n)]);
            kill_spring(s, s->bend_down[a]);
        }
    }
}

static Vector3 wind_velocity(const SimState *s, Vector3 at)
{
    if (s->wind_power == 0.0f) return (Vector3){ 0 };
    float gust = s->wind_power * (0.65f + 0.35f * sinf(s->sim_time * 1.7f)
                                       + 0.20f * sinf(s->sim_time * 4.3f));
    float ripple = sinf(at.z * 2.1f + s->sim_time * 3.0f) * 0.3f + 1.0f;
    return (Vector3){ gust * ripple, 0.0f, gust * -0.35f * ripple };
}

static void step_ball(SimState *s, float h)
{
    if (!s->ball_active) return;
    s->ball_vel.y -= s->gravity * h;
    s->ball_pos = Vector3Add(s->ball_pos, Vector3Scale(s->ball_vel, h));

    if (s->ball_pos.y < GROUND_Y + BALL_RADIUS) {
        s->ball_pos.y = GROUND_Y + BALL_RADIUS;
        if (s->ball_vel.y < 0.0f) s->ball_vel.y *= -0.45f;
        if (fabsf(s->ball_vel.y) < 0.3f) s->ball_vel.y = 0.0f;
        float keep = fmaxf(0.0f, 1.0f - 2.0f * h);
        s->ball_vel.x *= keep;
        s->ball_vel.z *= keep;
    }

    if (s->collider == COLLIDER_SPHERE) {
        Vector3 d = Vector3Subtract(s->ball_pos, sphere_center(s));
        float dist = Vector3Length(d);
        float min_d = sphere_radius(s) + BALL_RADIUS;
        if (dist < min_d && dist > 1e-6f) {
            Vector3 nrm = Vector3Scale(d, 1.0f / dist);
            s->ball_pos = Vector3Add(sphere_center(s), Vector3Scale(nrm, min_d));
            float vn = Vector3DotProduct(s->ball_vel, nrm);
            if (vn < 0.0f) s->ball_vel = Vector3Subtract(s->ball_vel, Vector3Scale(nrm, 1.4f * vn));
        }
    } else if (s->collider == COLLIDER_CUBE) {
        Vector3 c = cube_center(s), half = cube_half(s);
        Vector3 lo = Vector3Subtract(c, half), hi = Vector3Add(c, half);
        Vector3 q = { Clamp(s->ball_pos.x, lo.x, hi.x),
                      Clamp(s->ball_pos.y, lo.y, hi.y),
                      Clamp(s->ball_pos.z, lo.z, hi.z) };
        Vector3 d = Vector3Subtract(s->ball_pos, q);
        float dist = Vector3Length(d);
        if (dist < BALL_RADIUS) {
            Vector3 nrm = (dist > 1e-6f) ? Vector3Scale(d, 1.0f / dist) : (Vector3){ 0, 1, 0 };
            if (dist <= 1e-6f) q.y = hi.y;
            s->ball_pos = Vector3Add(q, Vector3Scale(nrm, BALL_RADIUS));
            float vn = Vector3DotProduct(s->ball_vel, nrm);
            if (vn < 0.0f) s->ball_vel = Vector3Subtract(s->ball_vel, Vector3Scale(nrm, 1.4f * vn));
        }
    } else if (s->collider == COLLIDER_MESH) {
        mesh_push_ball(s, &s->ball_pos, &s->ball_vel, BALL_RADIUS);
    }
}

static void ball_vs_cloth(SimState *s, float h)
{
    if (!s->ball_active) return;
    int n2 = s->grid_n * s->grid_n;
    float outer = BALL_RADIUS + s->collision_shell;
    float backstop = BALL_RADIUS + 0.25f * s->collision_shell;
    float omega = 2.0f * CONTACT_VREF / s->collision_shell;
    float kc = PARTICLE_MASS * omega * omega;
    float e = (s->restitution < 0.01f) ? 0.01f : s->restitution;
    float ln_e = logf(e);
    float zeta = -ln_e / sqrtf(PI * PI + ln_e * ln_e);
    float cd = 2.0f * zeta * sqrtf(kc * PARTICLE_MASS);
    float inv_mb = 1.0f / BALL_MASS;

    for (int i = 0; i < n2; i++) {
        Particle *p = &s->particles[i];
        Vector3 d = Vector3Subtract(p->pos, s->ball_pos);
        float dist = Vector3Length(d);
        if (dist >= outer || dist < 1e-6f) continue;
        Vector3 nrm = Vector3Scale(d, 1.0f / dist);

        if (dist < backstop && p->inv_mass > 0.0f) {
            p->pos = Vector3Add(s->ball_pos, Vector3Scale(nrm, backstop));
            float rel = Vector3DotProduct(Vector3Subtract(p->vel, s->ball_vel), nrm);
            if (rel < 0.0f) {
                float imp = rel / (p->inv_mass + inv_mb);
                p->vel = Vector3Subtract(p->vel, Vector3Scale(nrm, imp * p->inv_mass));
                s->ball_vel = Vector3Add(s->ball_vel, Vector3Scale(nrm, imp * inv_mb));
            }
            dist = backstop;
        }

        float rel_vn = Vector3DotProduct(Vector3Subtract(p->vel, s->ball_vel), nrm);
        float fn = kc * (outer - dist) - cd * rel_vn;
        if (fn > 0.0f) {
            if (p->inv_mass > 0.0f)
                p->vel = Vector3Add(p->vel, Vector3Scale(nrm, fn * p->inv_mass * h));
            s->ball_vel = Vector3Subtract(s->ball_vel, Vector3Scale(nrm, fn * inv_mb * h));
        }
    }
}

static void physics_substep(SimState *s, float h)
{
    int n = s->grid_n, n2 = n * n;

    for (int i = 0; i < n2; i++) {
        Particle *p = &s->particles[i];
        p->force = (Vector3){ 0, -s->gravity * PARTICLE_MASS, 0 };
        p->force = Vector3Add(p->force, Vector3Scale(p->vel, -AIR_DRAG * PARTICLE_MASS));
    }

    {
        float c_aero = AERO_DRAG * PARTICLE_MASS * (float)n2 / (s->cloth_size * s->cloth_size);
        for (int j = 0; j < n - 1; j++)
        for (int i = 0; i < n - 1; i++) {
            if (!quad_alive(s, i, j)) continue;
            int q[4] = {
                particle_idx(i, j, n), particle_idx(i + 1, j, n),
                particle_idx(i + 1, j + 1, n), particle_idx(i, j + 1, n)
            };
            for (int t = 0; t < 2; t++) {
                Particle *p0 = &s->particles[q[0]];
                Particle *p1 = &s->particles[q[t + 1]];
                Particle *p2 = &s->particles[q[t + 2]];
                Vector3 nA = Vector3Scale(Vector3CrossProduct(
                    Vector3Subtract(p1->pos, p0->pos),
                    Vector3Subtract(p2->pos, p0->pos)), 0.5f);
                float area = Vector3Length(nA);
                if (area < 1e-9f) continue;
                Vector3 centroid = Vector3Scale(
                    Vector3Add(Vector3Add(p0->pos, p1->pos), p2->pos), 1.0f / 3.0f);
                Vector3 v_rel = Vector3Scale(
                    Vector3Add(Vector3Add(p0->vel, p1->vel), p2->vel), 1.0f / 3.0f);
                v_rel = Vector3Subtract(v_rel, wind_velocity(s, centroid));
                float speed = Vector3Length(v_rel);
                if (speed < 1e-6f) continue;
                Vector3 n_hat = Vector3Scale(nA, 1.0f / area);
                float vn = Vector3DotProduct(v_rel, n_hat);
                Vector3 f = Vector3Scale(n_hat, -c_aero * area * speed * vn / 3.0f);
                p0->force = Vector3Add(p0->force, f);
                p1->force = Vector3Add(p1->force, f);
                p2->force = Vector3Add(p2->force, f);
            }
        }
    }

    for (int i = 0; i < s->spring_count; i++) {
        Spring *sp = &s->springs[i];
        if (!sp->alive) continue;
        Particle *pa = &s->particles[sp->a];
        Particle *pb = &s->particles[sp->b];
        Vector3 d = Vector3Subtract(pb->pos, pa->pos);
        float len = Vector3Length(d);
        if (len < 1e-7f) continue;
        Vector3 dhat = Vector3Scale(d, 1.0f / len);
        float stretch = len - sp->rest_len;
        float rel_vel = Vector3DotProduct(Vector3Subtract(pb->vel, pa->vel), dhat);
        float k = (sp->type == SPRING_BENDING) ? s->stiffness * s->bend : s->stiffness;
        float mag = k * stretch + s->damping * rel_vel;
        Vector3 f = Vector3Scale(dhat, mag);
        pa->force = Vector3Add(pa->force, f);
        pb->force = Vector3Subtract(pb->force, f);
    }

    for (int i = 0; i < n2; i++) {
        Particle *p = &s->particles[i];
        if (p->inv_mass == 0.0f) continue;
        p->prev_pos = p->pos;
        p->vel = Vector3Add(p->vel, Vector3Scale(p->force, p->inv_mass * h));
        p->pos = Vector3Add(p->pos, Vector3Scale(p->vel, h));
    }

    apply_tearing(s);
    apply_strain_limiting(s, h);
    if (s->self_collide) resolve_self_collision(s);
    step_ball(s, h);
    ball_vs_cloth(s, h);
    resolve_colliders(s, h);
}

void step_frame(SimState *s)
{
    float omega = sqrtf(s->stiffness / PARTICLE_MASS);
    float contact_omega = 2.0f * CONTACT_VREF / s->collision_shell;
    if (contact_omega > omega) omega = contact_omega;
    int substeps = (int)ceilf(FRAME_DT * omega / 0.7f);
    if (substeps < 4) substeps = 4;
    if (substeps > 120) substeps = 120;

    float h = FRAME_DT / (float)substeps;
    for (int i = 0; i < substeps; i++) {
        physics_substep(s, h);
        s->sim_time += h;
    }
}
