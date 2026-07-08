#include "facefall.h"

static void contact_absorb(SimState *s, Particle *p, Vector3 normal)
{
    float vn = Vector3DotProduct(p->vel, normal);
    Vector3 v_normal = Vector3Scale(normal, vn);
    Vector3 v_tangent = Vector3Subtract(p->vel, v_normal);
    if (vn < 0.0f) v_normal = (Vector3){ 0 };
    v_tangent = Vector3Scale(v_tangent, s->friction);
    if (Vector3LengthSqr(v_tangent) < STICK_SPEED * STICK_SPEED)
        v_tangent = (Vector3){ 0 };
    p->vel = Vector3Add(v_normal, v_tangent);
}

static void contact_penalty(SimState *s, Particle *p, Vector3 normal, float depth, float h)
{
    if (depth <= 0.0f || p->inv_mass == 0.0f) return;

    float omega = 2.0f * CONTACT_VREF / s->collision_shell;
    float kc = PARTICLE_MASS * omega * omega;
    float e = (s->restitution < 0.01f) ? 0.01f : s->restitution;
    float ln_e = logf(e);
    float zeta = -ln_e / sqrtf(PI * PI + ln_e * ln_e);
    float cd = 2.0f * zeta * sqrtf(kc * PARTICLE_MASS);

    float vn = Vector3DotProduct(p->vel, normal);
    float fn = kc * depth - cd * vn;
    if (fn > 0.0f) {
        p->vel = Vector3Add(p->vel, Vector3Scale(normal, fn * p->inv_mass * h));
        vn = Vector3DotProduct(p->vel, normal);
        Vector3 v_n = Vector3Scale(normal, vn);
        Vector3 v_t = Vector3Subtract(p->vel, v_n);
        v_t = Vector3Scale(v_t, s->friction);
        if (Vector3LengthSqr(v_t) < STICK_SPEED * STICK_SPEED)
            v_t = (Vector3){ 0 };
        p->vel = Vector3Add(v_n, v_t);
    }
}

static void collide_sphere(SimState *s, Particle *p, float h)
{
    Vector3 centre = sphere_center(s);
    Vector3 d = Vector3Subtract(p->pos, centre);
    float dist = Vector3Length(d);
    float outer = sphere_radius(s) + s->collision_shell;
    if (dist >= outer || dist < 1e-6f) return;

    Vector3 n = Vector3Scale(d, 1.0f / dist);
    float backstop = sphere_radius(s) + 0.25f * s->collision_shell;
    if (dist < backstop) {
        p->pos = Vector3Add(centre, Vector3Scale(n, backstop));
        contact_absorb(s, p, n);
        dist = backstop;
    }
    contact_penalty(s, p, n, outer - dist, h);
}

static void collide_cube(SimState *s, Particle *p, float h)
{
    Vector3 centre = cube_center(s), half = cube_half(s);
    Vector3 lo = Vector3Subtract(centre, half);
    Vector3 hi = Vector3Add(centre, half);

    bool inside = (p->pos.x > lo.x && p->pos.x < hi.x &&
                   p->pos.y > lo.y && p->pos.y < hi.y &&
                   p->pos.z > lo.z && p->pos.z < hi.z);

    if (!inside) {
        Vector3 closest = {
            Clamp(p->pos.x, lo.x, hi.x),
            Clamp(p->pos.y, lo.y, hi.y),
            Clamp(p->pos.z, lo.z, hi.z)
        };
        Vector3 d = Vector3Subtract(p->pos, closest);
        float dist = Vector3Length(d);
        if (dist >= s->collision_shell || dist < 1e-6f) return;
        Vector3 n = Vector3Scale(d, 1.0f / dist);
        float backstop = 0.25f * s->collision_shell;
        if (dist < backstop) {
            p->pos = Vector3Add(closest, Vector3Scale(n, backstop));
            contact_absorb(s, p, n);
            dist = backstop;
        }
        contact_penalty(s, p, n, s->collision_shell - dist, h);
    } else {
        float dx1 = p->pos.x - lo.x, dx2 = hi.x - p->pos.x;
        float dy1 = p->pos.y - lo.y, dy2 = hi.y - p->pos.y;
        float dz1 = p->pos.z - lo.z, dz2 = hi.z - p->pos.z;
        float best = dx1; Vector3 n = { -1, 0, 0 };
        if (dx2 < best) { best = dx2; n = (Vector3){ 1, 0, 0 }; }
        if (dy1 < best) { best = dy1; n = (Vector3){ 0,-1, 0 }; }
        if (dy2 < best) { best = dy2; n = (Vector3){ 0, 1, 0 }; }
        if (dz1 < best) { best = dz1; n = (Vector3){ 0, 0,-1 }; }
        if (dz2 < best) { best = dz2; n = (Vector3){ 0, 0, 1 }; }
        p->pos = Vector3Add(p->pos, Vector3Scale(n, best + 0.25f * s->collision_shell));
        contact_absorb(s, p, n);
        contact_penalty(s, p, n, 0.75f * s->collision_shell, h);
    }
}

static Vector3 closest_point_on_tri(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
{
    Vector3 ab = Vector3Subtract(b, a);
    Vector3 ac = Vector3Subtract(c, a);
    Vector3 ap = Vector3Subtract(p, a);
    float d1 = Vector3DotProduct(ab, ap);
    float d2 = Vector3DotProduct(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    Vector3 bp = Vector3Subtract(p, b);
    float d3 = Vector3DotProduct(ab, bp);
    float d4 = Vector3DotProduct(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        return Vector3Add(a, Vector3Scale(ab, d1 / (d1 - d3)));
    Vector3 cp = Vector3Subtract(p, c);
    float d5 = Vector3DotProduct(ab, cp);
    float d6 = Vector3DotProduct(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        return Vector3Add(a, Vector3Scale(ac, d2 / (d2 - d6)));
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        return Vector3Add(b, Vector3Scale(Vector3Subtract(c, b),
                              (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    float denom = 1.0f / (va + vb + vc);
    return Vector3Add(a, Vector3Add(Vector3Scale(ab, vb * denom),
                                    Vector3Scale(ac, vc * denom)));
}

static inline int grid_coord(float v, float origin, float cell, int n)
{
    int c = (int)((v - origin) / cell);
    if (c < 0) c = 0;
    if (c > n - 1) c = n - 1;
    return c;
}

static Vector3 tri_normal(const MeshCollider *mc, int t)
{
    Vector3 *tri = &mc->tri[t * 3];
    Vector3 cr = Vector3CrossProduct(Vector3Subtract(tri[1], tri[0]),
                                     Vector3Subtract(tri[2], tri[0]));
    return (Vector3LengthSqr(cr) > 1e-12f) ? Vector3Normalize(cr)
                                           : (Vector3){ 0, 1, 0 };
}

static bool segment_hits_tri(Vector3 o, Vector3 d, Vector3 a, Vector3 b, Vector3 c, float *t_out)
{
    Vector3 e1 = Vector3Subtract(b, a);
    Vector3 e2 = Vector3Subtract(c, a);
    Vector3 pv = Vector3CrossProduct(d, e2);
    float det = Vector3DotProduct(e1, pv);
    if (fabsf(det) < 1e-12f) return false;
    float inv = 1.0f / det;
    Vector3 tv = Vector3Subtract(o, a);
    float u = Vector3DotProduct(tv, pv) * inv;
    if (u < -0.001f || u > 1.001f) return false;
    Vector3 qv = Vector3CrossProduct(tv, e1);
    float v = Vector3DotProduct(d, qv) * inv;
    if (v < -0.001f || u + v > 1.001f) return false;
    float t = Vector3DotProduct(e2, qv) * inv;
    if (t < 0.0f || t > 1.0f) return false;
    *t_out = t;
    return true;
}

static void collide_mesh(SimState *s, Particle *p, float h)
{
    (void)h;
    const MeshCollider *mc = &s->mesh_collider;
    if (mc->tri_count == 0) return;

    Vector3 disp = Vector3Subtract(p->pos, p->prev_pos);
    float travel = Vector3Length(disp);
    float reach = s->collision_shell + travel;

    BoundingBox bb = mc->bounds;
    if (p->pos.x < bb.min.x - reach || p->pos.x > bb.max.x + reach ||
        p->pos.y < bb.min.y - reach || p->pos.y > bb.max.y + reach ||
        p->pos.z < bb.min.z - reach || p->pos.z > bb.max.z + reach) return;

    float best_dist_sq = reach * reach;
    Vector3 best_point = { 0 };
    int best_tri = -1;
    float best_t = 2.0f;
    int sweep_tri = -1;

    Vector3 lo = Vector3Min(p->pos, p->prev_pos);
    Vector3 hi = Vector3Max(p->pos, p->prev_pos);
    int cx0 = grid_coord(lo.x - s->collision_shell, mc->grid_min.x, mc->cell_size.x, mc->grid_nx);
    int cx1 = grid_coord(hi.x + s->collision_shell, mc->grid_min.x, mc->cell_size.x, mc->grid_nx);
    int cy0 = grid_coord(lo.y - s->collision_shell, mc->grid_min.y, mc->cell_size.y, mc->grid_ny);
    int cy1 = grid_coord(hi.y + s->collision_shell, mc->grid_min.y, mc->cell_size.y, mc->grid_ny);
    int cz0 = grid_coord(lo.z - s->collision_shell, mc->grid_min.z, mc->cell_size.z, mc->grid_nz);
    int cz1 = grid_coord(hi.z + s->collision_shell, mc->grid_min.z, mc->cell_size.z, mc->grid_nz);

    for (int cz = cz0; cz <= cz1; cz++)
    for (int cy = cy0; cy <= cy1; cy++)
    for (int cx = cx0; cx <= cx1; cx++) {
        int cell = (cz * mc->grid_ny + cy) * mc->grid_nx + cx;
        for (int k = mc->grid_start[cell]; k < mc->grid_start[cell + 1]; k++) {
            int t = mc->grid_items[k];
            Vector3 *tri = &mc->tri[t * 3];
            if (travel > 1e-8f) {
                float tc;
                if (segment_hits_tri(p->prev_pos, disp, tri[0], tri[1], tri[2], &tc) && tc < best_t) {
                    best_t = tc;
                    sweep_tri = t;
                }
            }
            Vector3 q = closest_point_on_tri(p->pos, tri[0], tri[1], tri[2]);
            float dsq = Vector3DistanceSqr(p->pos, q);
            if (dsq < best_dist_sq) {
                best_dist_sq = dsq;
                best_point = q;
                best_tri = t;
            }
        }
    }

    if (sweep_tri >= 0) {
        Vector3 face_n = tri_normal(mc, sweep_tri);
        Vector3 n = (Vector3DotProduct(disp, face_n) < 0.0f) ? face_n : Vector3Negate(face_n);
        p->pos = p->prev_pos;
        contact_absorb(s, p, n);
        return;
    }

    if (best_tri < 0) return;

    Vector3 face_n = tri_normal(mc, best_tri);
    Vector3 d = Vector3Subtract(p->pos, best_point);
    float dist = sqrtf(best_dist_sq);
    float backstop = 0.25f * s->collision_shell;

    if (Vector3DotProduct(d, face_n) < 0.0f) {
        p->pos = Vector3Add(best_point, Vector3Scale(face_n, backstop));
        contact_absorb(s, p, face_n);
        contact_penalty(s, p, face_n, s->collision_shell - backstop, h);
    } else if (dist < s->collision_shell) {
        Vector3 n = (dist > 1e-5f) ? Vector3Scale(d, 1.0f / dist) : face_n;
        if (dist < backstop) {
            p->pos = Vector3Add(best_point, Vector3Scale(n, backstop));
            contact_absorb(s, p, n);
            dist = backstop;
        }
        contact_penalty(s, p, n, s->collision_shell - dist, h);
    }
}

static void collide_ground(SimState *s, Particle *p, float h)
{
    float y = p->pos.y - GROUND_Y;
    if (y >= s->collision_shell) return;
    Vector3 up = { 0, 1, 0 };
    float backstop = 0.25f * s->collision_shell;
    if (y < backstop) {
        p->pos.y = GROUND_Y + backstop;
        contact_absorb(s, p, up);
        y = backstop;
    }
    contact_penalty(s, p, up, s->collision_shell - y, h);
}

void resolve_colliders(SimState *s, float h)
{
    int n2 = s->grid_n * s->grid_n;
    for (int i = 0; i < n2; i++) {
        Particle *p = &s->particles[i];
        if (p->inv_mass == 0.0f) continue;
        switch (s->collider) {
        case COLLIDER_SPHERE: collide_sphere(s, p, h); break;
        case COLLIDER_CUBE:   collide_cube(s, p, h);   break;
        case COLLIDER_MESH:   collide_mesh(s, p, h);   break;
        default: break;
        }
        collide_ground(s, p, h);
    }
}

void mesh_push_ball(SimState *s, Vector3 *pos, Vector3 *vel, float radius)
{
    const MeshCollider *mc = &s->mesh_collider;
    if (mc->tri_count == 0) return;

    BoundingBox bb = mc->bounds;
    if (pos->x < bb.min.x - radius || pos->x > bb.max.x + radius ||
        pos->y < bb.min.y - radius || pos->y > bb.max.y + radius ||
        pos->z < bb.min.z - radius || pos->z > bb.max.z + radius) return;

    float best_sq = radius * radius;
    Vector3 best_q = { 0 };
    int best_tri = -1;

    int cx0 = grid_coord(pos->x - radius, mc->grid_min.x, mc->cell_size.x, mc->grid_nx);
    int cx1 = grid_coord(pos->x + radius, mc->grid_min.x, mc->cell_size.x, mc->grid_nx);
    int cy0 = grid_coord(pos->y - radius, mc->grid_min.y, mc->cell_size.y, mc->grid_ny);
    int cy1 = grid_coord(pos->y + radius, mc->grid_min.y, mc->cell_size.y, mc->grid_ny);
    int cz0 = grid_coord(pos->z - radius, mc->grid_min.z, mc->cell_size.z, mc->grid_nz);
    int cz1 = grid_coord(pos->z + radius, mc->grid_min.z, mc->cell_size.z, mc->grid_nz);

    for (int cz = cz0; cz <= cz1; cz++)
    for (int cy = cy0; cy <= cy1; cy++)
    for (int cx = cx0; cx <= cx1; cx++) {
        int cell = (cz * mc->grid_ny + cy) * mc->grid_nx + cx;
        for (int k = mc->grid_start[cell]; k < mc->grid_start[cell + 1]; k++) {
            int t = mc->grid_items[k];
            Vector3 *tri = &mc->tri[t * 3];
            Vector3 q = closest_point_on_tri(*pos, tri[0], tri[1], tri[2]);
            float dsq = Vector3DistanceSqr(*pos, q);
            if (dsq < best_sq) {
                best_sq = dsq;
                best_q = q;
                best_tri = t;
            }
        }
    }

    if (best_tri < 0) return;

    float dist = sqrtf(best_sq);
    Vector3 face_n = tri_normal(mc, best_tri);
    Vector3 d = Vector3Subtract(*pos, best_q);
    Vector3 nrm;
    if (dist > 1e-5f && Vector3DotProduct(d, face_n) >= 0.0f)
        nrm = Vector3Scale(d, 1.0f / dist);
    else
        nrm = face_n;

    *pos = Vector3Add(best_q, Vector3Scale(nrm, radius));
    float vn = Vector3DotProduct(*vel, nrm);
    if (vn < 0.0f) *vel = Vector3Subtract(*vel, Vector3Scale(nrm, 1.4f * vn));
}

void bake_mesh_collider(SimState *s)
{
    Model *model = &s->mesh_collider.model;
    if (model->meshCount == 0) return;

    BoundingBox raw = GetMeshBoundingBox(model->meshes[0]);
    Vector3 size = Vector3Subtract(raw.max, raw.min);
    Vector3 centre = Vector3Scale(Vector3Add(raw.min, raw.max), 0.5f);
    float footprint = fmaxf(size.x, size.z);
    if (footprint < 1e-6f) footprint = 1.0f;
    float scale = 2.3f * s->collider_scale / footprint;

    Mesh mesh = model->meshes[0];
    int tri_count = (mesh.indices != NULL) ? mesh.triangleCount : mesh.vertexCount / 3;

    MemFree(s->mesh_collider.tri);
    MemFree(s->mesh_collider.grid_start);
    MemFree(s->mesh_collider.grid_items);

    s->mesh_collider.tri = (Vector3 *)MemAlloc(sizeof(Vector3) * 3 * tri_count);
    s->mesh_collider.tri_count = tri_count;

    for (int t = 0; t < tri_count; t++) {
        for (int k = 0; k < 3; k++) {
            int vi = (mesh.indices != NULL) ? mesh.indices[t * 3 + k] : (t * 3 + k);
            Vector3 v = { mesh.vertices[vi * 3 + 0], mesh.vertices[vi * 3 + 1], mesh.vertices[vi * 3 + 2] };
            v = Vector3Scale(Vector3Subtract(v, centre), scale);
            v.y += (size.y * 0.5f) * scale;
            s->mesh_collider.tri[t * 3 + k] = v;
        }
    }

    Vector3 mn = s->mesh_collider.tri[0], mx = s->mesh_collider.tri[0];
    for (int i = 1; i < tri_count * 3; i++) {
        mn = Vector3Min(mn, s->mesh_collider.tri[i]);
        mx = Vector3Max(mx, s->mesh_collider.tri[i]);
    }
    s->mesh_collider.bounds.min = Vector3SubtractValue(mn, 0.35f);
    s->mesh_collider.bounds.max = Vector3AddValue(mx, 0.35f);

    {
        Vector3 ext = Vector3Subtract(mx, mn);
        float target = fmaxf(fmaxf(ext.x, ext.y), ext.z) / 16.0f;
        if (target < 1e-4f) target = 1.0f;
        int nx = (int)ceilf(ext.x / target); nx = (nx < 1) ? 1 : (nx > 32) ? 32 : nx;
        int ny = (int)ceilf(ext.y / target); ny = (ny < 1) ? 1 : (ny > 32) ? 32 : ny;
        int nz = (int)ceilf(ext.z / target); nz = (nz < 1) ? 1 : (nz > 32) ? 32 : nz;
        Vector3 cs = {
            (ext.x > 1e-6f) ? ext.x / nx : 1.0f,
            (ext.y > 1e-6f) ? ext.y / ny : 1.0f,
            (ext.z > 1e-6f) ? ext.z / nz : 1.0f
        };
        s->mesh_collider.grid_nx = nx;
        s->mesh_collider.grid_ny = ny;
        s->mesh_collider.grid_nz = nz;
        s->mesh_collider.grid_min = mn;
        s->mesh_collider.cell_size = cs;

        int cells = nx * ny * nz;
        int *start = (int *)MemAlloc(sizeof(int) * (cells + 1));
        int *count = (int *)MemAlloc(sizeof(int) * cells);

        for (int pass = 0; pass < 2; pass++) {
            for (int t = 0; t < tri_count; t++) {
                Vector3 *tri = &s->mesh_collider.tri[t * 3];
                Vector3 tmn = Vector3Min(tri[0], Vector3Min(tri[1], tri[2]));
                Vector3 tmx = Vector3Max(tri[0], Vector3Max(tri[1], tri[2]));
                int x0 = grid_coord(tmn.x, mn.x, cs.x, nx), x1 = grid_coord(tmx.x, mn.x, cs.x, nx);
                int y0 = grid_coord(tmn.y, mn.y, cs.y, ny), y1 = grid_coord(tmx.y, mn.y, cs.y, ny);
                int z0 = grid_coord(tmn.z, mn.z, cs.z, nz), z1 = grid_coord(tmx.z, mn.z, cs.z, nz);
                for (int cz = z0; cz <= z1; cz++)
                for (int cy = y0; cy <= y1; cy++)
                for (int cx = x0; cx <= x1; cx++) {
                    int cell = (cz * ny + cy) * nx + cx;
                    if (pass == 0) count[cell]++;
                    else s->mesh_collider.grid_items[start[cell] + count[cell]++] = t;
                }
            }
            if (pass == 0) {
                start[0] = 0;
                for (int c = 0; c < cells; c++) start[c + 1] = start[c] + count[c];
                s->mesh_collider.grid_items = (int *)MemAlloc(sizeof(int) * start[cells]);
                memset(count, 0, sizeof(int) * cells);
            }
        }
        s->mesh_collider.grid_start = start;
        MemFree(count);
    }

    model->transform = MatrixMultiply(
        MatrixMultiply(MatrixTranslate(-centre.x, -centre.y, -centre.z),
                       MatrixScale(scale, scale, scale)),
        MatrixTranslate(0, (size.y * 0.5f) * scale, 0));

    TraceLog(LOG_INFO, "FACEFALL: mesh baked (%d tris, x%.1f, grid %dx%dx%d)",
             tri_count, s->collider_scale, s->mesh_collider.grid_nx,
             s->mesh_collider.grid_ny, s->mesh_collider.grid_nz);
}

void init_mesh_collider(SimState *s)
{
    Model model;
    bool from_file = false;

    if (FileExists("assets/model.obj")) {
        model = LoadModel("assets/model.obj");
        if (model.meshCount > 0 && model.meshes[0].vertexCount > 0) from_file = true;
        else TraceLog(LOG_WARNING, "FACEFALL: model.obj loaded but empty, using torus");
    }
    if (!from_file) {
        model = LoadModelFromMesh(GenMeshTorus(0.45f, 2.0f, 24, 32));
        TraceLog(LOG_WARNING, "FACEFALL: model.obj not found, generated torus");
    }

    s->mesh_collider.model = model;
    s->mesh_collider.from_file = from_file;
    bake_mesh_collider(s);
}
