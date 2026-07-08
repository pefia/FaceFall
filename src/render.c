#include "facefall.h"

static const char *cloth_vs =
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

static const char *cloth_fs =
    "#version 330\n"
    "in vec3 fragPos; in vec2 fragTexCoord; in vec3 fragNormal; in vec4 fragColor;\n"
    "uniform vec4 colDiffuse; uniform vec3 lightDir; uniform vec3 viewPos;\n"
    "uniform float uSpecular; uniform float uRoughness; uniform float uCloth;\n"
    "out vec4 finalColor;\n"
    "void main(){\n"
    "  vec3 N = normalize(fragNormal);\n"
    "  vec3 V = normalize(viewPos - fragPos);\n"
    "  if (dot(N,V) < 0.0) N = -N;\n"
    "  vec3 L = normalize(-lightDir);\n"
    "  if (uCloth > 0.5) {\n"
    "    vec3 T = normalize(cross(N, vec3(0.0,1.0,0.0)) + vec3(1e-4));\n"
    "    vec3 B = cross(N, T);\n"
    "    N = normalize(N + T*cos(fragTexCoord.x*140.0)*0.04 + B*cos(fragTexCoord.y*140.0)*0.04);\n"
    "  }\n"
    "  float NdotL = max(dot(N,L), 0.0);\n"
    "  float wrap = max((NdotL + 0.3)/1.3, 0.0);\n"
    "  vec3 base = (colDiffuse*fragColor).rgb;\n"
    "  vec3 col = base*0.25 + base*wrap*0.9;\n"
    "  vec3 H = normalize(L+V);\n"
    "  float shin = mix(120.0, 8.0, clamp(uRoughness,0.0,1.0));\n"
    "  col += vec3(pow(max(dot(N,H),0.0), shin) * uSpecular * NdotL);\n"
    "  float rim = pow(1.0 - max(dot(N,V),0.0), 3.0);\n"
    "  col += vec3(0.5,0.6,0.8)*rim*0.6;\n"
    "  if (uCloth > 0.5) col += base*pow(max(dot(-N,L),0.0),2.0)*0.4;\n"
    "  finalColor = vec4(col, 1.0);\n"
    "}\n";

bool renderer_init(SimState *s)
{
    s->shader = LoadShaderFromMemory(cloth_vs, cloth_fs);
    s->shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(s->shader, "viewPos");
    s->loc_light_dir = GetShaderLocation(s->shader, "lightDir");
    s->loc_view_pos  = s->shader.locs[SHADER_LOC_VECTOR_VIEW];
    s->loc_spec      = GetShaderLocation(s->shader, "uSpecular");
    s->loc_rough     = GetShaderLocation(s->shader, "uRoughness");
    s->loc_cloth     = GetShaderLocation(s->shader, "uCloth");

    s->cloth_mat = LoadMaterialDefault();
    s->cloth_mat.shader = s->shader;
    s->cloth_mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 226, 56, 82, 255 };

    s->collider_mat = LoadMaterialDefault();
    s->collider_mat.shader = s->shader;
    s->collider_mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 90, 140, 220, 255 };

    s->sphere_mesh = GenMeshSphere(1.0f, 24, 32);
    s->cube_mesh   = GenMeshCube(1.0f, 1.0f, 1.0f);
    return true;
}

void renderer_cleanup(SimState *s)
{
    if (s->cloth_mesh_ready) UnloadMesh(s->cloth_mesh);
    UnloadMesh(s->sphere_mesh);
    UnloadMesh(s->cube_mesh);
    UnloadShader(s->shader);
    UnloadModel(s->mesh_collider.model);
    MemFree(s->mesh_collider.tri);
    MemFree(s->mesh_collider.grid_start);
    MemFree(s->mesh_collider.grid_items);
}

static void fill_cloth_vertices(SimState *s)
{
    int n = s->grid_n;
    float *V = s->cloth_mesh.vertices;
    float *N = s->cloth_mesh.normals;

    for (int v = 0; v < n * n; v++) {
        Vector3 p = s->particles[v].pos;
        V[v*3+0] = p.x; V[v*3+1] = p.y; V[v*3+2] = p.z;
        N[v*3+0] = N[v*3+1] = N[v*3+2] = 0.0f;
    }
    for (int j = 0; j < n - 1; j++)
    for (int i = 0; i < n - 1; i++) {
        if (!quad_alive(s, i, j)) continue;
        int i00 = particle_idx(i, j, n), i10 = particle_idx(i+1, j, n);
        int i01 = particle_idx(i, j+1, n), i11 = particle_idx(i+1, j+1, n);
        Vector3 fn = Vector3CrossProduct(
            Vector3Subtract(s->particles[i10].pos, s->particles[i00].pos),
            Vector3Subtract(s->particles[i01].pos, s->particles[i00].pos));
        int c[4] = { i00, i10, i01, i11 };
        for (int k = 0; k < 4; k++) {
            N[c[k]*3+0] += fn.x; N[c[k]*3+1] += fn.y; N[c[k]*3+2] += fn.z;
        }
    }
    for (int v = 0; v < n * n; v++) {
        Vector3 nn = { N[v*3+0], N[v*3+1], N[v*3+2] };
        float lsq = Vector3LengthSqr(nn);
        nn = (lsq > 1e-12f) ? Vector3Scale(nn, 1.0f/sqrtf(lsq)) : (Vector3){ 0, 1, 0 };
        N[v*3+0] = nn.x; N[v*3+1] = nn.y; N[v*3+2] = nn.z;
    }
}

void rebuild_cloth_mesh(SimState *s)
{
    if (s->shader.id == 0) return;
    int n = s->grid_n;
    int verts = n * n, tris = 2 * (n - 1) * (n - 1);
    if (s->cloth_mesh_ready && s->cloth_mesh.vertexCount == verts && !s->topo_dirty) {
        fill_cloth_vertices(s);
        return;
    }

    if (s->cloth_mesh_ready) UnloadMesh(s->cloth_mesh);

    Mesh m = { 0 };
    m.vertexCount = verts;
    m.triangleCount = tris;
    m.vertices  = (float *)MemAlloc(sizeof(float) * 3 * verts);
    m.normals   = (float *)MemAlloc(sizeof(float) * 3 * verts);
    m.texcoords = (float *)MemAlloc(sizeof(float) * 2 * verts);
    m.colors    = (unsigned char *)MemAlloc(sizeof(unsigned char) * 4 * verts);
    m.indices   = (unsigned short *)MemAlloc(sizeof(unsigned short) * 3 * tris);
    memset(m.colors, 255, sizeof(unsigned char) * 4 * verts);

    for (int j = 0; j < n; j++)
    for (int i = 0; i < n; i++) {
        int v = j * n + i;
        m.texcoords[v*2+0] = (float)i / (n - 1);
        m.texcoords[v*2+1] = (float)j / (n - 1);
    }
    int t = 0;
    for (int j = 0; j < n - 1; j++)
    for (int i = 0; i < n - 1; i++) {
        unsigned short i00 = j*n+i, i10 = j*n+i+1, i01 = (j+1)*n+i, i11 = (j+1)*n+i+1;
        if (!quad_alive(s, i, j)) i10 = i01 = i11 = i00;
        m.indices[t++] = i00; m.indices[t++] = i10; m.indices[t++] = i11;
        m.indices[t++] = i00; m.indices[t++] = i11; m.indices[t++] = i01;
    }

    s->cloth_mesh = m;
    fill_cloth_vertices(s);
    UploadMesh(&s->cloth_mesh, true);
    s->cloth_mesh_ready = true;
    s->topo_dirty = false;
}

static float heat_accum[MAX_PARTICLES];
static unsigned char heat_count[MAX_PARTICLES];

static Color heat_ramp(float t)
{
    Color a = (t < 0.5f) ? (Color){ 45, 95, 235, 255 }  : (Color){ 65, 215, 120, 255 };
    Color b = (t < 0.5f) ? (Color){ 65, 215, 120, 255 } : (Color){ 250, 60, 50, 255 };
    float f = (t < 0.5f) ? t * 2.0f : t * 2.0f - 1.0f;
    return (Color){ (unsigned char)(a.r + ((int)b.r - a.r) * f),
                    (unsigned char)(a.g + ((int)b.g - a.g) * f),
                    (unsigned char)(a.b + ((int)b.b - a.b) * f), 255 };
}

static void fill_cloth_colors(SimState *s)
{
    int verts = s->grid_n * s->grid_n;
    unsigned char *C = s->cloth_mesh.colors;
    if (!s->heatmap) {
        memset(C, 255, (size_t)4 * verts);
        return;
    }

    memset(heat_accum, 0, sizeof(float) * verts);
    memset(heat_count, 0, sizeof(unsigned char) * verts);
    for (int i = 0; i < s->spring_count; i++) {
        Spring *sp = &s->springs[i];
        if (sp->type != SPRING_STRUCTURAL || !sp->alive) continue;
        float ratio = Vector3Distance(s->particles[sp->a].pos, s->particles[sp->b].pos) / sp->rest_len;
        heat_accum[sp->a] += ratio; heat_count[sp->a]++;
        heat_accum[sp->b] += ratio; heat_count[sp->b]++;
    }

    float inv_range = 1.0f / (MAX_STRETCH - 1.0f);
    for (int v = 0; v < verts; v++) {
        float avg = (heat_count[v] > 0) ? heat_accum[v] / heat_count[v] : 1.0f;
        Color c = heat_ramp(Clamp((avg - 1.0f) * inv_range, 0.0f, 1.0f));
        C[v*4+0] = c.r; C[v*4+1] = c.g; C[v*4+2] = c.b; C[v*4+3] = 255;
    }
}

static void draw_cloth(SimState *s, Camera3D camera)
{
    fill_cloth_vertices(s);
    fill_cloth_colors(s);
    UpdateMeshBuffer(s->cloth_mesh, 0, s->cloth_mesh.vertices, sizeof(float)*3*s->cloth_mesh.vertexCount, 0);
    UpdateMeshBuffer(s->cloth_mesh, 2, s->cloth_mesh.normals,  sizeof(float)*3*s->cloth_mesh.vertexCount, 0);
    UpdateMeshBuffer(s->cloth_mesh, 3, s->cloth_mesh.colors,   sizeof(unsigned char)*4*s->cloth_mesh.vertexCount, 0);
    s->cloth_mat.maps[MATERIAL_MAP_DIFFUSE].color = s->heatmap ? WHITE : (Color){ 226, 56, 82, 255 };

    Vector3 light_dir = Vector3Normalize((Vector3){ 0.5f, -1.0f, 0.35f });
    float cloth = 1.0f;
    SetShaderValue(s->shader, s->loc_light_dir, &light_dir,        SHADER_UNIFORM_VEC3);
    SetShaderValue(s->shader, s->loc_view_pos,  &camera.position,  SHADER_UNIFORM_VEC3);
    SetShaderValue(s->shader, s->loc_spec,      &s->specular,      SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->shader, s->loc_rough,     &s->roughness,     SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->shader, s->loc_cloth,     &cloth,            SHADER_UNIFORM_FLOAT);

    rlDisableBackfaceCulling();
    DrawMesh(s->cloth_mesh, s->cloth_mat, MatrixIdentity());
    rlEnableBackfaceCulling();

    if (s->wireframe)
        for (int i = 0; i < s->spring_count; i++) {
            if (s->springs[i].type != SPRING_STRUCTURAL || !s->springs[i].alive) continue;
            DrawLine3D(s->particles[s->springs[i].a].pos,
                       s->particles[s->springs[i].b].pos,
                       (Color){ 255, 255, 255, 90 });
        }
}

static void draw_collider(SimState *s)
{
    float not_cloth = 0.0f, spec = 0.35f, rough = 0.55f;
    SetShaderValue(s->shader, s->loc_cloth, &not_cloth, SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->shader, s->loc_spec,  &spec,      SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->shader, s->loc_rough, &rough,     SHADER_UNIFORM_FLOAT);

    switch (s->collider) {
    case COLLIDER_SPHERE: {
        float r = sphere_radius(s);
        Vector3 c = sphere_center(s);
        DrawMesh(s->sphere_mesh, s->collider_mat,
                 MatrixMultiply(MatrixScale(r, r, r), MatrixTranslate(c.x, c.y, c.z)));
        break;
    }
    case COLLIDER_CUBE: {
        Vector3 h = cube_half(s), c = cube_center(s);
        DrawMesh(s->cube_mesh, s->collider_mat,
                 MatrixMultiply(MatrixScale(h.x*2, h.y*2, h.z*2),
                                MatrixTranslate(c.x, c.y, c.z)));
        break;
    }
    case COLLIDER_MESH:
        s->mesh_collider.model.materials[0].shader = s->shader;
        DrawModel(s->mesh_collider.model, Vector3Zero(), 1.0f, (Color){ 90, 140, 220, 255 });
        break;
    default: break;
    }

    if (s->ball_active) {
        Color keep = s->collider_mat.maps[MATERIAL_MAP_DIFFUSE].color;
        s->collider_mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 235, 165, 70, 255 };
        DrawMesh(s->sphere_mesh, s->collider_mat,
                 MatrixMultiply(MatrixScale(BALL_RADIUS, BALL_RADIUS, BALL_RADIUS),
                                MatrixTranslate(s->ball_pos.x, s->ball_pos.y, s->ball_pos.z)));
        s->collider_mat.maps[MATERIAL_MAP_DIFFUSE].color = keep;
    }
}

void draw_scene(SimState *s, Camera3D camera, float cam_scale)
{
    DrawGrid(20, 0.5f * cam_scale);
    draw_collider(s);
    draw_cloth(s, camera);
}
