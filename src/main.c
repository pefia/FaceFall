#include "facefall.h"

static SimState state;

static void advance_sim(SimState *s)
{
    if (s->mode == MODE_SETTINGS) return;
    if (s->mode == MODE_RENDER) {
        if (!s->paused) step_frame(s);
        return;
    }
    if (s->paused) {
        if (s->step_once) { s->step_once = false; step_frame(s); }
        return;
    }
    s->step_accum += s->time_scale * (float)(RENDER_FPS / PREVIEW_FPS);
    while (s->step_accum >= 1.0f) {
        step_frame(s);
        s->step_accum -= 1.0f;
    }
}

static void frame_camera(const SimState *s, Camera3D *camera, bool free_cam, float *cam_scale)
{
    *cam_scale = fmaxf(1.0f, fmaxf(s->cloth_size / 4.0f, collider_top_y(s) / 2.2f));
    if (free_cam) return;
    camera->position = (Vector3){ 8.5f * *cam_scale, 6.0f * *cam_scale, 8.5f * *cam_scale };
    camera->target   = (Vector3){ 0.0f, 2.0f * *cam_scale, 0.0f };
}

static bool draw_window(SimState *s, Camera3D camera, float cam_scale, int menu_row, bool free_cam)
{
    const Color bg = { 24, 26, 34, 255 };
    bool pipe_ok = true;

    if (s->topo_dirty) rebuild_cloth_mesh(s);

    BeginDrawing();
        ClearBackground(bg);

        if (s->mode == MODE_RENDER) {
            int ew = EXPORT_RES[s->export_res].w, eh = EXPORT_RES[s->export_res].h;
            BeginTextureMode(s->export_rt);
                ClearBackground(bg);
                BeginMode3D(camera);
                    draw_scene(s, camera, cam_scale);
                EndMode3D();
                rlDrawRenderBatchActive();
                pipe_ok = pipe_frame(s, ew, eh);
            EndTextureMode();

            DrawTexturePro(s->export_rt.texture,
                (Rectangle){ 0, 0, (float)ew, -(float)eh },
                (Rectangle){ 0, 0, SCREEN_W, SCREEN_H },
                (Vector2){ 0, 0 }, 0.0f, WHITE);
        } else {
            BeginMode3D(camera);
                draw_scene(s, camera, cam_scale);
            EndMode3D();
        }

        if (s->mode == MODE_SETTINGS) draw_settings(s, menu_row);
        else                          draw_hud(s, free_cam);

        if (GetTime() < s->status_until)
            DrawText(s->status_msg, SCREEN_W / 2 - MeasureText(s->status_msg, 20) / 2,
                     SCREEN_H - 48, 20, YELLOW);
    EndDrawing();

    return pipe_ok;
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--selftest") == 0) return run_selftest();
    bool auto_render = (argc > 1 && strcmp(argv[1], "--render") == 0);

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_W, SCREEN_H, "FaceFall - cloth simulator");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);

    SimState *s = &state;
    physics_init(s);
    detect_ffmpeg(s);
    renderer_init(s);
    init_mesh_collider(s);
    build_cloth(s, GRID_SIZE);

    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 8.5f, 6.0f, 8.5f };
    camera.target     = (Vector3){ 0.0f, 2.0f, 0.0f };
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy       = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    bool free_cam = false;

    int res_step = 0;
    for (int i = 0; i < NUM_RESOLUTION_STEPS; i++)
        if (RESOLUTION_STEPS[i] == GRID_SIZE) res_step = i;

    int menu_row = 0;
    bool quit = false;

    if (auto_render) begin_render(s);

    while (!WindowShouldClose() && !quit) {
        switch (s->mode) {
        case MODE_SETTINGS: handle_input_settings(s, &menu_row, &res_step, &quit); break;
        case MODE_PREVIEW:  handle_input_preview(s, &free_cam);                    break;
        case MODE_RENDER:   handle_input_render(s);                                break;
        }
        if (s->mode == MODE_PREVIEW && free_cam) UpdateCamera(&camera, CAMERA_FREE);

        advance_sim(s);

        float cam_scale;
        frame_camera(s, &camera, free_cam, &cam_scale);

        if (s->mode == MODE_PREVIEW) handle_mouse_preview(s, camera, free_cam, cam_scale);

        bool pipe_ok = draw_window(s, camera, cam_scale, menu_row, free_cam);

        if (s->mode == MODE_RENDER) {
            if (!pipe_ok) {
                end_render(s, false);
                if (auto_render) break;
            } else if (++s->render_frame >= RENDER_TOTAL_FRAMES) {
                end_render(s, true);
                if (auto_render) break;
            }
        }
    }

    if (s->ffmpeg_pipe != NULL) end_render(s, false);
    renderer_cleanup(s);
    CloseWindow();
    return 0;
}

int run_selftest(void)
{
    SimState *s = &state;
    physics_init(s);
    s->pin_top_edge = false;
    s->wind_power   = 0.0f;
    s->collider     = COLLIDER_SPHERE;
    build_cloth(s, 24);

    const int   centre    = particle_idx(s->grid_n / 2, s->grid_n / 2, s->grid_n);
    const float contact_y = sphere_center(s).y + sphere_radius(s) + s->collision_shell + 0.02f;
    bool  touched = false;
    float touch_y = 0.0f, rebound_peak = -1e9f;

    for (int frame = 0; frame < 3 * RENDER_FPS; frame++) {
        step_frame(s);
        float y = s->particles[centre].pos.y;
        if (!touched) { if (y <= contact_y) { touched = true; touch_y = y; } }
        else if (y > rebound_peak) rebound_peak = y;
    }
    printf("[selftest] sphere: impact y=%.3f, later peak y=%.3f (+%.3f)\n",
           touch_y, rebound_peak, rebound_peak - touch_y);
    assert(touched && "cloth never reached the sphere");
    assert(rebound_peak - touch_y < 0.05f && "real cloth should not bounce");

    int n2 = s->grid_n * s->grid_n;
    float max_y = -1e9f, min_y = 1e9f;
    for (int i = 0; i < n2; i++) {
        Vector3 pos = s->particles[i].pos;
        assert(isfinite(pos.x) && isfinite(pos.y) && isfinite(pos.z) && "position non-finite");
        assert(fabsf(pos.x) < 100.0f && fabsf(pos.y) < 100.0f && fabsf(pos.z) < 100.0f && "cloth flung");
        if (pos.y > max_y) max_y = pos.y;
        if (pos.y < min_y) min_y = pos.y;
    }
    printf("[selftest] drape settled: y in [%.3f, %.3f]\n", min_y, max_y);
    assert(min_y >= -0.05f && "cloth fell through the ground");

    float worst = 0.0f;
    for (int i = 0; i < s->spring_count; i++) {
        Spring *sp = &s->springs[i];
        if (sp->type != SPRING_STRUCTURAL) continue;
        float ratio = Vector3Distance(s->particles[sp->a].pos, s->particles[sp->b].pos) / sp->rest_len;
        if (ratio > worst) worst = ratio;
    }
    printf("[selftest] worst structural stretch: %.3f (limit %.3f)\n", worst, MAX_STRETCH);
    assert(worst < MAX_STRETCH + 0.05f && "strain limiting failed");

    s->collider    = COLLIDER_CUBE;
    s->restitution = 0.6f;
    build_cloth(s, 24);
    const float cube_contact_y = cube_center(s).y + cube_half(s).y + s->collision_shell + 0.02f;
    touched = false;
    touch_y = 0.0f; rebound_peak = -1e9f;

    for (int frame = 0; frame < 2 * RENDER_FPS; frame++) {
        step_frame(s);
        float y = s->particles[centre].pos.y;
        if (!touched) { if (y <= cube_contact_y) { touched = true; touch_y = y; } }
        else if (y > rebound_peak) rebound_peak = y;
    }
    printf("[selftest] cube (e=0.6): impact y=%.3f, rebound peak y=%.3f (+%.3f)\n",
           touch_y, rebound_peak, rebound_peak - touch_y);
    assert(touched && "cloth never reached the cube");
    assert(rebound_peak > touch_y + 0.05f && "contact layer returned no energy");

    physics_init(s);
    s->pin_top_edge = true;
    s->wind_power = 0.0f;
    s->collider = COLLIDER_SPHERE;
    s->tear_ratio = 1.2f;
    build_cloth(s, 24);

    for (int frame = 0; frame < RENDER_FPS / 2; frame++) step_frame(s);
    int torn = 0;
    for (int i = 0; i < s->spring_count; i++) if (!s->springs[i].alive) torn++;
    printf("[selftest] tearing: %d springs torn while hanging (must be 0)\n", torn);
    assert(torn == 0 && "cloth tore itself under plain gravity");

    for (int i = 0; i < s->grid_n; i++) {
        int lo = particle_idx(i, s->grid_n - 1, s->grid_n);
        int up = particle_idx(i, s->grid_n - 2, s->grid_n);
        Vector3 dir = Vector3Subtract(s->particles[lo].pos, s->particles[up].pos);
        float len = Vector3Length(dir);
        if (len > 1e-6f)
            s->particles[lo].vel = Vector3Add(s->particles[up].vel, Vector3Scale(dir, 30.0f / len));
    }
    for (int frame = 0; frame < RENDER_FPS; frame++) step_frame(s);
    torn = 0;
    for (int i = 0; i < s->spring_count; i++) if (!s->springs[i].alive) torn++;

    n2 = s->grid_n * s->grid_n;
    min_y = 1e9f;
    for (int i = 0; i < n2; i++) {
        Vector3 pos = s->particles[i].pos;
        assert(isfinite(pos.x) && isfinite(pos.y) && isfinite(pos.z) && "position non-finite after tear");
        assert(fabsf(pos.x) < 100.0f && fabsf(pos.y) < 100.0f && fabsf(pos.z) < 100.0f && "cloth flung after tear");
        if (pos.y < min_y) min_y = pos.y;
    }
    printf("[selftest] tearing: yank tore %d of %d springs, lowest particle y=%.3f\n",
           torn, s->spring_count, min_y);
    assert(torn > 0 && "a 30 m/s yank should tear springs");
    assert(min_y < 0.5f && "torn strip never fell to the ground");
    assert(min_y >= -0.05f && "torn strip fell through the ground");

    physics_init(s);
    s->pin_top_edge = false;
    s->wind_power = 0.0f;
    s->collider = COLLIDER_SPHERE;
    build_cloth(s, 24);

    for (int frame = 0; frame < RENDER_FPS; frame++) step_frame(s);
    s->ball_pos = (Vector3){ 0.0f, collider_top_y(s) + 2.5f, 0.0f };
    s->ball_vel = (Vector3){ 0.0f, -6.0f, 0.0f };
    s->ball_active = true;

    float ball_min_y = 1e9f;
    for (int frame = 0; frame < 2 * RENDER_FPS; frame++) {
        step_frame(s);
        if (s->ball_pos.y < ball_min_y) ball_min_y = s->ball_pos.y;
    }
    printf("[selftest] ball: lowest y=%.3f, final pos (%.2f, %.2f, %.2f)\n",
           ball_min_y, s->ball_pos.x, s->ball_pos.y, s->ball_pos.z);
    assert(isfinite(s->ball_pos.x) && isfinite(s->ball_pos.y) && isfinite(s->ball_pos.z) && "ball non-finite");
    assert(ball_min_y < collider_top_y(s) + 1.0f && "ball never came down");
    assert(s->ball_pos.y >= GROUND_Y + BALL_RADIUS - 0.05f && "ball fell through the ground");
    assert(fabsf(s->ball_pos.x) < 30.0f && fabsf(s->ball_pos.z) < 30.0f && "ball flung out of the world");
    for (int i = 0; i < n2; i++) {
        Vector3 pos = s->particles[i].pos;
        assert(isfinite(pos.x) && isfinite(pos.y) && isfinite(pos.z) && "cloth non-finite after ball");
    }

    printf("[selftest] all checks passed\n");
    return 0;
}
