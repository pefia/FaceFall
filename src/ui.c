#include "facefall.h"

#define SETTINGS_ROWS 18

void draw_hud(SimState *s, bool free_cam)
{
    const char *collider_names[] = { "SPHERE", "CUBE",
        s->mesh_collider.from_file ? "MESH (assets/model.obj)" : "MESH (generated torus)" };

    DrawRectangle(8, 8, 380, s->show_help ? 240 : 94, Fade(BLACK, 0.55f));
    DrawText("FaceFall - cloth simulator", 16, 14, 20, RAYWHITE);
    DrawText(TextFormat("cloth %dx%d (%d faces, %.1f units)   collider: %s x%.1f",
                        s->grid_n, s->grid_n, (s->grid_n - 1) * (s->grid_n - 1), s->cloth_size,
                        collider_names[s->collider], s->collider_scale),
             16, 40, 10, RAYWHITE);
    DrawText(TextFormat("%s  k=%.0f  bend=%.2f  rest=%.2f  wind=%.1f  %s%s%s",
                        FABRICS[s->fabric].name, s->stiffness, s->bend, s->restitution,
                        s->wind_power, s->self_collide ? "" : "NO-SELFCOL ",
                        s->paused ? "PAUSED " : "", s->pin_top_edge ? "PINNED" : ""),
             16, 56, 10, RAYWHITE);
    DrawText(TextFormat("speed %.3gx   tear %s   %s%s",
                        s->time_scale,
                        s->tear_ratio > 0.0f ? TextFormat("%.2f", s->tear_ratio) : "off",
                        s->heatmap ? "HEATMAP " : "",
                        s->ball_active ? "BALL" : ""),
             16, 72, 10, RAYWHITE);

    if (s->show_help) {
        int y = 92;
        const char *lines[] = {
            "R       reset cloth",
            "SPACE   pause      W  wireframe",
            "N       step one frame (paused)",
            ", / .   slow / fast motion",
            "T       tension heatmap",
            "LMB     drag cloth (wheel reels)",
            "RMB     throw ball",
            "F       free camera (WASD+mouse)",
            "V       export video (ffmpeg)",
            "ESC     back    H hide help",
        };
        for (int i = 0; i < (int)(sizeof(lines) / sizeof(lines[0])); i++, y += 15)
            DrawText(lines[i], 16, y, 10, (Color){ 200, 200, 200, 255 });
    }

    if (!s->ffmpeg_ok)
        DrawText("ffmpeg NOT FOUND - export disabled", 16, SCREEN_H - 24, 10,
                 (Color){ 255, 120, 100, 255 });

    if (free_cam)
        DrawText("FREE CAM: WASD + mouse (F to release)", SCREEN_W - 260, 10, 10, YELLOW);

    if (s->mode == MODE_RENDER) {
        float t = (float)s->render_frame / RENDER_TOTAL_FRAMES;
        DrawRectangle(0, SCREEN_H - 8, (int)(SCREEN_W * t), 8, RED);
        DrawText(TextFormat("RENDERING %d / %d", s->render_frame, RENDER_TOTAL_FRAMES),
                 SCREEN_W - 200, SCREEN_H - 30, 20, RED);
    }

    DrawFPS(SCREEN_W - 90, SCREEN_H - 60);
}

void draw_settings(SimState *s, int row)
{
    const char *collider_names[] = { "sphere", "cube",
        s->mesh_collider.from_file ? "mesh (assets/model.obj)" : "mesh (torus)" };
    const char *labels[SETTINGS_ROWS] = {
        "cloth resolution", "cloth size", "collider", "collider size",
        "fabric preset", "stiffness", "bend", "damping", "friction",
        "restitution", "self collision",
        "gravity", "wind", "specular", "roughness",
        "export res", "pin top edge", "tearing"
    };

    const int w = 500, h = 517;
    const int px = SCREEN_W / 2 - w / 2, py = SCREEN_H / 2 - h / 2;

    DrawRectangle(px, py, w, h, Fade(BLACK, 0.82f));
    DrawRectangleLines(px, py, w, h, Fade(RAYWHITE, 0.35f));
    DrawText("FACEFALL - cloth simulator", px + 20, py + 14, 20, RAYWHITE);

    int y = py + 46;
    for (int i = 0; i < SETTINGS_ROWS; i++, y += 23) {
        const char *val =
            (i == 0)  ? TextFormat("%d x %d", s->grid_n, s->grid_n) :
            (i == 1)  ? TextFormat("%.1f units", s->cloth_size) :
            (i == 2)  ? collider_names[s->collider] :
            (i == 3)  ? TextFormat("x%.1f", s->collider_scale) :
            (i == 4)  ? FABRICS[s->fabric].name :
            (i == 5)  ? TextFormat("%.0f", s->stiffness) :
            (i == 6)  ? TextFormat("%.2f", s->bend) :
            (i == 7)  ? TextFormat("%.1f", s->damping) :
            (i == 8)  ? TextFormat("%.2f", s->friction) :
            (i == 9)  ? TextFormat("%.2f", s->restitution) :
            (i == 10) ? (s->self_collide ? "on" : "off") :
            (i == 11) ? TextFormat("%.1f", s->gravity) :
            (i == 12) ? TextFormat("%.1f", s->wind_power) :
            (i == 13) ? TextFormat("%.2f", s->specular) :
            (i == 14) ? TextFormat("%.2f", s->roughness) :
            (i == 15) ? EXPORT_RES[s->export_res].name :
            (i == 16) ? (s->pin_top_edge ? "yes" : "no") :
                        (s->tear_ratio > 0.0f ? TextFormat("yank > %.2f x", s->tear_ratio) : "off");
        Color c = (i == row) ? YELLOW : (Color){ 205, 205, 205, 255 };
        if (i == row) DrawText(">", px + 20, y, 16, c);
        DrawText(labels[i], px + 40, y, 16, c);
        DrawText(val, px + 300, y, 16, c);
    }

    DrawText("UP/DOWN select    LEFT/RIGHT change", px + 20, y + 6, 10,
             (Color){ 170, 170, 170, 255 });
    DrawText("ENTER render + save mp4    P live preview    ESC quit",
             px + 20, y + 20, 10, (Color){ 170, 170, 170, 255 });
    if (!s->ffmpeg_ok)
        DrawText("ffmpeg NOT FOUND - render disabled", px + 20, y + 34, 10,
                 (Color){ 255, 120, 100, 255 });
}

void handle_input_settings(SimState *s, int *row, int *res_step, bool *quit)
{
    if (IsKeyPressed(KEY_DOWN)) *row = (*row + 1) % SETTINGS_ROWS;
    if (IsKeyPressed(KEY_UP))   *row = (*row + SETTINGS_ROWS - 1) % SETTINGS_ROWS;

    int dir = (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT)) ? 1
            : (IsKeyPressed(KEY_LEFT)  || IsKeyPressedRepeat(KEY_LEFT))  ? -1 : 0;
    if (dir != 0) switch (*row) {
        case 0:
            *res_step += dir;
            if (*res_step < 0) *res_step = 0;
            if (*res_step > NUM_RESOLUTION_STEPS - 1) *res_step = NUM_RESOLUTION_STEPS - 1;
            build_cloth(s, RESOLUTION_STEPS[*res_step]);
            break;
        case 1:
            s->cloth_size = Clamp(s->cloth_size + dir * 0.5f, MIN_CLOTH_SIZE, MAX_CLOTH_SIZE);
            build_cloth(s, s->grid_n);
            break;
        case 2:
            s->collider = (ColliderKind)(((int)s->collider + dir + COLLIDER_COUNT) % COLLIDER_COUNT);
            build_cloth(s, s->grid_n);
            break;
        case 3:
            s->collider_scale = Clamp(s->collider_scale + dir * 0.1f, 0.3f, 3.0f);
            bake_mesh_collider(s);
            build_cloth(s, s->grid_n);
            break;
        case 4:  apply_fabric(s, (s->fabric + dir + NUM_FABRICS) % NUM_FABRICS);           break;
        case 5:  s->stiffness   = Clamp(s->stiffness   + dir * 500.0f, 300.0f, 20000.0f);  break;
        case 6:  s->bend        = Clamp(s->bend        + dir * 0.05f,  0.0f,    1.0f);      break;
        case 7:  s->damping     = Clamp(s->damping     + dir * 0.2f,   0.0f,   10.0f);      break;
        case 8:  s->friction    = Clamp(s->friction    + dir * 0.05f,  0.0f,    1.0f);      break;
        case 9:  s->restitution = Clamp(s->restitution + dir * 0.05f,  0.0f,    1.0f);      break;
        case 10: s->self_collide = !s->self_collide;                                        break;
        case 11: s->gravity     = Clamp(s->gravity     + dir * 0.5f,   0.0f,   30.0f);      break;
        case 12: s->wind_power  = Clamp(s->wind_power  + dir * 0.5f,   0.0f,   12.0f);      break;
        case 13: s->specular    = Clamp(s->specular    + dir * 0.05f,  0.0f,    1.0f);      break;
        case 14: s->roughness   = Clamp(s->roughness   + dir * 0.05f,  0.0f,    1.0f);      break;
        case 15: s->export_res  = (int)Clamp((float)(s->export_res + dir), 0.0f, (float)(NUM_EXPORT_RES - 1)); break;
        case 16: s->pin_top_edge = !s->pin_top_edge; build_cloth(s, s->grid_n);             break;
        case 17:
            s->tear_ratio = (s->tear_ratio <= 0.0f && dir > 0) ? 1.30f
                                                               : s->tear_ratio + dir * 0.05f;
            if (s->tear_ratio < 1.10f) s->tear_ratio = 0.0f;
            if (s->tear_ratio > 2.00f) s->tear_ratio = 2.0f;
            break;
    }

    if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) begin_render(s);
    if (IsKeyPressed(KEY_P)) { build_cloth(s, s->grid_n); s->paused = false; s->mode = MODE_PREVIEW; }
    if (IsKeyPressed(KEY_ESCAPE)) *quit = true;
}

void handle_input_preview(SimState *s, bool *free_cam)
{
    if (IsKeyPressed(KEY_R)) build_cloth(s, s->grid_n);
    if (IsKeyPressed(KEY_W) && !*free_cam) s->wireframe = !s->wireframe;
    if (IsKeyPressed(KEY_SPACE)) s->paused = !s->paused;
    if (IsKeyPressed(KEY_H)) s->show_help = !s->show_help;
    if (IsKeyPressed(KEY_T)) s->heatmap = !s->heatmap;
    if (IsKeyPressed(KEY_COMMA))  s->time_scale = fmaxf(0.125f, s->time_scale * 0.5f);
    if (IsKeyPressed(KEY_PERIOD)) s->time_scale = fminf(2.0f, s->time_scale * 2.0f);
    if (IsKeyPressed(KEY_N) && s->paused) s->step_once = true;
    if (IsKeyPressed(KEY_F)) {
        *free_cam = !*free_cam;
        if (*free_cam) DisableCursor(); else EnableCursor();
    }
    if (IsKeyPressed(KEY_V)) begin_render(s);
    if (IsKeyPressed(KEY_ESCAPE)) {
        if (*free_cam) { *free_cam = false; EnableCursor(); }
        release_grab(s);
        s->mode = MODE_SETTINGS;
    }
}

void handle_mouse_preview(SimState *s, Camera3D camera, bool free_cam, float cam_scale)
{
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        Ray ray;
        if (free_cam) {
            ray.position = camera.position;
            ray.direction = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
        } else {
            ray = GetScreenToWorldRay(GetMousePosition(), camera);
        }
        s->ball_pos = Vector3Add(ray.position, ray.direction);
        s->ball_vel = Vector3Scale(ray.direction, BALL_SPEED);
        s->ball_active = true;
    }

    if (free_cam) {
        release_grab(s);
        return;
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
        int n2 = s->grid_n * s->grid_n;
        float spacing = s->cloth_size / (float)(s->grid_n - 1);
        float max_r = fmaxf(0.15f * cam_scale, 0.8f * spacing);
        float best = max_r * max_r;
        int best_i = -1;
        float best_t = 0.0f;
        for (int i = 0; i < n2; i++) {
            Vector3 to = Vector3Subtract(s->particles[i].pos, ray.position);
            float t = Vector3DotProduct(to, ray.direction);
            if (t < 0.3f) continue;
            float dsq = Vector3LengthSqr(Vector3Subtract(to, Vector3Scale(ray.direction, t)));
            if (dsq < best) { best = dsq; best_i = i; best_t = t; }
        }
        if (best_i >= 0) {
            s->grab_idx = best_i;
            s->grab_dist = best_t;
            s->particles[best_i].inv_mass = 0.0f;
        }
    }

    if (s->grab_idx >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
        s->grab_dist = Clamp(s->grab_dist + GetMouseWheelMove() * 0.6f, 0.5f, 60.0f);
        Particle *p = &s->particles[s->grab_idx];
        Vector3 target = Vector3Add(ray.position, Vector3Scale(ray.direction, s->grab_dist));
        Vector3 v = Vector3Scale(Vector3Subtract(target, p->pos), (float)PREVIEW_FPS);
        float speed = Vector3Length(v);
        if (speed > GRAB_MAX_SPEED) v = Vector3Scale(v, GRAB_MAX_SPEED / speed);
        p->vel = v;
        p->prev_pos = p->pos;
        p->pos = target;
    } else if (s->grab_idx >= 0) {
        release_grab(s);
    }
}

void handle_input_render(SimState *s)
{
    if (IsKeyPressed(KEY_V) || IsKeyPressed(KEY_ESCAPE)) end_render(s, false);
}
