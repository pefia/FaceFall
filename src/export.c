#include "facefall.h"

void detect_ffmpeg(SimState *s)
{
    for (int i = 0; i < NUM_FFMPEG_CANDIDATES; i++) {
        char probe[512];
        snprintf(probe, sizeof(probe), "\"%s\" -version > %s 2>&1",
                 FFMPEG_CANDIDATES[i], DEVNULL);
        if (system(probe) == 0) {
            snprintf(s->ffmpeg_path, sizeof(s->ffmpeg_path), "%s", FFMPEG_CANDIDATES[i]);
            s->ffmpeg_ok = true;
            TraceLog(LOG_INFO, "FACEFALL: ffmpeg found: %s", s->ffmpeg_path);
            return;
        }
    }
    s->ffmpeg_ok = false;
    TraceLog(LOG_WARNING, "FACEFALL: ffmpeg not found — export disabled");
}

void begin_render(SimState *s)
{
    if (!s->ffmpeg_ok) {
        set_status(s, "ffmpeg not found - export unavailable");
        return;
    }

    int ew = EXPORT_RES[s->export_res].w;
    int eh = EXPORT_RES[s->export_res].h;

    s->export_rt = LoadRenderTexture(ew, eh);
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
             "\"\"%s\" -y -loglevel error -f rawvideo -pix_fmt rgba -s %dx%d "
             "-framerate %d -i - -c:v libx264 -preset medium -crf 18 "
             "-pix_fmt yuv420p \"%s\"\"",
             s->ffmpeg_path, ew, eh, RENDER_FPS, OUTPUT_FILENAME);

    s->ffmpeg_pipe = popen_wrap(cmd, POPEN_WRITE_MODE);
    if (s->ffmpeg_pipe == NULL) {
        UnloadRenderTexture(s->export_rt);
        s->export_rt = (RenderTexture2D){ 0 };
        set_status(s, "popen() failed - could not start ffmpeg");
        return;
    }

    build_cloth(s, s->grid_n);
    s->render_frame = 0;
    s->mode = MODE_RENDER;
    s->paused = false;
    set_status(s, TextFormat("Rendering %s... (V or ESC cancels)", EXPORT_RES[s->export_res].name));
}

void end_render(SimState *s, bool completed)
{
    if (s->ffmpeg_pipe != NULL) {
        pclose_wrap(s->ffmpeg_pipe);
        s->ffmpeg_pipe = NULL;
    }
    if (s->export_rt.id != 0) {
        UnloadRenderTexture(s->export_rt);
        s->export_rt = (RenderTexture2D){ 0 };
    }
    s->mode = MODE_SETTINGS;
    set_status(s, completed ? "Saved " OUTPUT_FILENAME : "Export cancelled");
}

bool pipe_frame(SimState *s, int w, int h)
{
    unsigned char *pixels = rlReadScreenPixels(w, h);
    size_t frame_bytes = (size_t)w * h * 4;
    size_t written = fwrite(pixels, 1, frame_bytes, s->ffmpeg_pipe);
    RL_FREE(pixels);
    if (written != frame_bytes) {
        TraceLog(LOG_ERROR, "FACEFALL: ffmpeg pipe broke after %d frames", s->render_frame);
        return false;
    }
    return true;
}
