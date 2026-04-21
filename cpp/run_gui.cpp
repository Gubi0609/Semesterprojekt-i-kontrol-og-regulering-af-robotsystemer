// ═══════════════════════════════════════════════════════════════════
// run_gui.cpp — Real-time GUI for Qube-Servo 3 inverted pendulum
// ═══════════════════════════════════════════════════════════════════
//
// Dear ImGui + ImPlot + OpenGL3 + GLFW frontend with 3D visualization.
// Control loop runs in a background thread at 1 kHz.
// GUI renders at ~60fps showing real-time 3D model, plots, and controls.
//
// Build: make run_gui
// Run:   ./run_gui
//
// Requires: libglfw3-dev libglew-dev (sudo apt install libglfw3-dev libglew-dev)
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"
#include "controllers.h"
#include "plant.h"
#include "render3d.h"

// ImGui + ImPlot (vendored source)
#include "vendor/imgui/imgui.h"
#include "vendor/imgui/backends/imgui_impl_glfw.h"
#include "vendor/imgui/backends/imgui_impl_opengl3.h"
#include "vendor/implot/implot.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <cstdio>
#include <cmath>
#include <cstring>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <chrono>

// Conditionally include HIL for hardware mode
#ifdef USE_HARDWARE
#include <hil.h>
#endif

// ─── Rolling buffer for real-time plots ────────────────────────────
struct RollingBuffer {
    std::vector<float> data;
    int max_size;
    int offset;
    int count;

    RollingBuffer(int max_sz = 30000)
        : data(max_sz, 0.0f), max_size(max_sz), offset(0), count(0) {}

    void push(float v) {
        data[offset] = v;
        offset = (offset + 1) % max_size;
        if (count < max_size) count++;
    }

    void clear() { offset = 0; count = 0; }

    float operator[](int i) const {
        int idx = (offset - count + i + max_size) % max_size;
        return data[idx];
    }
};

// ─── Shared state between control thread and GUI ───────────────────
struct SharedData {
    std::mutex mtx;

    double t = 0, theta = 0, alpha = 0, theta_dot = 0, alpha_dot = 0;
    double voltage = 0, energy = 0;
    int mode = 0;
    bool active = false;

    RollingBuffer hist_t, hist_theta, hist_alpha;
    RollingBuffer hist_theta_dot, hist_alpha_dot;
    RollingBuffer hist_voltage, hist_energy;

    float mu = 5.0f, catch_angle = 20.0f, vel_deadzone = 0.1f, initial_kick = 0.8f;
    // Parallel PID gains
    float alpha_Kp = 20.0f, alpha_Ki = 0.3f, alpha_Kd = 2.0f;
    float theta_Kp = 2.0f,  theta_Ki = 0.0f, theta_Kd = 1.0f;
    float voltage_limit = 6.0f;

    SharedData() : hist_t(30000), hist_theta(30000), hist_alpha(30000),
                   hist_theta_dot(30000), hist_alpha_dot(30000),
                   hist_voltage(30000), hist_energy(30000) {}

    void clear_history() {
        hist_t.clear(); hist_theta.clear(); hist_alpha.clear();
        hist_theta_dot.clear(); hist_alpha_dot.clear();
        hist_voltage.clear(); hist_energy.clear();
    }
};

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_control_active{false};
static std::atomic<bool> g_reset_request{false};

// ─── Simulation control thread ─────────────────────────────────────
void sim_control_thread(SharedData& sd) {
    QubeParams params;
    double dt = 0.001;
    int substeps = 10;

    BalanceController balance(dt);
    SwingUpController swing_up(params, dt);

    State x = {0.0, M_PI, 0.0, 0.0};
    enum Mode { SWING_UP, BALANCE } mode = SWING_UP;

    auto t_start = std::chrono::steady_clock::now();
    auto t_next  = t_start;

    while (g_running.load()) {
        if (!g_control_active.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            t_start = std::chrono::steady_clock::now();
            t_next  = t_start;
            x = {0.0, M_PI, 0.0, 0.0};
            mode = SWING_UP;
            balance.reset();
            swing_up.kick_timer = 0.0;
            continue;
        }

        if (g_reset_request.load()) {
            g_reset_request.store(false);
            x = {0.0, M_PI, 0.0, 0.0};
            mode = SWING_UP;
            balance.reset();
            swing_up.kick_timer = 0.0;
            t_start = std::chrono::steady_clock::now();
            t_next  = t_start;
        }

        double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();

        {
            std::lock_guard<std::mutex> lk(sd.mtx);
            swing_up.mu           = sd.mu;
            swing_up.catch_angle  = sd.catch_angle * M_PI / 180.0;
            swing_up.vel_deadzone = sd.vel_deadzone;
            swing_up.initial_kick = sd.initial_kick;
            balance.alpha_pid.Kp = sd.alpha_Kp;
            balance.alpha_pid.Ki = sd.alpha_Ki;
            balance.alpha_Kd = sd.alpha_Kd;
            balance.theta_pid.Kp = sd.theta_Kp;
            balance.theta_pid.Ki = sd.theta_Ki;
            balance.theta_Kd = sd.theta_Kd;
            balance.voltage_limit = sd.voltage_limit;
        }

        QubeState s = {x[0], x[1], x[2], x[3]};
        double voltage = 0.0;

        if (mode == SWING_UP) {
            voltage = swing_up.compute(s);
            if (swing_up.should_catch(s.alpha)) { mode = BALANCE; balance.reset(); }
        } else {
            voltage = balance.compute(s);
            if (fabs(s.alpha) > 30.0 * M_PI / 180.0) { mode = SWING_UP; balance.reset(); }
        }

        double sub_dt = dt / substeps;
        for (int j = 0; j < substeps; j++)
            x = rk4_step(x, voltage, sub_dt, params);
        x[0] = wrap_angle(x[0]);
        x[1] = wrap_angle(x[1]);

        double hLp = params.half_Lp();
        double E = 0.5 * params.Jp * x[3] * x[3]
                 - params.mp * params.g * hLp * (cos(x[1]) - 1.0);

        {
            std::lock_guard<std::mutex> lk(sd.mtx);
            sd.t = t; sd.theta = x[0]; sd.alpha = x[1];
            sd.theta_dot = x[2]; sd.alpha_dot = x[3];
            sd.voltage = voltage; sd.energy = E;
            sd.mode = (mode == BALANCE) ? 1 : 0;
            sd.active = true;

            sd.hist_t.push((float)t);
            sd.hist_theta.push((float)(x[0] * 180.0 / M_PI));
            sd.hist_alpha.push((float)(x[1] * 180.0 / M_PI));
            sd.hist_theta_dot.push((float)x[2]);
            sd.hist_alpha_dot.push((float)x[3]);
            sd.hist_voltage.push((float)voltage);
            sd.hist_energy.push((float)E);
        }

        t_next += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(t_next);
    }
}

#ifdef USE_HARDWARE
void hw_control_thread(SharedData& sd) {
    static constexpr double PEND_OFFSET_RAD = M_PI;
    static constexpr t_uint32 TACHO_CHANNELS[] = {14000, 14001};
    double dt = 0.001;
    QubeParams params;
    BalanceController balance(dt);
    SwingUpController swing_up(params, dt);
    enum Mode { SWING_UP, BALANCE } mode = SWING_UP;

    t_card card;
    if (hil_open("qube_servo3_usb", "0", &card) < 0) {
        fprintf(stderr, "Failed to open Qube-Servo 3\n"); return;
    }
    const t_uint32 enc_ch[] = {0, 1}, aout_ch[] = {0}, dout_ch[] = {0};
    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);
    t_int32 zero_counts[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero_counts);

    t_int32 enc_buf[2]; t_double tacho_buf[2]; t_double aout_buf[1] = {0};
    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);
    long dt_ns = (long)(dt * 1e9);
    auto t_start = std::chrono::steady_clock::now();

    while (g_running.load()) {
        if (!g_control_active.load()) {
            aout_buf[0] = 0; hil_write_analog(card, aout_ch, 1, aout_buf);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            clock_gettime(CLOCK_MONOTONIC, &t_next);
            t_start = std::chrono::steady_clock::now();
            mode = SWING_UP; balance.reset(); swing_up.kick_timer = 0;
            hil_set_encoder_counts(card, enc_ch, 2, zero_counts);
            continue;
        }
        if (g_reset_request.load()) {
            g_reset_request.store(false);
            mode = SWING_UP; balance.reset(); swing_up.kick_timer = 0;
            hil_set_encoder_counts(card, enc_ch, 2, zero_counts);
            t_start = std::chrono::steady_clock::now();
            clock_gettime(CLOCK_MONOTONIC, &t_next);
        }

        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        { std::lock_guard<std::mutex> lk(sd.mtx);
          swing_up.mu = sd.mu; swing_up.catch_angle = sd.catch_angle * M_PI/180;
          swing_up.vel_deadzone = sd.vel_deadzone; swing_up.initial_kick = sd.initial_kick;
          balance.alpha_pid.Kp = sd.alpha_Kp; balance.alpha_pid.Ki = sd.alpha_Ki;
          balance.alpha_Kd = sd.alpha_Kd; balance.theta_pid.Kp = sd.theta_Kp;
          balance.theta_pid.Ki = sd.theta_Ki; balance.theta_Kd = sd.theta_Kd;
          balance.voltage_limit = sd.voltage_limit; }

        hil_read_encoder(card, enc_ch, 2, enc_buf);
        hil_read_other(card, TACHO_CHANNELS, 2, tacho_buf);
        double theta = enc_buf[0] * COUNTS_TO_RAD;
        double alpha = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));
        double theta_dot = tacho_buf[0] * COUNTS_TO_RAD;
        double alpha_dot = -tacho_buf[1] * COUNTS_TO_RAD;

        QubeState s = {theta, alpha, theta_dot, alpha_dot};
        double voltage = 0;
        if (mode == SWING_UP) {
            voltage = swing_up.compute(s);
            if (swing_up.should_catch(s.alpha)) { mode = BALANCE; balance.reset(); }
        } else {
            voltage = balance.compute(s);
            if (fabs(s.alpha) > 30.0 * M_PI/180) { mode = SWING_UP; balance.reset(); }
        }
        aout_buf[0] = voltage; hil_write_analog(card, aout_ch, 1, aout_buf);

        double hLp = params.half_Lp();
        double E = 0.5*params.Jp*alpha_dot*alpha_dot - params.mp*params.g*hLp*(cos(alpha)-1);
        { std::lock_guard<std::mutex> lk(sd.mtx);
          sd.t=t; sd.theta=theta; sd.alpha=alpha; sd.theta_dot=theta_dot; sd.alpha_dot=alpha_dot;
          sd.voltage=voltage; sd.energy=E; sd.mode=(mode==BALANCE)?1:0; sd.active=true;
          sd.hist_t.push((float)t); sd.hist_theta.push((float)(theta*180/M_PI));
          sd.hist_alpha.push((float)(alpha*180/M_PI)); sd.hist_theta_dot.push((float)theta_dot);
          sd.hist_alpha_dot.push((float)alpha_dot); sd.hist_voltage.push((float)voltage);
          sd.hist_energy.push((float)E); }

        t_next.tv_nsec += dt_ns;
        while (t_next.tv_nsec >= 1000000000L) { t_next.tv_sec++; t_next.tv_nsec -= 1000000000L; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t_next, nullptr);
    }
    aout_buf[0] = 0; hil_write_analog(card, aout_ch, 1, aout_buf);
    t_boolean disable[] = {0}; hil_write_digital(card, dout_ch, 1, disable);
    hil_close(card);
}
#endif

// ─── ImPlot helper ─────────────────────────────────────────────────
struct PlotData { const RollingBuffer* t_buf; const RollingBuffer* y_buf; };
static ImPlotPoint RollingGetter(int idx, void* ud) {
    PlotData* pd = (PlotData*)ud;
    return ImPlotPoint((*pd->t_buf)[idx], (*pd->y_buf)[idx]);
}

// ═══════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    bool hardware_mode = false;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--hardware") || !strcmp(argv[i], "-hw"))
            hardware_mode = true;

#ifndef USE_HARDWARE
    if (hardware_mode) {
        fprintf(stderr, "Built without hardware support. Rebuild with: make run_gui_hw\n");
        return 1;
    }
#endif

    if (!glfwInit()) { fprintf(stderr, "GLFW init failed\n"); return 1; }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1600, 950,
        "Qube-Servo 3 — Inverted Pendulum Controller", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { fprintf(stderr, "GLEW init failed\n"); return 1; }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.FrameRounding  = 4.0f;
    style.GrabRounding   = 4.0f;
    style.WindowPadding  = ImVec2(10, 10);
    style.FramePadding   = ImVec2(6, 4);
    style.ItemSpacing    = ImVec2(8, 6);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]         = ImVec4(0.10f, 0.10f, 0.12f, 1.0f);
    colors[ImGuiCol_TitleBg]          = ImVec4(0.08f, 0.08f, 0.10f, 1.0f);
    colors[ImGuiCol_TitleBgActive]    = ImVec4(0.15f, 0.15f, 0.20f, 1.0f);
    colors[ImGuiCol_FrameBg]          = ImVec4(0.16f, 0.16f, 0.20f, 1.0f);
    colors[ImGuiCol_FrameBgHovered]   = ImVec4(0.22f, 0.22f, 0.28f, 1.0f);
    colors[ImGuiCol_Button]           = ImVec4(0.20f, 0.40f, 0.70f, 1.0f);
    colors[ImGuiCol_ButtonHovered]    = ImVec4(0.25f, 0.50f, 0.80f, 1.0f);
    colors[ImGuiCol_ButtonActive]     = ImVec4(0.15f, 0.35f, 0.65f, 1.0f);
    colors[ImGuiCol_SliderGrab]       = ImVec4(0.30f, 0.55f, 0.85f, 1.0f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.35f, 0.60f, 0.90f, 1.0f);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    SharedData sd;
    std::thread control_thread;
#ifdef USE_HARDWARE
    if (hardware_mode)
        control_thread = std::thread(hw_control_thread, std::ref(sd));
    else
#endif
    control_thread = std::thread(sim_control_thread, std::ref(sd));

    QubeRenderer renderer;
    float plot_window = 10.0f;

    // Camera drag state
    bool dragging_3d = false;
    double drag_start_x = 0, drag_start_y = 0;
    float drag_start_h = 0, drag_start_v = 0;

    // ── Main loop ──────────────────────────────────────────────────
    while (!glfwWindowShouldClose(window) && g_running.load()) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Snapshot state
        double cur_t, cur_theta, cur_alpha, cur_theta_dot, cur_alpha_dot;
        double cur_voltage, cur_energy;
        int cur_mode;
        {
            std::lock_guard<std::mutex> lk(sd.mtx);
            cur_t = sd.t; cur_theta = sd.theta; cur_alpha = sd.alpha;
            cur_theta_dot = sd.theta_dot; cur_alpha_dot = sd.alpha_dot;
            cur_voltage = sd.voltage; cur_energy = sd.energy;
            cur_mode = sd.mode;
        }

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        float dw = (float)display_w, dh = (float)display_h;

        // Layout: left column = 3D view (top) + controls (bottom)
        //         right column = plots
        float left_w  = dw * 0.38f;
        float right_w = dw - left_w;
        float view3d_h = left_w * 0.75f;  // 4:3-ish aspect
        float ctrl_h   = dh - view3d_h;

        // ════════════════════════════════════════════════════════════
        // 3D VISUALIZATION
        // ════════════════════════════════════════════════════════════
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(left_w, view3d_h), ImGuiCond_Always);
        ImGui::Begin("3D View", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

        ImVec2 avail = ImGui::GetContentRegionAvail();
        int vw = (int)avail.x, vh = (int)avail.y;
        if (vw > 0 && vh > 0) {
            GLuint tex = renderer.render(vw, vh, (float)cur_theta, (float)cur_alpha);
            // Flip V because OpenGL texture origin is bottom-left
            ImGui::Image((ImTextureID)(intptr_t)tex, avail, ImVec2(0, 1), ImVec2(1, 0));

            // Mouse drag to orbit camera (only when hovering the 3D image)
            if (ImGui::IsItemHovered()) {
                // Scroll to zoom
                float scroll = io.MouseWheel;
                if (scroll != 0) {
                    renderer.cam_dist -= scroll * 0.05f;
                    if (renderer.cam_dist < 0.15f) renderer.cam_dist = 0.15f;
                    if (renderer.cam_dist > 2.0f)  renderer.cam_dist = 2.0f;
                }

                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    dragging_3d = true;
                    drag_start_x = io.MousePos.x;
                    drag_start_y = io.MousePos.y;
                    drag_start_h = renderer.cam_angle_h;
                    drag_start_v = renderer.cam_angle_v;
                }
            }
            if (dragging_3d) {
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    float dx = (float)(io.MousePos.x - drag_start_x);
                    float dy = (float)(io.MousePos.y - drag_start_y);
                    renderer.cam_angle_h = drag_start_h - dx * 0.008f;
                    renderer.cam_angle_v = drag_start_v + dy * 0.008f;
                    // Clamp vertical to avoid flipping
                    if (renderer.cam_angle_v < -1.2f) renderer.cam_angle_v = -1.2f;
                    if (renderer.cam_angle_v >  1.4f) renderer.cam_angle_v = 1.4f;
                } else {
                    dragging_3d = false;
                }
            }
        }
        ImGui::End();

        // ════════════════════════════════════════════════════════════
        // CONTROL PANEL
        // ════════════════════════════════════════════════════════════
        ImGui::SetNextWindowPos(ImVec2(0, view3d_h), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(left_w, ctrl_h), ImGuiCond_Always);
        ImGui::Begin("Controls", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse);

        // Mode indicator
        {
            const char* mode_str = cur_mode == 1 ? "BALANCING" : "SWING-UP";
            ImVec4 mode_col = cur_mode == 1
                ? ImVec4(0.2f, 0.8f, 0.3f, 1.0f)
                : ImVec4(0.9f, 0.6f, 0.1f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, mode_col);
            ImGui::SetWindowFontScale(1.3f);
            ImGui::Text("%s", mode_str);
            ImGui::SetWindowFontScale(1.0f);
            ImGui::PopStyleColor();
            ImGui::SameLine(200);
            ImGui::TextDisabled("t = %.2f s", cur_t);
        }

        ImGui::Separator();

        // Start / Stop / Reset
        bool is_active = g_control_active.load();
        if (is_active) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
            if (ImGui::Button("Stop", ImVec2(90, 28))) g_control_active.store(false);
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.3f, 1.0f));
            if (ImGui::Button("Start", ImVec2(90, 28))) {
                { std::lock_guard<std::mutex> lk(sd.mtx); sd.clear_history(); }
                g_control_active.store(true);
            }
            ImGui::PopStyleColor();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(90, 28))) {
            g_reset_request.store(true);
            { std::lock_guard<std::mutex> lk(sd.mtx); sd.clear_history(); }
        }

        // State readout (compact)
        ImGui::Spacing();
        ImGui::Columns(4, "state", false);
        ImGui::TextColored(ImVec4(0.4f,0.7f,1,1), "theta"); ImGui::NextColumn();
        ImGui::Text("%.1f", cur_theta * 180/M_PI); ImGui::NextColumn();
        ImGui::TextColored(ImVec4(1,0.5f,0.3f,1), "alpha"); ImGui::NextColumn();
        ImGui::Text("%.1f", cur_alpha * 180/M_PI); ImGui::NextColumn();
        ImGui::TextColored(ImVec4(0.4f,0.7f,1,1), "theta_dot"); ImGui::NextColumn();
        ImGui::Text("%.0f", cur_theta_dot * 180/M_PI); ImGui::NextColumn();
        ImGui::TextColored(ImVec4(1,0.5f,0.3f,1), "alpha_dot"); ImGui::NextColumn();
        ImGui::Text("%.0f", cur_alpha_dot * 180/M_PI); ImGui::NextColumn();
        ImGui::TextColored(ImVec4(0.9f,0.9f,0.3f,1), "voltage"); ImGui::NextColumn();
        ImGui::Text("%.2f V", cur_voltage); ImGui::NextColumn();
        ImGui::TextColored(ImVec4(0.7f,0.4f,0.9f,1), "energy"); ImGui::NextColumn();
        ImGui::Text("%.5f", cur_energy); ImGui::NextColumn();
        ImGui::Columns(1);

        // Parameters in two side-by-side groups
        ImGui::Spacing(); ImGui::Separator();
        ImGui::Columns(2, "params", true);

        ImGui::Text("Swing-Up");
        {
            std::lock_guard<std::mutex> lk(sd.mtx);
            ImGui::SliderFloat("mu", &sd.mu, 1.0f, 10.0f, "%.1f V");
            ImGui::SliderFloat("catch", &sd.catch_angle, 5.0f, 45.0f, "%.0f deg");
            ImGui::SliderFloat("dz", &sd.vel_deadzone, 0.01f, 1.0f, "%.2f");
            ImGui::SliderFloat("kick", &sd.initial_kick, 0.0f, 2.0f, "%.1f V");
        }

        ImGui::NextColumn();
        ImGui::Text("Balance — Alpha PID");
        {
            std::lock_guard<std::mutex> lk(sd.mtx);
            ImGui::SliderFloat("aKp", &sd.alpha_Kp, 0.0f, 40.0f, "%.1f");
            ImGui::SliderFloat("aKi", &sd.alpha_Ki, 0.0f, 2.0f, "%.2f");
            ImGui::SliderFloat("aKd", &sd.alpha_Kd, 0.0f, 10.0f, "%.1f");
            ImGui::Spacing();
            ImGui::Text("Balance — Theta PID");
            ImGui::SliderFloat("tKp", &sd.theta_Kp, 0.0f, 10.0f, "%.1f");
            ImGui::SliderFloat("tKi", &sd.theta_Ki, 0.0f, 2.0f, "%.2f");
            ImGui::SliderFloat("tKd", &sd.theta_Kd, 0.0f, 5.0f, "%.1f");
            ImGui::SliderFloat("Vlim", &sd.voltage_limit, 1.0f, 10.0f, "%.1f V");
        }
        ImGui::Columns(1);

        ImGui::Spacing(); ImGui::Separator();
        ImGui::SliderFloat("Plot window (s)", &plot_window, 2.0f, 30.0f, "%.0f");

        ImGui::End();

        // ════════════════════════════════════════════════════════════
        // PLOTS (right side)
        // ════════════════════════════════════════════════════════════
        ImGui::SetNextWindowPos(ImVec2(left_w, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(right_w, dh), ImGuiCond_Always);
        ImGui::Begin("Plots", nullptr,
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse);

        float t_max = (float)cur_t;
        float t_min = t_max - plot_window;
        if (t_min < 0) t_min = 0;

        float plot_h = (ImGui::GetContentRegionAvail().y - 30) / 4.0f;

        std::lock_guard<std::mutex> lk(sd.mtx);
        int n = sd.hist_t.count;

        auto plot_line = [&](const char* label, const RollingBuffer& y_buf,
                            ImVec4 color, float weight = 1.5f) {
            PlotData pd = {&sd.hist_t, &y_buf};
            ImPlotSpec spec(ImPlotProp_LineColor, color, ImPlotProp_LineWeight, weight);
            ImPlot::PlotLineG(label, RollingGetter, &pd, n, spec);
        };

        auto ref_spec = [](ImVec4 color, float weight = 1.0f) {
            return ImPlotSpec(ImPlotProp_LineColor, color, ImPlotProp_LineWeight, weight);
        };

        // Angles
        if (ImPlot::BeginPlot("Angles", ImVec2(-1, plot_h))) {
            ImPlot::SetupAxes("", "deg");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -200, 200, ImGuiCond_Once);
            plot_line("theta", sd.hist_theta, ImVec4(0.4f,0.7f,1,1));
            plot_line("alpha", sd.hist_alpha, ImVec4(1,0.5f,0.3f,1));
            float cd = sd.catch_angle, ct[] = {t_min, t_max};
            float cy[] = {cd, cd}, cn[] = {-cd, -cd};
            ImPlot::PlotLine("##c+", ct, cy, 2, ref_spec(ImVec4(0.3f,0.8f,0.3f,0.4f)));
            ImPlot::PlotLine("##c-", ct, cn, 2, ref_spec(ImVec4(0.3f,0.8f,0.3f,0.4f)));
            ImPlot::EndPlot();
        }

        // Velocities
        if (ImPlot::BeginPlot("Velocities", ImVec2(-1, plot_h))) {
            ImPlot::SetupAxes("", "rad/s");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -20, 20, ImGuiCond_Once);
            plot_line("theta_dot", sd.hist_theta_dot, ImVec4(0.4f,0.7f,1,1));
            plot_line("alpha_dot", sd.hist_alpha_dot, ImVec4(1,0.5f,0.3f,1));
            ImPlot::EndPlot();
        }

        // Voltage
        if (ImPlot::BeginPlot("Voltage", ImVec2(-1, plot_h))) {
            ImPlot::SetupAxes("", "V");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10, ImGuiCond_Once);
            plot_line("V", sd.hist_voltage, ImVec4(0.9f,0.9f,0.3f,1));
            ImPlot::EndPlot();
        }

        // Energy
        if (ImPlot::BeginPlot("Energy", ImVec2(-1, plot_h))) {
            ImPlot::SetupAxes("time (s)", "J");
            ImPlot::SetupAxisLimits(ImAxis_X1, t_min, t_max, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -0.001, 0.001, ImGuiCond_Once);
            plot_line("E", sd.hist_energy, ImVec4(0.7f,0.4f,0.9f,1));
            float zt[] = {t_min, t_max}, zy[] = {0, 0};
            ImPlot::PlotLine("##0", zt, zy, 2, ref_spec(ImVec4(0.5f,0.5f,0.5f,0.4f)));
            ImPlot::EndPlot();
        }

        ImGui::End();

        // ── Render ─────────────────────────────────────────────────
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    g_running.store(false);
    g_control_active.store(false);
    if (control_thread.joinable()) control_thread.join();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    printf("GUI closed.\n");
    return 0;
}
