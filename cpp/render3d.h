#pragma once
// ═══════════════════════════════════════════════════════════════════
// render3d.h — 3D OpenGL visualization of the Qube-Servo 3
// ═══════════════════════════════════════════════════════════════════
//
// Renders a stylized model of the Qube: cube body, motor arm, and
// pendulum rod. Uses an offscreen FBO so the result can be displayed
// as a texture inside an ImGui window.
//
// All geometry is generated procedurally (no mesh files needed).
// Uses basic Phong shading with one directional light.
// ═══════════════════════════════════════════════════════════════════

#include <GL/glew.h>
#include <cmath>
#include <vector>
#include <cstdio>

// ─── Simple 3D math (no GLM dependency) ────────────────────────────

struct Vec3 { float x, y, z; };
struct Mat4 { float m[16]; };

static inline Mat4 mat4_identity() {
    Mat4 r = {};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

static inline Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
    Mat4 r = {};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[j * 4 + i] += a.m[k * 4 + i] * b.m[j * 4 + k];
    return r;
}

static inline Mat4 mat4_translate(float x, float y, float z) {
    Mat4 r = mat4_identity();
    r.m[12] = x; r.m[13] = y; r.m[14] = z;
    return r;
}

static inline Mat4 mat4_rotate_y(float rad) {
    Mat4 r = mat4_identity();
    float c = cosf(rad), s = sinf(rad);
    r.m[0] = c;  r.m[2] = s;
    r.m[8] = -s; r.m[10] = c;
    return r;
}

static inline Mat4 mat4_rotate_x(float rad) {
    Mat4 r = mat4_identity();
    float c = cosf(rad), s = sinf(rad);
    r.m[5] = c;  r.m[6] = s;
    r.m[9] = -s; r.m[10] = c;
    return r;
}

static inline Mat4 mat4_rotate_z(float rad) {
    Mat4 r = mat4_identity();
    float c = cosf(rad), s = sinf(rad);
    r.m[0] = c;  r.m[1] = s;
    r.m[4] = -s; r.m[5] = c;
    return r;
}

static inline Mat4 mat4_scale(float sx, float sy, float sz) {
    Mat4 r = mat4_identity();
    r.m[0] = sx; r.m[5] = sy; r.m[10] = sz;
    return r;
}

static inline Mat4 mat4_perspective(float fov_rad, float aspect, float near, float far) {
    Mat4 r = {};
    float f = 1.0f / tanf(fov_rad * 0.5f);
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    r.m[10] = (far + near) / (near - far);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * far * near) / (near - far);
    return r;
}

static inline Mat4 mat4_lookat(Vec3 eye, Vec3 center, Vec3 up) {
    float fx = center.x - eye.x, fy = center.y - eye.y, fz = center.z - eye.z;
    float len = sqrtf(fx*fx + fy*fy + fz*fz);
    fx /= len; fy /= len; fz /= len;

    // s = f × up
    float sx = fy * up.z - fz * up.y;
    float sy = fz * up.x - fx * up.z;
    float sz = fx * up.y - fy * up.x;
    len = sqrtf(sx*sx + sy*sy + sz*sz);
    sx /= len; sy /= len; sz /= len;

    // u = s × f
    float ux = sy * fz - sz * fy;
    float uy = sz * fx - sx * fz;
    float uz = sx * fy - sy * fx;

    Mat4 r = mat4_identity();
    r.m[0] = sx;  r.m[4] = sy;  r.m[8]  = sz;
    r.m[1] = ux;  r.m[5] = uy;  r.m[9]  = uz;
    r.m[2] = -fx; r.m[6] = -fy; r.m[10] = -fz;
    r.m[12] = -(sx * eye.x + sy * eye.y + sz * eye.z);
    r.m[13] = -(ux * eye.x + uy * eye.y + uz * eye.z);
    r.m[14] = (fx * eye.x + fy * eye.y + fz * eye.z);
    return r;
}

// ─── Shader compilation ────────────────────────────────────────────

static GLuint compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        fprintf(stderr, "Shader error: %s\n", log);
    }
    return s;
}

static GLuint create_program(const char* vert_src, const char* frag_src) {
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vert_src);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, frag_src);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
}

// ─── Vertex data: position (3) + normal (3) ────────────────────────

struct Vertex {
    float px, py, pz;
    float nx, ny, nz;
};

// ─── Mesh: VAO + VBO + vertex count ───────────────────────────────

struct Mesh {
    GLuint vao = 0, vbo = 0;
    int count = 0;

    void upload(const std::vector<Vertex>& verts) {
        count = (int)verts.size();
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    void draw() const {
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, count);
    }
};

// ─── Geometry generators ───────────────────────────────────────────

// Unit cube centered at origin
static std::vector<Vertex> gen_cube() {
    std::vector<Vertex> v;
    // 6 faces, 2 triangles each
    auto face = [&](Vec3 n, Vec3 u, Vec3 r) {
        // 4 corners
        float hx = 0.5f;
        Vertex v0 = {n.x*hx - u.x*hx - r.x*hx, n.y*hx - u.y*hx - r.y*hx, n.z*hx - u.z*hx - r.z*hx, n.x, n.y, n.z};
        Vertex v1 = {n.x*hx + u.x*hx - r.x*hx, n.y*hx + u.y*hx - r.y*hx, n.z*hx + u.z*hx - r.z*hx, n.x, n.y, n.z};
        Vertex v2 = {n.x*hx + u.x*hx + r.x*hx, n.y*hx + u.y*hx + r.y*hx, n.z*hx + u.z*hx + r.z*hx, n.x, n.y, n.z};
        Vertex v3 = {n.x*hx - u.x*hx + r.x*hx, n.y*hx - u.y*hx + r.y*hx, n.z*hx - u.z*hx + r.z*hx, n.x, n.y, n.z};
        v.push_back(v0); v.push_back(v1); v.push_back(v2);
        v.push_back(v0); v.push_back(v2); v.push_back(v3);
    };
    face({0,0,1},  {0,1,0}, {1,0,0});   // +Z
    face({0,0,-1}, {0,1,0}, {-1,0,0});  // -Z
    face({1,0,0},  {0,1,0}, {0,0,-1});  // +X
    face({-1,0,0}, {0,1,0}, {0,0,1});   // -X
    face({0,1,0},  {0,0,1}, {1,0,0});   // +Y
    face({0,-1,0}, {0,0,-1},{1,0,0});    // -Y
    return v;
}

// Cylinder along +Y axis, centered at origin, radius=0.5, height=1
static std::vector<Vertex> gen_cylinder(int segments = 24) {
    std::vector<Vertex> v;
    float r = 0.5f, h = 0.5f;
    for (int i = 0; i < segments; i++) {
        float a0 = 2.0f * M_PI * i / segments;
        float a1 = 2.0f * M_PI * (i + 1) / segments;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);

        // Side quad (2 triangles)
        Vertex bl = {r*c0, -h, r*s0, c0, 0, s0};
        Vertex br = {r*c1, -h, r*s1, c1, 0, s1};
        Vertex tl = {r*c0,  h, r*s0, c0, 0, s0};
        Vertex tr = {r*c1,  h, r*s1, c1, 0, s1};
        v.push_back(bl); v.push_back(br); v.push_back(tr);
        v.push_back(bl); v.push_back(tr); v.push_back(tl);

        // Top cap
        Vertex tc = {0, h, 0, 0, 1, 0};
        Vertex t0 = {r*c0, h, r*s0, 0, 1, 0};
        Vertex t1 = {r*c1, h, r*s1, 0, 1, 0};
        v.push_back(tc); v.push_back(t0); v.push_back(t1);

        // Bottom cap
        Vertex bc = {0, -h, 0, 0, -1, 0};
        Vertex b0 = {r*c0, -h, r*s0, 0, -1, 0};
        Vertex b1 = {r*c1, -h, r*s1, 0, -1, 0};
        v.push_back(bc); v.push_back(b1); v.push_back(b0);
    }
    return v;
}

// Grid on the XZ plane at y=0
static std::vector<Vertex> gen_grid(int half_extent = 5, float spacing = 0.05f) {
    std::vector<Vertex> v;
    float ext = half_extent * spacing;
    float t = 0.001f; // thin line thickness
    for (int i = -half_extent; i <= half_extent; i++) {
        float p = i * spacing;
        // Line along Z
        Vertex a = {p - t, 0, -ext, 0, 1, 0};
        Vertex b = {p + t, 0, -ext, 0, 1, 0};
        Vertex c = {p + t, 0,  ext, 0, 1, 0};
        Vertex d = {p - t, 0,  ext, 0, 1, 0};
        v.push_back(a); v.push_back(b); v.push_back(c);
        v.push_back(a); v.push_back(c); v.push_back(d);
        // Line along X
        Vertex e = {-ext, 0, p - t, 0, 1, 0};
        Vertex f = { ext, 0, p - t, 0, 1, 0};
        Vertex g = { ext, 0, p + t, 0, 1, 0};
        Vertex h = {-ext, 0, p + t, 0, 1, 0};
        v.push_back(e); v.push_back(f); v.push_back(g);
        v.push_back(e); v.push_back(g); v.push_back(h);
    }
    return v;
}

// Sphere (for pivot joint visualization)
static std::vector<Vertex> gen_sphere(int stacks = 12, int slices = 16) {
    std::vector<Vertex> v;
    for (int i = 0; i < stacks; i++) {
        float t0 = M_PI * i / stacks;
        float t1 = M_PI * (i + 1) / stacks;
        for (int j = 0; j < slices; j++) {
            float p0 = 2.0f * M_PI * j / slices;
            float p1 = 2.0f * M_PI * (j + 1) / slices;

            auto vert = [](float t, float p) -> Vertex {
                float x = sinf(t) * cosf(p);
                float y = cosf(t);
                float z = sinf(t) * sinf(p);
                return {0.5f * x, 0.5f * y, 0.5f * z, x, y, z};
            };

            Vertex a = vert(t0, p0), b = vert(t1, p0);
            Vertex c = vert(t1, p1), d = vert(t0, p1);
            v.push_back(a); v.push_back(b); v.push_back(c);
            v.push_back(a); v.push_back(c); v.push_back(d);
        }
    }
    return v;
}

// ═══════════════════════════════════════════════════════════════════
// QubeRenderer — encapsulates all 3D state
// ═══════════════════════════════════════════════════════════════════

struct QubeRenderer {
    // FBO for offscreen rendering
    GLuint fbo = 0, rbo_depth = 0, tex_color = 0;
    int fbo_w = 0, fbo_h = 0;

    // Shader
    GLuint program = 0;
    GLint u_mvp = -1, u_model = -1, u_color = -1, u_light_dir = -1;
    GLint u_ambient = -1, u_view_pos = -1;

    // Meshes
    Mesh cube_mesh, cyl_mesh, grid_mesh, sphere_mesh;

    // Camera
    float cam_angle_h = 0.6f;   // horizontal orbit angle
    float cam_angle_v = 0.35f;  // vertical angle (from horizontal)
    float cam_dist    = 0.55f;  // distance from center

    bool initialized = false;

    void init() {
        if (initialized) return;
        initialized = true;

        // ── Shaders ────────────────────────────────────────────────
        const char* vert_src = R"(
            #version 330 core
            layout(location=0) in vec3 aPos;
            layout(location=1) in vec3 aNorm;
            uniform mat4 uMVP;
            uniform mat4 uModel;
            out vec3 vNorm;
            out vec3 vWorldPos;
            void main() {
                vec4 wp = uModel * vec4(aPos, 1.0);
                vWorldPos = wp.xyz;
                vNorm = mat3(transpose(inverse(uModel))) * aNorm;
                gl_Position = uMVP * vec4(aPos, 1.0);
            }
        )";

        const char* frag_src = R"(
            #version 330 core
            in vec3 vNorm;
            in vec3 vWorldPos;
            uniform vec3 uColor;
            uniform vec3 uLightDir;
            uniform float uAmbient;
            uniform vec3 uViewPos;
            out vec4 FragColor;
            void main() {
                vec3 n = normalize(vNorm);
                vec3 l = normalize(uLightDir);
                float diff = max(dot(n, l), 0.0);

                // Specular (Blinn-Phong)
                vec3 viewDir = normalize(uViewPos - vWorldPos);
                vec3 halfDir = normalize(l + viewDir);
                float spec = pow(max(dot(n, halfDir), 0.0), 32.0) * 0.3;

                vec3 result = uColor * (uAmbient + diff * 0.7) + vec3(spec);
                FragColor = vec4(result, 1.0);
            }
        )";

        program = create_program(vert_src, frag_src);
        u_mvp       = glGetUniformLocation(program, "uMVP");
        u_model     = glGetUniformLocation(program, "uModel");
        u_color     = glGetUniformLocation(program, "uColor");
        u_light_dir = glGetUniformLocation(program, "uLightDir");
        u_ambient   = glGetUniformLocation(program, "uAmbient");
        u_view_pos  = glGetUniformLocation(program, "uViewPos");

        // ── Geometry ───────────────────────────────────────────────
        cube_mesh.upload(gen_cube());
        cyl_mesh.upload(gen_cylinder(32));
        grid_mesh.upload(gen_grid(8, 0.04f));
        sphere_mesh.upload(gen_sphere(16, 24));
    }

    void ensure_fbo(int w, int h) {
        if (w == fbo_w && h == fbo_h && fbo != 0) return;
        if (fbo) {
            glDeleteFramebuffers(1, &fbo);
            glDeleteRenderbuffers(1, &rbo_depth);
            glDeleteTextures(1, &tex_color);
        }
        fbo_w = w; fbo_h = h;

        glGenTextures(1, &tex_color);
        glBindTexture(GL_TEXTURE_2D, tex_color);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glGenRenderbuffers(1, &rbo_depth);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);

        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex_color, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo_depth);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    // Draw a mesh with a given model matrix and color
    void draw_mesh(const Mesh& mesh, const Mat4& model, const Mat4& vp,
                   float r, float g, float b) {
        Mat4 mvp = mat4_mul(vp, model);
        glUniformMatrix4fv(u_mvp, 1, GL_FALSE, mvp.m);
        glUniformMatrix4fv(u_model, 1, GL_FALSE, model.m);
        glUniform3f(u_color, r, g, b);
        mesh.draw();
    }

    // ── Main render call ───────────────────────────────────────────
    // theta = arm angle (rad), alpha = pendulum angle from upright (rad)
    // Returns the FBO texture ID for ImGui::Image()
    GLuint render(int w, int h, float theta, float alpha) {
        init();
        ensure_fbo(w, h);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, w, h);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(program);

        // Camera
        float aspect = (float)w / (float)h;
        Vec3 eye = {
            cam_dist * cosf(cam_angle_v) * sinf(cam_angle_h),
            cam_dist * sinf(cam_angle_v) + 0.08f,
            cam_dist * cosf(cam_angle_v) * cosf(cam_angle_h)
        };
        Vec3 center = {0.0f, 0.06f, 0.0f};
        Vec3 up = {0.0f, 1.0f, 0.0f};

        Mat4 view = mat4_lookat(eye, center, up);
        Mat4 proj = mat4_perspective(45.0f * M_PI / 180.0f, aspect, 0.01f, 10.0f);
        Mat4 vp   = mat4_mul(proj, view);

        // Lighting
        glUniform3f(u_light_dir, 0.4f, 0.8f, 0.3f);
        glUniform1f(u_ambient, 0.25f);
        glUniform3f(u_view_pos, eye.x, eye.y, eye.z);

        // ── Dimensions (meters, true scale) ────────────────────────
        // Qube body: ~5.5cm cube
        float body_size = 0.055f;
        // Arm length: pivot to pendulum = 8.26cm
        float arm_len   = 0.0826f;
        float arm_radius = 0.004f;
        // Pendulum: 12.9cm total length, ~3mm radius rod
        float pend_len    = 0.129f;
        float pend_radius = 0.003f;
        // Motor hub height above cube top
        float hub_h = 0.012f;
        // Body sits on the ground: bottom at y=0, top at y=body_size
        float body_top = body_size;

        // ── Ground grid ────────────────────────────────────────────
        draw_mesh(grid_mesh, mat4_identity(), vp, 0.25f, 0.25f, 0.30f);

        // ── Qube body (dark cube) ──────────────────────────────────
        {
            Mat4 m = mat4_mul(
                mat4_translate(0, body_size * 0.5f, 0),
                mat4_scale(body_size, body_size, body_size)
            );
            draw_mesh(cube_mesh, m, vp, 0.15f, 0.15f, 0.18f);
        }

        // ── Motor hub (small cylinder on top, rotates with theta) ──
        float hub_y = body_top + hub_h * 0.5f;
        Mat4 arm_rotate = mat4_rotate_y(-theta);  // arm rotation around vertical axis

        {
            Mat4 m = mat4_mul(
                mat4_mul(mat4_translate(0, hub_y, 0), arm_rotate),
                mat4_scale(0.012f, hub_h, 0.012f)
            );
            draw_mesh(cyl_mesh, m, vp, 0.55f, 0.55f, 0.58f);
        }

        // ── Arm (horizontal cylinder from hub to pendulum pivot) ───
        // The arm goes along +X in the rotated frame
        float arm_y = body_top + hub_h;
        {
            // Cylinder is along Y by default. We need it along X.
            // Scale: radius in X,Z, length in Y → then rotate 90° around Z
            Mat4 m = mat4_mul(
                mat4_translate(0, arm_y, 0),
                mat4_mul(
                    arm_rotate,
                    mat4_mul(
                        mat4_translate(arm_len * 0.5f, 0, 0),
                        mat4_mul(
                            mat4_rotate_z(-M_PI * 0.5f),
                            mat4_scale(arm_radius * 2, arm_len, arm_radius * 2)
                        )
                    )
                )
            );
            draw_mesh(cyl_mesh, m, vp, 0.65f, 0.65f, 0.68f);
        }

        // ── Pivot joint (small sphere at arm tip) ──────────────────
        // Arm tip position in world coordinates
        float pivot_x = arm_len * cosf(-theta);
        float pivot_z = arm_len * sinf(-theta);
        float pivot_y = arm_y;
        {
            Mat4 m = mat4_mul(
                mat4_translate(pivot_x, pivot_y, pivot_z),
                mat4_scale(0.008f, 0.008f, 0.008f)
            );
            draw_mesh(sphere_mesh, m, vp, 0.5f, 0.5f, 0.55f);
        }

        // ── Pendulum rod ───────────────────────────────────────────
        // alpha=0 → upright (+Y), alpha=π → hanging down (-Y)
        // The pendulum swings in the plane defined by the arm direction.
        // Its rotation axis is perpendicular to the arm, horizontal.
        //
        // In the arm's local frame (arm along +X):
        //   pendulum hangs in the XY plane
        //   rotation axis is Z (perpendicular to arm, horizontal)
        //   alpha=0 → +Y (up), alpha=π → -Y (down)
        //
        // Steps:
        //   1. Start with cylinder along +Y (default)
        //   2. Offset so bottom is at origin (translate up by half length)
        //   3. Rotate by alpha around Z axis (in arm-local frame)
        //   4. Rotate by theta around Y (arm rotation)
        //   5. Translate to pivot point
        {
            // The pendulum rotation axis is perpendicular to the arm (horizontal).
            // rotate_z rotates +Y toward -X, but positive alpha should swing
            // along +X (the arm direction), so we negate alpha.
            Mat4 m = mat4_mul(
                mat4_translate(pivot_x, pivot_y, pivot_z),
                mat4_mul(
                    arm_rotate,  // rotate into arm frame
                    mat4_mul(
                        mat4_rotate_z(-alpha),  // pendulum swing (in arm's XY plane)
                        mat4_mul(
                            mat4_translate(0, pend_len * 0.5f, 0),  // offset so bottom at pivot
                            mat4_scale(pend_radius * 2, pend_len, pend_radius * 2)
                        )
                    )
                )
            );
            draw_mesh(cyl_mesh, m, vp, 0.9f, 0.45f, 0.2f);
        }

        // ── Pendulum tip (small sphere) ────────────────────────────
        // Use the exact same transform chain as the rod, but translate
        // to the tip (full pend_len along +Y in the rod's local frame).
        {
            Mat4 m = mat4_mul(
                mat4_translate(pivot_x, pivot_y, pivot_z),
                mat4_mul(
                    arm_rotate,
                    mat4_mul(
                        mat4_rotate_z(-alpha),
                        mat4_mul(
                            mat4_translate(0, pend_len, 0),
                            mat4_scale(0.006f, 0.006f, 0.006f)
                        )
                    )
                )
            );
            draw_mesh(sphere_mesh, m, vp, 1.0f, 0.55f, 0.25f);
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDisable(GL_DEPTH_TEST);

        return tex_color;
    }
};
