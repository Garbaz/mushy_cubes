#include <GL/glew.h>
#include <GLFW/glfw3.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>
#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>

#include "GLUtils.hpp"
#include "ShaderProgram.hpp"
#include "Camera.hpp"

#include "Main.hpp"

#define DBG(s) std::cerr << "DEBUG @" << __LINE__ << " : " << s << std::endl

template <typename T>
size_t vectorsizeof(const typename std::vector<T> &vec) {
    return sizeof(T) * vec.size();
}

#define PRINT_DEBUG true

const unsigned int CUBE_RESOLUTION = 4;
const unsigned int TOTAL_NUM_PARTICLES = CUBE_RESOLUTION * CUBE_RESOLUTION * CUBE_RESOLUTION;
const float CUBE_SIZE = 4.0;
const float CUBE_MASS = 10.0;
const float CUBE_SPRING_K = 250.0;
const float VELOCITY_DAMPENING = 0.98;

const glm::vec3 PROJECTILE_COLOR = glm::vec3(1.0);
const float PROJECTILE_RADIUS = 1.0;
const float PROJECTILE_RENDER_RADIUS = PROJECTILE_RADIUS / 0.02;  //TODO
const float PROJECTILE_SHOOT_VEL = 10.0;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

GLFWwindow *window;
glm::ivec2 viewport_size;

bool mouse_captured = false;
glm::ivec2 cursor_pos;

Camera camera(glm::vec3(0, 0, -10));

struct Particle {
    glm::vec3 vel;
    float mass;
};
std::vector<Particle> particles(TOTAL_NUM_PARTICLES);
std::vector<glm::vec3> particle_pos(TOTAL_NUM_PARTICLES);
#define PARTICLES_INDEX(x, y, z) ((x) + (y)*CUBE_RESOLUTION + (z)*CUBE_RESOLUTION * CUBE_RESOLUTION)

struct Spring {
    float L;  // The rest length
    float k;  // The spring constant
};
std::vector<Spring> springs;
std::vector<glm::uvec2> spring_indices;
void add_spring_between(const glm::uvec2 &indices) {
    spring_indices.push_back(indices);
    float L = glm::distance(particle_pos[indices.x], particle_pos[indices.y]);
    springs.push_back({L, CUBE_SPRING_K});
}

struct Projectile {
    glm::vec3 pos;
    glm::vec3 vel;
    glm::vec3 color;
    float radius;
};
std::vector<Projectile> projectiles;
void spawn_projectile(glm::vec3 pos, glm::vec3 vel) {
    projectiles.push_back({pos, vel, PROJECTILE_COLOR, PROJECTILE_RENDER_RADIUS});
}

void physics_step(float dt) {
    for (Projectile &p : projectiles) {
        p.vel += dt * GRAVITY;
        p.pos += dt * p.vel;
    }

    for (size_t i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];
        glm::vec3 &pos = particle_pos[i];

        //Gravity
        p.vel += dt * GRAVITY;
    }

    {  //Implicit Euler spring simulation
        const float h = dt;
        const float h2 = h * h;

        // Jacobi solver parameters
        const int SOLVER_STEPS = 3;
        const float OMEGA = 0.5;

        struct SpringData {
            /*const after init*/
            glm::mat3 S;  // Jacobian of the force
        } spring_data[springs.size()];

        struct ParticleData {
            /*const after init*/
            glm::mat3 D = glm::mat3(0);  // Diagonal of A (inverted!)
            glm::vec3 s = glm::vec3(0);  // The right side of our linear equation

            /*changes each Jacobi iteration*/
            glm::vec3 t = glm::vec3(0);  // t = A*v_
            glm::vec3 vl;                // The velocity approximated by the Jacobi solver
        } particle_data[particles.size()];

        // Spring initialization
        for (size_t i = 0; i < springs.size(); i++) {
            Spring &sp = springs[i];
            unsigned int ix = spring_indices[i].x, iy = spring_indices[i].y;
            const Particle &px = particles[ix], &py = particles[iy];

            glm::vec3 d = particle_pos[iy] - particle_pos[ix];
            float l = glm::length(d);
            // Derivative of the force applied by this spring on the particles
            spring_data[i].S = sp.k / sp.L * (-glm::mat3(1) + sp.L / l * (glm::mat3(1) - glm::outerProduct(d, d) / (l * l)));

            // This springs contribution to D (from matrix A)
            glm::mat3 D = glm::matrixCompMult(glm::mat3(1), spring_data[i].S);
            particle_data[ix].D += D;
            particle_data[iy].D += D;

            // The force applied to each particle by this spring
            glm::vec3 f = sp.k / sp.L * (d - sp.L * d / l);
            particle_data[ix].s += h * f;
            particle_data[iy].s -= h * f;
        }

        // Particle initialization
        for (size_t i = 0; i < particles.size(); i++) {
            particle_data[i].D = glm::inverse(glm::mat3(particles[i].mass) - h2 * particle_data[i].D);  // Invert D here once for efficiency
            particle_data[i].s += particles[i].mass * particles[i].vel;
            particle_data[i].vl = particles[i].vel;
        }

        // Jacobi solver
        for (size_t unused = 0; unused < SOLVER_STEPS; unused++) {
            for (size_t i = 0; i < particles.size(); i++) {
                particle_data[i].t = particles[i].mass * particle_data[i].vl;
            }
            for (size_t i = 0; i < springs.size(); i++) {
                const SpringData &sd = spring_data[i];
                ParticleData &dx = particle_data[spring_indices[i].x], &dy = particle_data[spring_indices[i].y];

                dx.t += -h2 * sd.S * (dx.vl - dy.vl);
                dy.t += -h2 * sd.S * (dy.vl - dx.vl);
            }
            for (size_t i = 0; i < particles.size(); i++) {
                ParticleData &pd = particle_data[i];
                pd.vl += OMEGA * pd.D * (pd.s - pd.t);
            }
        }
        for (size_t i = 0; i < particles.size(); i++) {
            particles[i].vel = particle_data[i].vl;
        }
    }

    for (size_t i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];
        glm::vec3 &pos = particle_pos[i];

        // Dampening
        p.vel *= VELOCITY_DAMPENING;

        // Collision with projectiles
        for (Projectile &pj : projectiles) {
            glm::vec3 r = (pos + dt * p.vel) - pj.pos;
            if (glm::length(r) < PROJECTILE_RADIUS) {
                p.vel = glm::reflect(p.vel, pos - pj.pos);
                pos += r;
            }
        }

        // Step
        pos += dt * p.vel;

        // Collision with floor
        if (pos.y < 0) {
            p.vel.y = -p.vel.y;
            pos.y = -pos.y;
        }
    }
}

std::vector<glm::uvec3> faces;

bool draw_vertecies = true, draw_edges = true, draw_faces = true;

size_t picked_particle;

std::vector<ShaderProgram *> shaders;

int main() {
    window = init_glfw("Mushy Cubes");
    init_glew();

    glfwGetFramebufferSize(window, &viewport_size.x, &viewport_size.y);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_PROGRAM_POINT_SIZE);

    glEnable(GL_LINE_SMOOTH);

    glClearColor(0.8, 1.0, 1.0, 1.0);

    for (size_t x = 0; x < CUBE_RESOLUTION; x++) {
        for (size_t y = 0; y < CUBE_RESOLUTION; y++) {
            for (size_t z = 0; z < CUBE_RESOLUTION; z++) {
                size_t i = PARTICLES_INDEX(x, y, z);
                Particle &p = particles[i];
                particle_pos[i] = glm::vec3(x, y, z) / CUBE_RESOLUTION * CUBE_SIZE;
                p.vel = glm::vec3(0);
                p.mass = CUBE_MASS / TOTAL_NUM_PARTICLES;

                //axis aligned springs
                if (x >= 1) add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x - 1, y, z)));
                if (y >= 1) add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x, y - 1, z)));
                if (z >= 1) add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x, y, z - 1)));

                //cross springs
                if (x >= 1 && y >= 1) {
                    add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x - 1, y - 1, z)));
                    add_spring_between(glm::uvec2(PARTICLES_INDEX(x - 1, y, z), PARTICLES_INDEX(x, y - 1, z)));
                }
                if (y >= 1 && z >= 1) {
                    add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x, y - 1, z - 1)));
                    add_spring_between(glm::uvec2(PARTICLES_INDEX(x, y - 1, z), PARTICLES_INDEX(x, y, z - 1)));
                }
                if (z >= 1 && x >= 1) {
                    add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x - 1, y, z - 1)));
                    add_spring_between(glm::uvec2(PARTICLES_INDEX(x, y, z - 1), PARTICLES_INDEX(x - 1, y, z)));
                }

                //internal springs
                if (x >= 1 && y >= 1 && z >= 1) {
                    add_spring_between(glm::uvec2(i, PARTICLES_INDEX(x - 1, y - 1, z - 1)));
                    add_spring_between(glm::uvec2(PARTICLES_INDEX(x - 1, y, z), PARTICLES_INDEX(x, y - 1, z - 1)));
                    add_spring_between(glm::uvec2(PARTICLES_INDEX(x, y - 1, z), PARTICLES_INDEX(x - 1, y, z - 1)));
                    add_spring_between(glm::uvec2(PARTICLES_INDEX(x, y, z - 1), PARTICLES_INDEX(x - 1, y - 1, z)));
                }
            }
        }
    }

    GLuint vbo_particles;
    glGenBuffers(1, &vbo_particles);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_particles);
    glBufferData(GL_ARRAY_BUFFER, vectorsizeof(particle_pos), particle_pos.data(), GL_STATIC_DRAW);

    // GLuint ebo_faces;
    // glGenBuffers(1, &ebo_faces);
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_faces);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(faces), faces, GL_STATIC_DRAW);

    GLuint ebo_edges;
    glGenBuffers(1, &ebo_edges);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, vectorsizeof(spring_indices), spring_indices.data(), GL_STATIC_DRAW);

    ShaderProgram prog_render_point = ShaderProgram("shaders/vertex.vert", "shaders/point.frag");
    shaders.push_back(&prog_render_point);
    prog_render_point.point_attribute("position", 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
    ShaderProgram prog_render_line = ShaderProgram("shaders/vertex.vert", "shaders/line.frag");
    shaders.push_back(&prog_render_line);
    prog_render_line.point_attribute("position", 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);
    ShaderProgram prog_render_triangle = ShaderProgram("shaders/vertex.vert", "shaders/triangle.frag");
    shaders.push_back(&prog_render_triangle);
    prog_render_triangle.point_attribute("position", 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), 0);

    GLuint vbo_projectiles;
    glGenBuffers(1, &vbo_projectiles);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_projectiles);

    ShaderProgram prog_render_projectile("shaders/projectile.vert", "shaders/projectile.frag");
    shaders.push_back(&prog_render_projectile);
    prog_render_projectile.point_attribute("position", 3, GL_FLOAT, GL_FALSE, sizeof(Projectile), (void *)offsetof(Projectile, pos));
    prog_render_projectile.point_attribute("color", 3, GL_FLOAT, GL_FALSE, sizeof(Projectile), (void *)offsetof(Projectile, color));
    prog_render_projectile.point_attribute("radius", 1, GL_FLOAT, GL_FALSE, sizeof(Projectile), (void *)offsetof(Projectile, radius));

    { /* Generate projection matrix and pass to all shaders*/
        float aspect = float(viewport_size.x) / float(viewport_size.y);
        float fovy = FIELD_OF_VIEW_HORIZONTAL / aspect;
        glm::mat4 proj = glm::perspective(glm::radians(fovy), aspect, 0.01f, 100.0f);
        // DBG(glm::to_string(proj));
        for (ShaderProgram *sp : shaders) {
            sp->set_uniform("proj", proj);
        }
    }

    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, cursor_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    double start_time = glfwGetTime();
    double last_frame_time = start_time;

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update time & deltatime
        double frame_time = glfwGetTime();
        double time = frame_time - start_time;
        double deltatime = frame_time - last_frame_time;
        last_frame_time = frame_time;

        // Update camera
        camera.update(deltatime);
        if (camera.dirty) {
            glm::mat4 view = camera.get_view_matrix();
            for (ShaderProgram *sp : shaders) {
                sp->set_uniform("view", view);
            }
        }

        /* ----- PHYSICS ----- */

        physics_step(deltatime);

        /* ----- RENDER ----- */

        /* MUSHY CUBES */

        glBindBuffer(GL_ARRAY_BUFFER, vbo_particles);
        {
            //Update particle data
            glBufferSubData(GL_ARRAY_BUFFER, 0, vectorsizeof(particle_pos), particle_pos.data());

            // //FACES
            // if (draw_faces) {
            //     prog_render_triangle.use();
            //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_faces);
            //     glDrawElements(GL_TRIANGLES, sizeof(faces) / sizeof(GLuint), GL_UNSIGNED_INT, 0);
            //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            // }
            //EDGES
            if (draw_edges) {
                prog_render_line.use();
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_edges);
                glDrawElements(GL_LINES, 2 * spring_indices.size(), GL_UNSIGNED_INT, 0);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            }
            //VERTICES
            if (draw_vertecies) {
                prog_render_point.use();
                glDrawArrays(GL_POINTS, 0, particle_pos.size());
            }
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        /* PROJECTILES */

        glBindBuffer(GL_ARRAY_BUFFER, vbo_projectiles);
        {
            //Update projectile data
            glBufferData(GL_ARRAY_BUFFER, vectorsizeof(projectiles), projectiles.data(), GL_STREAM_DRAW);

            prog_render_projectile.use();
            glDrawArrays(GL_POINTS, 0, projectiles.size());
        }
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        exit(0);
    } else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        if (mouse_captured) {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            mouse_captured = false;
        } else {
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            mouse_captured = true;
        }
    } else if (key == GLFW_KEY_W) {
        if (action == GLFW_PRESS) {
            camera.input_movement.y++;
        } else if (action == GLFW_RELEASE) {
            camera.input_movement.y--;
        }
    } else if (key == GLFW_KEY_S) {
        if (action == GLFW_PRESS) {
            camera.input_movement.y--;
        } else if (action == GLFW_RELEASE) {
            camera.input_movement.y++;
        }
    } else if (key == GLFW_KEY_A) {
        if (action == GLFW_PRESS) {
            camera.input_movement.x++;
        } else if (action == GLFW_RELEASE) {
            camera.input_movement.x--;
        }
    } else if (key == GLFW_KEY_D) {
        if (action == GLFW_PRESS) {
            camera.input_movement.x--;
        } else if (action == GLFW_RELEASE) {
            camera.input_movement.x++;
        }
    } else if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        draw_vertecies = !draw_vertecies;
    } else if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
        draw_edges = !draw_edges;
    } else if (key == GLFW_KEY_3 && action == GLFW_PRESS) {
        draw_faces = !draw_faces;
    } else if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        for (Particle &p : particles) {
            p.vel += glm::sphericalRand(10.0f);
        }
    } else if (key == GLFW_KEY_J && action == GLFW_PRESS) {
        for (Particle &p : particles) {
            p.vel += glm::vec3(0, 10, 0);
            p.vel += glm::sphericalRand(10.0);
        }
    }
}

void cursor_callback(GLFWwindow *window, double xpos, double ypos) {
    if (mouse_captured) {
        camera.update_direction(xpos - cursor_pos.x, ypos - cursor_pos.y);
    }
    cursor_pos.x = xpos;
    cursor_pos.y = ypos;
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    if (mouse_captured) {
        // if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        //     glm::vec3 camera_pos = camera.get_position();
        //     glm::vec3 camera_dir = camera.get_direction();
        //     spawn_projectile(camera_pos + 1.0 * camera_dir, PROJECTILE_SHOOT_VEL * camera_dir);
        // }
        // if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        //     glm::vec3 camera_pos = camera.get_position();
        //     glm::vec3 camera_dir = camera.get_direction();
        //     for (size_t i = 0; i < particles.size(); i++) {
        //         Particle &p = particles[i];
        //         glm::vec3 &pos = particle_pos[i];
        //         glm::vec3 rel_pos = pos - camera_pos;
        //         float dist = glm::distance(rel_pos, camera_dir * glm::dot(rel_pos, camera_dir));
        //         if(dist < 0.01) {

        //         }
        //     }
        // }
    }
}