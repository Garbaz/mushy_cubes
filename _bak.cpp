void physics_step(float dt) {
    const float h = dt;

    // Spring constants
    const float L = 1.0;
    const float k = 1.0;

    // Number of jacobi solver steps
    const int SOLVER_STEPS = 3;

    auto vel_step = [h, L, k](glm::vec3 x, glm::vec3 xo, glm::vec3 v) -> glm::vec3 {
        glm::vec3 d = xo - x;
        float dl = glm::length(d);

        glm::vec3 v_ = v;

        glm::vec3 F = k / L * (d - L * d / dl);

        glm::mat3 J = k / L * (-glm::mat3(1) + L / dl * (glm::mat3(1) - outerProduct(d, d) / (dl * dl)));

        glm::mat3 A = (glm::mat3(1) - h * h * J);
        glm::vec3 s = v_ + h * F;

        glm::mat3 Di = inverse(matrixCompMult(glm::mat3(1), A));
        glm::vec3 vl = v;

        for (int i = 0; i < SOLVER_STEPS; i++) {
            vl = vl + 0.5f * Di * (s - A * vl);
        }

        return vl;

        std::cout << glm::to_string(J) << std::endl;
    };

    glm::vec3 v0 = vel_step(particles[0].pos,particles[1].pos, particles[0].vel);
    glm::vec3 v1 = vel_step(particles[1].pos,particles[0].pos, particles[1].vel);
    particles[0].vel = v0;
    particles[1].vel = v1;

    for(size_t i = 0; i < NUM_PARTICLES; i++) {
        Particle *p = &particles[i];
        p->pos += dt * p->vel;
    }
}








Particle particles[] = {
    {glm::vec3(-1, -1, 0), glm::vec3(0, 0, 0)},
    {glm::vec3(1, -1, 0), glm::vec3(0, 0, 0)},
    {glm::vec3(1, 1, 0), glm::vec3(0, 0, 0)},
    {glm::vec3(-1, 1, 0), glm::vec3(0, 0, 0)},
    {glm::vec3(-1, -1, 2), glm::vec3(0, 0, 0)},
    {glm::vec3(1, -1, 2), glm::vec3(0, 0, 0)},
    {glm::vec3(1, 1, 2), glm::vec3(0, 0, 0)},
    {glm::vec3(-1, 1, 2), glm::vec3(0, 0, 0)},
};

glm::uvec2 springs[] = {
    //Cube
    glm::uvec2(0, 1),
    glm::uvec2(1, 2),
    glm::uvec2(2, 3),
    glm::uvec2(3, 0),
    glm::uvec2(4, 5),
    glm::uvec2(5, 6),
    glm::uvec2(6, 7),
    glm::uvec2(7, 4),
    glm::uvec2(0, 4),
    glm::uvec2(1, 5),
    glm::uvec2(2, 6),
    glm::uvec2(3, 7),
    //Face Crosses
    glm::uvec2(0,2),
    glm::uvec2(1,3),
    glm::uvec2(4,6),
    glm::uvec2(5,7),
    glm::uvec2(0,5),
    glm::uvec2(1,4),
    glm::uvec2(2,7),
    glm::uvec2(3,6),
    glm::uvec2(1,6),
    glm::uvec2(2,5),
    glm::uvec2(0,7),
    glm::uvec2(3,4),
    //Internal Crosses
    glm::uvec2(0,6),
    glm::uvec2(1,7),
    glm::uvec2(2,4),
    glm::uvec2(3,5),
};