#include <stdlib.h>
#include <vcruntime.h>
#include <cmath>
#include "p6/p6.h"

#define DOCTEST_CONFIG_IMPLEMENT
#include <vector>
#include "boid.hpp"
#include "doctest/doctest.h"

int main(int argc, char* argv[])
{
    { // Run the tests
        if (doctest::Context{}.run() != 0)
            return EXIT_FAILURE;
        // The CI does not have a GPU so it cannot run the rest of the code.
        const bool no_gpu_available = argc >= 2 && strcmp(argv[1], "-nogpu") == 0; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        if (no_gpu_available)
            return EXIT_SUCCESS;
    }

    // Boid b1(glm::vec2(0, 0), glm::vec2(1, 2));

    // Actual app
    // auto ctx = p6::Context{{.title = "prog-algo"}};
    auto ctx = p6::Context{{1280, 720, "Dear ImGui"}};
    ctx.maximize_window();

    // data
    std::vector<Boid> boids;
    float             circle_radius = 0.1f;
    float             boid_speed    = 0.4f;
    float             cohesion      = 1.f;
    float             avoidance     = 0.5f;
    float             alignment     = 0.2f;
    int               number_boids  = 25;

    for (int i = 0; i <= number_boids; ++i)
    {
        glm::vec2 pos = p6::random::point(ctx);
        glm::vec2 dir = p6::random::point(ctx);
        Boid      boidx(pos, dir);
        boids.push_back(boidx);
    }

    ctx.imgui = [&]() {
        // Show a simple window
        ImGui::Begin("Choose your values");
        ImGui::SliderFloat("Speed", &boid_speed, 0.f, 2.f);
        ImGui::SliderFloat("Alignment", &alignment, 0.0f, 1.0f);
        ImGui::SliderFloat("Detection radius", &circle_radius, 0.01f, 0.5f);
        ImGui::SliderFloat("Cohesion", &cohesion, 1.f, 15.f);
        ImGui::SliderFloat("Separation", &avoidance, 0.1f, 1.f);
        ImGui::End();

        ImGui::ShowDemoWindow();
    };

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::LavenderFloral);

        for (auto& boid : boids)
        {
            boid.setDetectionRadius(circle_radius);
            // boid.setAlignment(alignment);
            // boid.setCohesion(cohesion);
            boid.setSeparation(avoidance);
            boid.setSpeed(boid_speed);

            boid.update(ctx, boids);
        }
    };

    // Should be done last. It starts the infinite loop.
    ctx.start();
}