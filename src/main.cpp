#include <vcruntime.h>
#include <cmath>
#include <cstdlib>
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

    // Actual app
    auto ctx = p6::Context{{1280, 720, "Boids project"}};
    ctx.maximize_window();

    // data
    std::vector<Boid> boids;
    int               number_boids = 25;
    Parameters        parameters{0.1f, 0.7f, 0.2f, 0.05f, 0.2f};

    for (int i = 0; i <= number_boids; ++i)
    {
        glm::vec2       pos = p6::random::point(ctx);
        glm::vec2       dir = p6::random::point(ctx);
        BoidsAttributes attributes{.position = pos, .direction = dir};
        Boid            boidx(attributes);
        boids.push_back(boidx);
    }

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::LavenderFloral);

        parameters.updateParameters();
        for (auto& boid : boids)
        {
            boid.update(ctx, boids, parameters);
        }
    };

    // Should be done last. It starts the infinite loop.
    ctx.start();
}