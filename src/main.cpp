#include <math.h>
#include <stdlib.h>
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
    auto ctx = p6::Context{{.title = "prog-algo"}};
    ctx.maximize_window();
    std::vector<Boid> boids;
    int               Nboids = 20;
    for (int i = 0; i <= Nboids; ++i)
    {
        glm::vec2 position  = p6::random::point(ctx);
        glm::vec2 direction = p6::random::point(ctx);
        Boid      boidx(position, direction);
        boids.push_back(boidx);
    }

    p6::Angle rotation = 0.011_turn;

    // Declare your infinite update loop.
    ctx.update = [&]() {
        ctx.background(p6::NamedColor::LavenderFloral);
        // ctx.circle(
        //     p6::Center{ctx.mouse()},
        //     p6::Radius{0.05f}
        // );
        ctx.fill = {1, 1, 1, 1};
        for (int i = 0; i < Nboids; i++)
        {
            boids[i].draw(ctx);
            boids[i].update(ctx);
            boids[i].rebond(ctx);
        }
    };

    // Should be done last. It starts the infinite loop.
    ctx.start();
}