#include <math.h>
#include <stdlib.h>
#include "p6/p6.h"
#define DOCTEST_CONFIG_IMPLEMENT
#include <vector>
#include "doctest/doctest.h"
class Boid {
private:
    glm::vec2 position;
    glm::vec2 direction;

public:
    Boid(){};
    Boid(glm::vec2 pos, glm::vec2 dir)
        : position(pos), direction(dir){};
    void draw(p6::Context& ctx);
    void update(p6::Context& ctx);
    bool sortie(p6::Context& ctx);
    void rebond(p6::Context& ctx);
};

void Boid::draw(p6::Context& ctx)
{
    ctx.equilateral_triangle(
        p6::Center{this->position},
        p6::Radius{0.08f},
        p6::Rotation{this->direction}
    );
}

void Boid::update(p6::Context& ctx)
{
    this->position += 0.02f * this->direction;
}

bool Boid::sortie(p6::Context& ctx)
{
    if (this->position.x > ctx.aspect_ratio() || this->position.x < -ctx.aspect_ratio() || this->position.y > 1 || this->position.y < -1)
    {
        return true;
    }
}

void Boid::rebond(p6::Context& ctx)
{
    if (sortie(ctx) == true)
    {
        this->direction = -(this->direction);
    }
}