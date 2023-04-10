#include "boid.hpp"
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <random>
#include <vector>
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "p6/p6.h"

void Boid::findNeighbors(std::vector<Boid>& boids)
{
    for (auto& boid : boids)
    {
        if (&boid != this && glm::distance(this->position, boid.position) < this->detection_radius + boid.detection_radius)
        {
            this->neighbors.push_back(boid);
        }
    }
}

void Boid::draw(p6::Context& ctx)
{
    const auto sg1 = ctx.transform_scope_guard();

    ctx.fill       = {0.f, 0.f, 0.f, 0.8f};
    ctx.use_stroke = false;

    ctx.push_transform();

    if (!this->neighbors.empty())
    {
        ctx.fill = {0.5f, 0.f, 0.1f, 0.5f};
    }
    else
    {
        ctx.fill = {0.f, 0.2f, 0.5f, 0.5f};
    }
    ctx.circle(this->position, p6::Radius{this->detection_radius});

    ctx.equilateral_triangle(
        p6::Center{this->position},
        p6::Radius{0.08f},
        p6::Rotation{p6::Angle(this->direction)}
    );

    ctx.pop_transform();
}

void Boid::checkBorders(p6::Context& ctx)
{
    outLeft(ctx);
    outRight(ctx);
    outTop();
    outBottom();
}

void Boid::outLeft(p6::Context& ctx)
{
    if (this->position.x - this->detection_radius < -ctx.aspect_ratio() + this->border_width)
    {
        this->position.x = -ctx.aspect_ratio() + this->detection_radius + this->border_width;
        this->direction.x *= -1;
    }
}

void Boid::outRight(p6::Context& ctx)
{
    if (this->position.x + this->detection_radius > ctx.aspect_ratio() - this->border_width)
    {
        this->position.x = ctx.aspect_ratio() - this->detection_radius - this->border_width;
        this->direction.x *= -1;
    }
}

void Boid::outTop()
{
    if (this->position.y + this->detection_radius > 1 - this->border_width)
    {
        this->position.y = 1 - this->detection_radius - this->border_width;
        this->direction.y *= -1;
    }
}

void Boid::outBottom()
{
    if (this->position.y - this->detection_radius < -1 + this->border_width)
    {
        this->position.y = -1 + this->detection_radius + this->border_width;
        this->direction.y *= -1;
    }
}

void Boid::move(p6::Context& ctx)
{
    this->position += this->direction * ctx.delta_time() * this->max_speed;
}

void Boid::UpdateDirection()
{
    glm::vec2 updateDirection(0, 0);
    for (auto& boid : this->neighbors)
    {
        updateDirection += boid.direction;
    }

    if (this->neighbors.empty() != 0)
    {
        updateDirection /= (static_cast<float>(this->neighbors.size()) + 1);
    }
    updateDirection = glm::normalize(updateDirection);

    this->direction = updateDirection;
}

void Boid::update(p6::Context& ctx, std::vector<Boid>& boids)
{
    findNeighbors(boids);
    move(ctx);
    UpdateDirection();
    checkBorders(ctx);
    Separation();
    Alignment(boids);

    // Cohesion(boids);

    draw(ctx);

    this->neighbors.clear();
}

void Boid::Separation()
{
    std::vector<Boid> collisions;
    glm::vec2         totalForce(0.0f, 0.0f);
    for (auto& boid : this->neighbors)
    {
        float distance = glm::distance(this->position, boid.position);
        if (distance != 0)
        {
            totalForce += (this->position - boid.position) / distance;
        }
        collisions.push_back(boid);
    }

    if (!collisions.empty())
    {
        totalForce /= static_cast<float>(collisions.size());
        this->direction += normalize(totalForce) * this->avoidance;
    }
}

void Boid::Alignment(const std::vector<Boid>& neighbors)
{
    glm::vec2 alignmentVector = {0.0f, 0.0f};
    int       neighborCount   = 0;

    // For each neighbor within the maximum alignment distance, add their direction to the alignment vector
    for (const auto& neighbor : neighbors)
    {
        if (glm::distance(neighbor.position, this->position) < this->detection_radius)
        {
            alignmentVector += neighbor.direction;
            neighborCount++;
        }
    }

    if (neighborCount > 0)
    {
        // Divide the alignment vector by the number of neighbors to get the average direction
        alignmentVector /= static_cast<float>(neighborCount);
        // Normalize the vector to get a unit vector in the direction of the average direction
        alignmentVector = glm::normalize(alignmentVector);
        std::cout << alignmentVector.x << std::endl;
    }

    this->direction += alignmentVector;
}

// void Boid::Cohesion(std::vector<Boid>& neighbors)
// {
//     int       count = 0;
//     glm::vec2 AveragePosition(0.0f, 0.0f);

//     for (auto& boid : neighbors)
//     {
//         if (glm::distance(this->position, boid.position) < this->detection_radius)
//         {
//             AveragePosition += boid.position;
//             count++;
//         }
//     }

//     if (count == 0)
//     {
//         this->position = glm::vec2(0.0f, 0.0f);
//     }
//     if (!neighbors.empty())
//     {
//         this->position /= static_cast<float>(neighbors.size());
//     }
//     AveragePosition /= static_cast<float>(count);
//     this->position = glm::normalize(AveragePosition - this->position) * this->max_speed - this->direction;
// }
