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
        if (&boid != this && glm::distance(this->boid_position, boid.boid_position) < this->detection_radius + boid.detection_radius)
        {
            this->neighbors.push_back(boid);
        }
    }
}

void Boid::draw(p6::Context& ctx)
{
    ctx.fill       = {0.f, 0.f, 0.f, 0.8f};
    ctx.use_stroke = false;

    if (!this->neighbors.empty())
    {
        ctx.fill = {0.5f, 0.f, 0.1f, 0.5f};
    }
    else
    {
        ctx.fill = {0.f, 0.2f, 0.5f, 0.5f};
    }
    ctx.circle(this->boid_position, p6::Radius{this->detection_radius});

    ctx.equilateral_triangle(
        p6::Center{this->boid_position},
        p6::Radius{0.08f},
        p6::Rotation{p6::Angle(this->boid_direction)}
    );
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
    float border_width = 0.2f;

    if (this->boid_position.x - this->detection_radius < -ctx.aspect_ratio() + border_width)
    {
        this->boid_position.x = -ctx.aspect_ratio() + this->detection_radius + border_width;
        this->boid_direction.x *= -1;
    }
}

void Boid::outRight(p6::Context& ctx)
{
    float border_width = 0.2f;
    if (this->boid_position.x + this->detection_radius > ctx.aspect_ratio() - border_width)
    {
        this->boid_position.x = ctx.aspect_ratio() - this->detection_radius - border_width;
        this->boid_direction.x *= -1;
    }
}

void Boid::outTop()
{
    float border_width = 0.2f;
    if (this->boid_position.y + this->detection_radius > 1 - border_width)
    {
        this->boid_position.y = 1 - this->detection_radius - border_width;
        this->boid_direction.y *= -1;
    }
}

void Boid::outBottom()
{
    float border_width = 0.2f;
    if (this->boid_position.y - this->detection_radius < -1 + border_width)
    {
        this->boid_position.y = -1 + this->detection_radius + border_width;
        this->boid_direction.y *= -1;
    }
}

void Boid::move(p6::Context& ctx)
{
    this->boid_position += this->boid_direction * ctx.delta_time() * this->max_speed;
}

void Boid::update(p6::Context& ctx, std::vector<Boid>& boids)
{
    findNeighbors(boids);
    move(ctx);
    checkBorders(ctx);
    Separate(boids);
    Align(boids);

    Cohese(boids);

    draw(ctx);

    this->neighbors.clear();
}

void Boid::Separate(const std::vector<Boid>& neighbors)
{
    glm::vec2 totalForce(0.0f, 0.0f);
    int       neighborCount = 0;

    for (const auto& boid : neighbors)
    {
        float distance = glm::distance(this->boid_position, boid.boid_position);

        if (distance != 0 && distance < this->detection_radius)
        {
            totalForce += (this->boid_position - boid.boid_position) / distance;
            neighborCount++;
        }
    }

    if (neighborCount > 0)
    {
        totalForce /= static_cast<float>(neighborCount);
        totalForce = normalize(totalForce);

        // std::cout << totalForce.x << std::endl;

        limitSpeed();
    }

    this->boid_direction += totalForce * this->separation_weight;
}

void Boid::Align(const std::vector<Boid>& neighbors)
{
    glm::vec2 alignmentVector = {0.0f, 0.0f};
    float     meanAlignment   = 0.0f;

    for (const auto& neighbor : neighbors)
    {
        float distance = glm::distance(this->boid_position, neighbor.boid_position);
        if (distance < this->detection_radius && distance != 0)
        {
            alignmentVector += neighbor.boid_direction * (1.0f / distance);
            meanAlignment += 1.0f / distance;
        }
    }

    if (meanAlignment > 0.0f)
    {
        alignmentVector /= meanAlignment;
        alignmentVector = glm::normalize(alignmentVector);
        // std::cout << alignmentVector.x << std::endl;

        limitSpeed();
    }

    this->boid_direction += alignmentVector * (this->alignment_weight);
}

void Boid::Cohese(const std::vector<Boid>& neighbors)
{
    glm::vec2 AveragePosition(0.0f, 0.0f);
    glm::vec2 cohesionDirection(0.0f, 0.0f);
    float     meanCohesion = 0.0f;

    for (const auto& boid : neighbors)
    {
        float distance = glm::distance(this->boid_position, boid.boid_position);
        if (distance < this->detection_radius && distance != 0)
        {
            AveragePosition += boid.boid_position * (1.0f / distance);
            meanCohesion += 1.0f / distance;
        }
    }

    if (meanCohesion > 0)
    {
        AveragePosition /= meanCohesion;
        AveragePosition   = normalize(AveragePosition);
        cohesionDirection = (AveragePosition - this->boid_position) * (this->cohesion_weight * 0.01f);

        limitSpeed();
    }

    this->boid_position += cohesionDirection;
}

void Boid::limitSpeed()
{
    float speed = glm::length(this->boid_direction);
    if (speed > this->max_speed)
    {
        this->boid_direction = normalize(this->boid_direction) * this->max_speed;
    }
}
