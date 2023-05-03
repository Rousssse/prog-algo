#include "boid.hpp"

void Boid::findNeighbors(std::vector<Boid>& boids, Parameters& parameters)
{
    for (auto& boid : boids)
    {
        if (&boid != this && glm::distance(this->boid_position, boid.boid_position) < 2.0 * parameters.detection_radius)
        {
            this->neighbors.push_back(boid);
        }
    }
}

void Boid::draw(p6::Context& ctx, Parameters& parameters)
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
    ctx.circle(this->boid_position, p6::Radius{parameters.detection_radius});

    ctx.equilateral_triangle(
        p6::Center{this->boid_position},
        p6::Radius{0.08f},
        p6::Rotation{p6::Angle(this->boid_direction)}
    );
}

void Boid::checkBorders(p6::Context& ctx, Parameters& parameters)
{
    outLeft(ctx, parameters);
    outRight(ctx, parameters);
    outTop(parameters);
    outBottom(parameters);
}

void Boid::outLeft(p6::Context& ctx, Parameters& parameters)
{
    if (this->boid_position.x - parameters.detection_radius < -ctx.aspect_ratio())
    {
        this->boid_position.x = -ctx.aspect_ratio() + parameters.detection_radius;
        this->boid_direction.x *= -1;
    }
}

void Boid::outRight(p6::Context& ctx, Parameters& parameters)
{
    if (this->boid_position.x + parameters.detection_radius > ctx.aspect_ratio())
    {
        this->boid_position.x = ctx.aspect_ratio() - parameters.detection_radius;
        this->boid_direction.x *= -1;
    }
}

void Boid::outTop(Parameters& parameters)
{
    if (this->boid_position.y + parameters.detection_radius > 1)
    {
        this->boid_position.y = 1 - parameters.detection_radius;
        this->boid_direction.y *= -1;
    }
}

void Boid::outBottom(Parameters& parameters)
{
    if (this->boid_position.y - parameters.detection_radius < -1)
    {
        this->boid_position.y = -1 + parameters.detection_radius;
        this->boid_direction.y *= -1;
    }
}

void Boid::move(p6::Context& ctx, Parameters& parameters)
{
    this->boid_position += this->boid_direction * ctx.delta_time() * parameters.max_speed;
}

void Boid::update(p6::Context& ctx, std::vector<Boid>& boids, Parameters& parameters)
{
    // parameters.updateParameters();
    findNeighbors(boids, parameters);
    move(ctx, parameters);
    checkBorders(ctx, parameters);
    for (const auto& neighbor : boids)
    {
        const float distance = glm::distance(this->boid_position, neighbor.boid_position);
        Separate(neighbor, distance, parameters);
        Align(neighbor, distance, parameters);
        Cohesion(neighbor, distance, parameters);
    }

    draw(ctx, parameters);

    this->neighbors.clear();
}

void Boid::Separate(const Boid& neighbor, const float& distance, Parameters& parameters)
{
    glm::vec2 separationVector(0.0f, 0.0f);
    int       neighborCount = 0;

    if (distance != 0 && distance < parameters.detection_radius)
    {
        separationVector += (this->boid_position - neighbor.boid_position) / distance;
        neighborCount++;
    }

    if (neighborCount > 0)
    {
        separationVector /= static_cast<float>(neighborCount);
        separationVector = normalize(separationVector);
        // std::cout << separationVector.x << std::endl;
        limitSpeed(parameters);
    }

    this->boid_direction += separationVector * parameters.separation_weight;
}

void Boid::Align(const Boid& neighbor, const float& distance, Parameters& parameters)
{
    glm::vec2 alignmentVector = {0.0f, 0.0f};
    float     meanAlignment   = 0.0f;

    if (distance < parameters.detection_radius && distance != 0)
    {
        alignmentVector += neighbor.boid_direction * (1.0f / distance);
        meanAlignment += 1.0f / distance;
    }

    if (meanAlignment > 0.0f)
    {
        alignmentVector /= meanAlignment;
        alignmentVector = glm::normalize(alignmentVector);
        // std::cout << alignmentVector.x << std::endl;

        limitSpeed(parameters);
    }

    this->boid_direction += alignmentVector * parameters.alignment_weight;
}

void Boid::Cohesion(const Boid& neighbor, const float& distance, Parameters& parameters)
{
    glm::vec2 AveragePosition(0.0f, 0.0f);
    glm::vec2 cohesionDirection(0.0f, 0.0f);
    float     meanCohesion = 0.0f;

    if (distance < parameters.detection_radius && distance != 0)
    {
        AveragePosition += neighbor.boid_position * (1.0f / distance);
        meanCohesion += 1.0f / distance;
    }

    if (meanCohesion > 0)
    {
        AveragePosition /= meanCohesion;
        cohesionDirection = (AveragePosition - this->boid_position) * (parameters.cohesion_weight * 0.05f);

        limitSpeed(parameters);
    }

    this->boid_position += cohesionDirection;
}

void Boid::limitSpeed(Parameters& parameters)
{
    float speed = glm::length(this->boid_direction);
    if (speed > parameters.max_speed)
    {
        this->boid_direction = normalize(this->boid_direction) * parameters.max_speed;
    }
}
