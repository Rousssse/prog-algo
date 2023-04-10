#pragma once
#include <stdlib.h>
#include <cmath>
#include <vector>
#include "doctest/doctest.h"
#include "p6/p6.h"

class Boid {
private:
    glm::vec2 position;
    glm::vec2 direction;

    float             max_speed;
    std::vector<Boid> neighbors;
    float             detection_radius;

    // global rules
    float avoidance;
    float alignment;

    // bordures
    float border_strength = 0.1f;
    float border_width    = 0.2f;

public:
    Boid(glm::vec2 pos, glm::vec2 dir)
        : position(pos), direction(dir){};

    // Draws the boid on the canvas
    void draw(p6::Context& ctx);

    // Updates the boid's position based on its neighbors and environment
    void update(p6::Context& ctx, std::vector<Boid>& boids);

    // Update the boid's new direction based on its neighbors
    void UpdateDirection();

    // Fonction between boids
    void Separation(const std::vector<Boid>& neighbors);
    void Cohesion(std::vector<Boid>& neighbors);
    void Alignment(const std::vector<Boid>& neighbors);

    // Checks if the boid is within the canvas boundaries and adjusts its direction if needed
    void checkBorders(p6::Context& ctx);
    void outRight(p6::Context& ctx);
    void outBottom();
    void outTop();
    void outLeft(p6::Context& ctx);

    // Finds the boid's neighbors within a certain radius
    void findNeighbors(std::vector<Boid>& boids);

    // Moves the boid in its current direction
    void move(p6::Context& ctx);

    // Sets the boid's independence (i.e. how much it follows its neighbors)
    void setAlignment(const float& align)
    {
        this->alignment = align;
    }

    // Sets the radius within which the boid can detect neighbors
    void setDetectionRadius(const float& radius)
    {
        this->detection_radius = radius;
    }

    // avoid collision with other boids
    void setSeparation(const float& avoid)
    {
        this->avoidance = avoid;
    }

    // Sets the boid's direction
    void setDirection(glm::vec2 dir)
    {
        this->direction = dir;
    }

    // Gets the boid's position
    glm::vec2 getPosition()
    {
        return this->position;
    }

    // Gets the boid's maximum speed
    float getSpeed() const
    {
        return this->max_speed;
    }

    // Sets the boid's position
    void setPosition(const glm::vec2& pos)
    {
        this->position = pos;
    }

    // Sets the boid's maximum speed
    void setSpeed(float speed)
    {
        this->max_speed = speed;
    }
};
