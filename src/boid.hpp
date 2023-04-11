#pragma once
#include <stdlib.h>
#include <cmath>
#include <vector>
#include "doctest/doctest.h"
#include "p6/p6.h"

class Boid {
private:
    glm::vec2 boid_position;    // Boid's position
    glm::vec2 boid_direction;   // Boid's direction
    float     max_speed;        // Maximum speed of the boid
    float     detection_radius; // Radius which boid can detect neighbors

    // parameters used for the rules of boids
    float separation_weight; // weight for separation rule
    float alignment_weight;  // weight for alignment rule
    float cohesion_weight;   // weight for cohesion rule

    std::vector<Boid> neighbors; // Vector of neighbors for a boid

public:
    // Constructor
    Boid(glm::vec2 position, glm::vec2 direction)
        : boid_position(position), boid_direction(direction){};

    // Draws the boids on the canvas
    void draw(p6::Context& ctx);

    // Updates the boid's position based on its neighbors and environment
    void update(p6::Context& ctx, std::vector<Boid>& boids);

    // Rules fonctions
    void Alignment(const std::vector<Boid>& neighbors);
    void Cohesion(const std::vector<Boid>& neighbors);
    void Separation(const std::vector<Boid>& neighbors);

    // Checks if the boid is within the canvas boundaries and adjusts its direction if needed
    void checkBorders(p6::Context& ctx);
    void outRight(p6::Context& ctx);
    void outBottom();
    void outTop();
    void outLeft(p6::Context& ctx);

    // Limits the boid's speed
    void limitSpeed();

    // Finds the boid's neighbors within a certain radius
    void findNeighbors(std::vector<Boid>& boids);

    // Moves the boid in its current direction
    void move(p6::Context& ctx);

    // Setter Functions

    void setDetectionRadius(const float& radius) { this->detection_radius = radius; }
    void setMaxSpeed(float speed) { this->max_speed = speed; }

    void setAlignment(const float& alignment) { this->alignment_weight = alignment; }
    void setCohesion(const float& cohesion) { this->cohesion_weight = cohesion; }
    void setSeparation(const float& avoid) { this->separation_weight = avoid; }
};
