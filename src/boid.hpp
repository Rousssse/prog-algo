#pragma once
#include <math.h>
#include <stdlib.h>
#include <vcruntime.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <random>
#include <vector>
#include "doctest/doctest.h"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "p6/p6.h"

struct Parameters {
    float detection_radius;
    float max_speed;
    float alignment_weight;  // weight for alignment rule
    float cohesion_weight;   // weight for cohesion rule
    float separation_weight; // weight for separation rule

    Parameters(const float radius, const float speed, const float alignment, const float cohesion, const float separation)
        : detection_radius(radius), max_speed(speed), alignment_weight(alignment), cohesion_weight(cohesion), separation_weight(separation)
    {
    }

    void updateParameters()
    {
        ImGui::Begin("Choose your values");
        ImGui::SliderFloat("Detection radius", &this->detection_radius, 0.f, 3.f);
        ImGui::SliderFloat("Max Speed", &this->max_speed, 0.0f, 2.f);
        ImGui::SliderFloat("Alignment", &this->alignment_weight, 0.f, 1.f);
        ImGui::SliderFloat("Cohesion", &this->cohesion_weight, 0.f, 0.1f);
        ImGui::SliderFloat("Separation", &this->separation_weight, 0.f, 1.f);
        ImGui::End();
    }
};

class Boid {
private:
    glm::vec2 boid_position;  // Boid's position
    glm::vec2 boid_direction; // Boid's direction
    // float     max_speed        = 0.5f; // Maximum speed of the boid
    // float     detection_radius = 0.5f; // Radius which boid can detect neighbors

    std::vector<Boid> neighbors; // Vector of neighbors for a boid

public:
    // Constructor
    Boid(glm::vec2 position, glm::vec2 direction)
        : boid_position(position), boid_direction(direction){};

    // Draws the boids on the canvas
    void draw(p6::Context& ctx, Parameters& parameters);

    // Updates the boid's position based on its neighbors and environment
    void update(p6::Context& ctx, std::vector<Boid>& boids, Parameters& parameters);

    // Rules fonctions
    void Align(const Boid& neighbor, const float& distance, Parameters& parameters);
    void Cohesion(const Boid& neighbor, const float& distance, Parameters& parameters);
    void Separate(const Boid& neighbor, const float& distance, Parameters& parameters);

    // Checks if the boid is within the canvas boundaries and adjusts its direction if needed
    void checkBorders(p6::Context& ctx, Parameters& parameters);
    void outRight(p6::Context& ctx, Parameters& parameters);
    void outBottom(Parameters& parameters);
    void outTop(Parameters& parameters);
    void outLeft(p6::Context& ctx, Parameters& parameters);

    // Limits the boid's speed
    void limitSpeed(Parameters& parameters);

    // Finds the boid's neighbors within a certain radius
    void findNeighbors(std::vector<Boid>& boids, Parameters& parameters);

    // Moves the boid in its current direction
    void move(p6::Context& ctx, Parameters& parameters);

    // void setDetectionRadius(const float& radius) { this->detection_radius = radius; }
    // void setMaxSpeed(float speed) { this->max_speed = speed; }
};
