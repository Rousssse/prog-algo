#pragma once
#include <vcruntime.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <random>
#include <vector>
#include "doctest/doctest.h"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "p6/p6.h"

struct ParametersAttributes {
    float radius;
    float speed;
    float weight_alignment;
    float weight_cohesion;
    float weight_separation;
};

struct Parameters {
private:
    float detection_radius;
    float max_speed;
    float alignment_weight;  // weight for alignment rule
    float cohesion_weight;   // weight for cohesion rule
    float separation_weight; // weight for separation rule

public:
    explicit Parameters(ParametersAttributes attributes)
        : detection_radius(attributes.radius), max_speed(attributes.speed), alignment_weight(attributes.weight_alignment), cohesion_weight(attributes.weight_cohesion), separation_weight(attributes.weight_separation){};

    void updateParameters()
    {
        ImGui::Begin("Choose your values");
        ImGui::SliderFloat("Detection radius", &detection_radius, 0.f, 1.f);
        ImGui::SliderFloat("Max Speed", &max_speed, 0.0f, 2.f);
        ImGui::SliderFloat("Alignment", &alignment_weight, 0.f, 1.f);
        ImGui::SliderFloat("Cohesion", &cohesion_weight, 0.f, 0.1f);
        ImGui::SliderFloat("Separation", &separation_weight, 0.f, 1.f);
        ImGui::End();
    }

    float getDetectionRadius() const
    {
        return detection_radius;
    }
    float getMaxSpeed() const
    {
        return max_speed;
    }
    float getAlignmentWeight() const
    {
        return alignment_weight;
    }
    float getCohesionWeight() const
    {
        return cohesion_weight;
    }
    float getSeparationWeight() const
    {
        return separation_weight;
    }
};

struct BoidsAttributes {
    glm::vec2 position;
    glm::vec2 direction;
};

class Boid {
private:
    glm::vec2 boid_position;  // Boid's position
    glm::vec2 boid_direction; // Boid's direction

    std::vector<Boid> neighbors; // Vector of neighbors for a boid

    // Draws the boids on the canvas
    void draw(p6::Context& ctx, Parameters& parameters);

    // Moves the boid in its current direction
    void move(p6::Context& ctx, Parameters& parameters);

    // Rules function
    void Align(const Boid& neighbor, const float& distance, Parameters& parameters);
    void Cohesion(const Boid& neighbor, const float& distance, Parameters& parameters);
    void Separate(const Boid& neighbor, const float& distance, Parameters& parameters);

    // Limits the boid's speed
    void limitSpeed(Parameters& parameters);

    // Checks if the boid is within the canvas boundaries and adjusts its direction if needed
    void checkBorders(p6::Context& ctx, Parameters& parameters);
    void outRight(p6::Context& ctx, Parameters& parameters);
    void outBottom(Parameters& parameters);
    void outTop(Parameters& parameters);
    void outLeft(p6::Context& ctx, Parameters& parameters);

public:
    // Constructor
    explicit Boid(BoidsAttributes attributes)
        : boid_position(attributes.position), boid_direction(attributes.position){};

    // Updates the boid's position based on its neighbors and environment
    void update(p6::Context& ctx, std::vector<Boid>& boids, Parameters& parameters);

    // Finds the boid's neighbors within a certain radius
    void findNeighbors(std::vector<Boid>& boids, Parameters& parameters);
};
