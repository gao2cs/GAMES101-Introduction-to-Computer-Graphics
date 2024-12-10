#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        double h = (end - start).norm() / (num_nodes - 1);
        Vector2D dir = (end - start) / (end - start).norm();
     
        for (int i = 0; i < num_nodes; ++i) {
            Mass* mass = new Mass(start + i * h * dir, node_mass, false);
            this->masses.push_back(mass);
        }

        for (int i = 0; i < num_nodes - 1; ++i) {
            Spring* spring = new Spring(masses[i], masses[i + 1], k);
            springs.push_back(spring);
        }

        // Comment-in this part when you implement the constructor
        for (auto& i : pinned_nodes) {
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Mass* a = s->m1;
            Mass* b = s->m2;
            
            double ks = s->k;
            double lres = s->rest_length;

            // Tips: Hold the endpoints a & b and stretch.
            b->forces += -ks * (b->position - a->position) / (b->position - a->position).norm() * ((b->position - a->position).norm() - lres);
            a->forces += ks * (b->position - a->position) / (b->position - a->position).norm() * ((b->position - a->position).norm() - lres);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                Vector2D Fg = m->mass * gravity;
                m->forces += Fg;

                // TODO (Part 2): Add global damping
                Vector2D at = m->forces / m->mass;
                Vector2D vold = m->velocity;
                m->velocity += at * delta_t;
                // m->position += vold * delta_t;     // Explicit Euler: Unstable
                m->position += m->velocity * delta_t; // Semi-Implicit Euler: Explicit Euler for updating velocity & Backward Euler for updating position. 

            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Mass* a = s->m1;
            Mass* b = s->m2;

            double ks = s->k;
            double lres = s->rest_length;

            // Tips: Hold the endpoints a & b and stretch.
            b->forces += -ks * (b->position - a->position) / (b->position - a->position).norm() * ((b->position - a->position).norm() - lres);
            a->forces += ks * (b->position - a->position) / (b->position - a->position).norm() * ((b->position - a->position).norm() - lres);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D Fg = m->mass * gravity;
                m->forces += Fg;

                Vector2D at = m->forces / m->mass;
                // m->position += (m->position - m->last_position) + at * delta_t * delta_t;
     
                // TODO (Part 4): Add global Verlet damping
                double kd = 0.00005;
                m->position += (1- kd) * (m->position - m->last_position) + at * delta_t * delta_t;
                m->last_position = temp_position;

            }
            m->forces = Vector2D(0, 0);
        }
    }
}
