#include <cmath>
#include <algorithm>
#include "broad_phase.h"
#include "rigid_body.h"

std::vector<BodyPair> SweepAndPrune::process(const std::vector<RigidBody*>& list) {
    std::vector<BodyPair> possible_collisions;
    std::vector<RigidBody*> active_intervall;

    if (m_var_x >= m_var_y) {
        sort_ascending_x(m_list);
        for (unsigned i(0); i < m_list.size(); ++i) {
            // Skip if the body is disabled
            if (!m_list[i]->is_enabled()) {
                continue;
            }

            for (unsigned j(0); j < active_intervall.size(); ++j) {
                if (m_list[i]->get_AABB().min.x > active_intervall[j]->get_AABB().max.x) {
                    active_intervall.erase(active_intervall.begin() +j);
                    --j;
                }else {
                    possible_collisions.push_back({m_list[i], active_intervall[j]});
                }
            }
            active_intervall.push_back(m_list[i]);
        }
    }else {
        sort_ascending_y(m_list);
        for (unsigned i(0); i < m_list.size(); ++i) {
            // Skip if the body is disabled
            if (!m_list[i]->is_enabled()) {
                continue;
            }
                
            for (unsigned j(0); j < active_intervall.size(); ++j) {
                if (m_list[i]->get_AABB().min.y > active_intervall[j]->get_AABB().max.y) {
                    active_intervall.erase(active_intervall.begin() +j);
                    --j;
                }else {
                    possible_collisions.push_back({m_list[i], active_intervall[j]});
                }
            }
            active_intervall.push_back(m_list[i]);
        }
    }
        
    return possible_collisions;
}

void SweepAndPrune::choose_axis(const std::vector<RigidBody*>& list) {
    m_var_x = 0;
    m_var_y = 0;

    double sum_x(0), sum_y(0);
    for (auto body : list) {
        sum_x += body->get_p().x;
        sum_y += body->get_p().y;
    }
    double mean_x(sum_x / list.size());
    double mean_y(sum_y / list.size());
    for (auto body : list) {
        m_var_x += pow(body->get_p().x - mean_x, 2);
        m_var_y += pow(body->get_p().y - mean_y, 2);
    }
}

void SweepAndPrune::sort_ascending_x(std::vector<RigidBody*>& list) {
    std::sort(list.begin(), list.end(), [=](RigidBody* a, RigidBody* b)->bool {
        return a->get_AABB().min.x < b->get_AABB().min.x;
    }); 
}

void SweepAndPrune::sort_ascending_y(std::vector<RigidBody*>& list) {
    std::sort(list.begin(), list.end(), [=](RigidBody* a, RigidBody* b)->bool {
        return a->get_AABB().min.y < b->get_AABB().min.y;
    }); 
}


bool AABB_overlap(const AABB& a, const AABB& b) {
    double d1x(b.min.x - a.max.x);
    double d1y(b.min.y - a.max.y);
    double d2x(a.min.x - b.max.x);
    double d2y(a.min.y - b.max.y);

    return !(d1x > 0.0 || d1y > 0.0 || d2x > 0.0 || d2y > 0.0);
}