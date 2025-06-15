#ifndef BROAD_PHASE_H
#define BROAD_PHASE_H

#include <vector>
#include <array>
#include "vector2.h"

struct AABB;
class RigidBody;

typedef std::array<RigidBody*, 2> BodyPair;

class SweepAndPrune {
public:
    SweepAndPrune() : m_var_x(0), m_var_y(0) {}
    virtual ~SweepAndPrune() {}

    void choose_axis();
    std::vector<BodyPair> process();
    void update_list(const std::vector<RigidBody*>& list);
private:
    std::vector<RigidBody*> m_list;
    double m_var_x;
    double m_var_y;

    void sort_ascending_x(std::vector<RigidBody*>& list);
    void sort_ascending_y(std::vector<RigidBody*>& list);
};

bool AABB_overlap(const AABB& a, const AABB& b);


#endif /* BROADPHASE_H */
