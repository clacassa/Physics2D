#include <iostream>
#include "ODE_solver.h"

void ODESolver::start(double dt) {
    m_dt = dt;
}

bool EulerSolver::step(double* dt) {
    *dt = m_dt;
    return true;
}

void EulerSolver::solve(RigidBody* body, double* dt) {
    *dt = m_dt;

    body->p += body->v * m_dt;
    body->v += body->a * m_dt;

    body->theta += body->omega * m_dt;
    body->omega += body->a_theta * m_dt;
}
