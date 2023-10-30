#ifndef ODE_SOLVER_H
#define ODE_SOLVER_H

#include "rigid_body.h"


class ODESolver {
public:
    ODESolver() : m_dt(0.0) {}
    virtual ~ODESolver() {}

    virtual void start(double dt);
    virtual bool step(double* dt) { return true; }
    virtual void solve(RigidBody* body, double* dt) { return; }

protected:
    double m_dt;
};

class EulerSolver : public ODESolver {
public:
    EulerSolver() {}
    virtual ~EulerSolver() {}

    virtual bool step(double* dt) override;
    virtual void solve(RigidBody* body, double* dt) override;
};

#endif /* ODE_SOLVER_H */
