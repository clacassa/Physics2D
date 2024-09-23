#include "integrator.h"
#include "rigid_body.h"

static double func1(double X2) {
    return X2;
}

static double func2(double u) {
    return u;
}

void rk4_step(RigidBody* body, double dt) {
    const unsigned N(6);
    double X[N+1];
    double U[N/2];

    const double a1(0.1666);
    const double a23(0.3333);
    const double a4(a1);

    double K1[N/2], K2[N/2], K3[N/2], K4[N/2];
    double L1[N/2], L2[N/2], L3[N/2], L4[N/2];

    X[1] = body->get_p().x;
    X[2] = body->get_v().x;
    X[3] = body->get_p().y;
    X[4] = body->get_v().y;
    X[5] = body->get_theta();
    X[6] = body->get_omega();

    U[0] = body->get_a().x;
    U[1] = body->get_a().y;
    U[2] = body->get_a_theta();

    // x, vx
    K1[0] = dt * X[2];
    L1[0] = dt * U[0];

    K2[0] = dt * (X[2] + 0.5 * L1[0]);
    L2[0] = dt * U[0];

    K3[0] = dt * (X[2] + 0.5 * L2[0]);
    L3[0] = dt * U[0];

    K4[0] = dt * (X[2] + L3[0]);
    L4[0] = dt * U[0];

    // y, vy
    K1[1] = dt * X[4];
    L1[1] = dt * U[1];

    K2[1] = dt * (X[4] + 0.5 * L1[1]);
    L2[1] = dt * U[1];

    K3[1] = dt * (X[4] + 0.5 * L2[1]);
    L3[1] = dt * U[1];

    K4[1] = dt * (X[4] + L3[1]);
    L4[1] = dt * U[1];

    // theta, omega
    K1[2] = dt * X[6];
    L1[2] = dt * U[2];

    K2[2] = dt * (X[6] + 0.5 * L1[2]);
    L2[2] = dt * U[2];

    K3[2] = dt * (X[6] + 0.5 * L2[2]);
    L3[2] = dt * U[2];

    K4[2] = dt * (X[6] + L3[2]);
    L4[2] = dt * U[2];

    // Update states
    X[1] = a1 * (K1[0] + K4[0]) + a23 * (K2[0] + K3[0]);
    X[2] = a1 * (L1[0] + L4[0]) + a23 * (L2[0] + L3[0]);
    X[3] = a1 * (K1[1] + K4[1]) + a23 * (K2[1] + K3[1]);
    X[4] = a1 * (L1[1] + L4[1]) + a23 * (L2[1] + L3[1]);
    X[5] = a1 * (K1[2] + K4[2]) + a23 * (K2[2] + K3[2]);
    X[6] = a1 * (L1[2] + L4[2]) + a23 * (L2[2] + L3[2]);

    // Apply to the body
    body->move(Vector2(X[1], X[3]));
    body->linear_impulse(Vector2(X[2], X[4]));
    body->rotate(X[5]);
    body->angular_impulse(X[6]);
}
