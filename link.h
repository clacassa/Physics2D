#ifndef LINK_H
#define LINK_H

#include "rigid_body.h"


class Constraint {
public:
    Constraint() {}
    virtual ~Constraint() {}
    virtual void apply() = 0;
    virtual void draw(SDL_Renderer* renderer) = 0;
    virtual double energy() const = 0;
};

class Link : public Constraint {
public:
    Link(RigidBody* A, RigidBody* B, double dist) : a(A), b(B), distance(dist) {}
    void apply() override;
    void draw(SDL_Renderer* renderer) override;
    double energy() const override { /* stub */ return 0.0;}
    
private:
    RigidBody* a;
    RigidBody* b;
    double distance;
};

struct FixedLink {
    FixedLink(RigidBody* A, Frame* F, double dist);
    void apply(double dt);

    RigidBody* a;
    Frame* f;
    double distance;
    double theta_0;
    double pulsation;
    double theta;
    double m_dt;
};

class FixedSpring : public Constraint {
public:
    FixedSpring(RigidBody* A, Frame* f, double stiffness, double length);
    void apply() override;
    void draw(SDL_Renderer* renderer) override;
    double energy() const override;

private:
    RigidBody* a;
    Frame* f;
    double k;
    double l_0;
};

class Spring : public Constraint {
public: 
    Spring(RigidBody* A, RigidBody* B, double stiffness, double length);
    void apply() override;
    void draw(SDL_Renderer* renderer) override;
    double energy() const override;

private:
    RigidBody* a;
    RigidBody* b;
    double k;
    double l_0;
};

#endif /* LINK_H */
