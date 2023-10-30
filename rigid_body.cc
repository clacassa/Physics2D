#include "rigid_body.h"


double Vector2::norm() const {
    return sqrt(x * x + y * y);
}

Vector2 Vector2::normalized() const {
    return Vector2{x / norm(), y / norm()};
}

const Vector2 Vector2::operator+(const Vector2& v) const {
    return Vector2{x + v.x, y + v.y};
}

Vector2& Vector2::operator+=(const Vector2& v) {
     *this = *this + v;
     return *this;
}

const Vector2 Vector2::operator-(const Vector2& v) const {
    return Vector2{x - v.x, y - v.y};
}

Vector2& Vector2::operator-=(const Vector2& v) {
    *this = *this - v;
    return *this;
}

const Vector2 Vector2::operator*(const double a) const {
    return Vector2{x * a, y * a};
}

const Vector2 Vector2::operator/(const double a) const {
    return Vector2{x / a, y / a};
}


double dot2(const Vector2 a, const Vector2 b) {
    return a.x * b.x + a.y * b.y;
}

double cross2(const Vector2 a, const Vector2 b) {
    return a.x * b.y - a.y * b.x;
}


RigidBody::RigidBody(double v_x, double v_y, double x, double y, double m_, double r_)//double I_)
:   a({0, 0}),
    v({v_x, v_y}),
    p({x, y}),
    f({0, 0}),
    omega(0),
    theta(0),
    torque(0),
    m(m_),
    r(r_),
    // I(I_)
    I(0.5 * m * r * r),
    // m_shape(shape),
    max_track_length(100)
{}

RigidBody::~RigidBody() {}

double RigidBody::energy() const {
    return k_energy() + p_energy();
}

double RigidBody::k_energy() const {
    const double v2(v.norm() * v.norm());
    return 0.5 * m * v2;
}

double RigidBody::p_energy() const {
    return m * 9.81 * p.y;
}

void RigidBody::draw(SDL_Renderer* renderer) {
    if (max_track_length > 0) {
        if (track.size() == max_track_length) {
            track.pop_front();
        }
        if (track.size() < max_track_length)
            track.push_back(p);

        size_t i(0);
        for (auto point : track) {
            int alpha(255 / track.size() * i);
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, alpha);
            render_body_circle(renderer, point.x, point.y, 1 / (double)RENDER_SCALE);
            ++i;
        }
    }

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    render_body_circle(renderer, p.x, p.y, r);
    double mark(0.75 * r);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    render_body_circle(renderer, p.x + mark * cos(theta), p.y + mark * sin(theta), r / 5);
}

void RigidBody::draw_forces(SDL_Renderer* renderer) const {
    // SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    // render_line(renderer, p.x, p.y, p.x + v.x / 10, p.y + v.y / 10.0);
    // SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);
    // render_line(renderer, p.x, p.y, p.x + v.x / 10.0, p.y);
}

Ball::Ball(double v_x, double v_y, double x, double y, double m_, double r_)
:   //RigidBody(v_x, v_y, x, y, m_, 0.5 * m_ * r_ * r_, Shape::Disc),
    RigidBody(v_x, v_y, x, y, m_, 0.5 * m_ * r_ * r_),
    r(r_)
{}

Polygon::Polygon(double v_x, double v_y, double x, double y, double m_, double I_)
:   RigidBody(v_x, v_y, x, y, m_, I_)
{}

Rectangle::Rectangle(double v_x, double v_y, double x, double y, double m_, double w_, double h_)
:   Polygon(v_x, v_y, x, y, m_, 1/12 * m_ * (w_ * w_ + h_ * h_))
{}

void Frame::draw(SDL_Renderer* renderer) {
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
    SDL_FRect rect({center.x - width / 2, SCENE_HEIGHT - center.y - height / 2, width, height});
    rect.x *= RENDER_SCALE;
    rect.y *= RENDER_SCALE;
    rect.w *= RENDER_SCALE;
    rect.h *= RENDER_SCALE;

    SDL_RenderDrawRectF(renderer, &rect);
}

