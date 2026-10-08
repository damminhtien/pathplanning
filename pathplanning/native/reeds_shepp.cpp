/*
 * Reeds-Shepp path families adapted from OMPL's ReedsSheppStateSpace.cpp.
 *
 * Copyright (c) 2010, Rice University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Rice University nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "reeds_shepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace pathplanning::kinodynamic {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

enum Curve { kRight = -1, kStraight = 0, kLeft = 1, kNoOp = 2 };

struct Path {
    std::array<int, 5> type{kNoOp, kNoOp, kNoOp, kNoOp, kNoOp};
    std::array<double, 5> length{kInfinity, 0.0, 0.0, 0.0, 0.0};

    double total_length() const {
        double total = 0.0;
        for (double segment : length) total += std::abs(segment);
        return total;
    }
};

constexpr std::array<std::array<int, 5>, 18> kPathTypes = {{
    {{kLeft, kRight, kLeft, kNoOp, kNoOp}},
    {{kRight, kLeft, kRight, kNoOp, kNoOp}},
    {{kLeft, kRight, kLeft, kRight, kNoOp}},
    {{kRight, kLeft, kRight, kLeft, kNoOp}},
    {{kLeft, kRight, kStraight, kLeft, kNoOp}},
    {{kRight, kLeft, kStraight, kRight, kNoOp}},
    {{kLeft, kRight, kStraight, kLeft, kNoOp}},
    {{kRight, kLeft, kStraight, kRight, kNoOp}},
    {{kLeft, kRight, kStraight, kRight, kNoOp}},
    {{kRight, kLeft, kStraight, kLeft, kNoOp}},
    {{kRight, kStraight, kRight, kLeft, kNoOp}},
    {{kLeft, kStraight, kLeft, kRight, kNoOp}},
    {{kLeft, kStraight, kRight, kNoOp, kNoOp}},
    {{kRight, kStraight, kLeft, kNoOp, kNoOp}},
    {{kLeft, kStraight, kLeft, kNoOp, kNoOp}},
    {{kRight, kStraight, kRight, kNoOp, kNoOp}},
    {{kLeft, kRight, kStraight, kLeft, kRight}},
    {{kRight, kLeft, kStraight, kRight, kLeft}},
}};

double mod2pi(double angle) {
    double value = std::fmod(angle, kTwoPi);
    if (value < -kPi) value += kTwoPi;
    else if (value > kPi) value -= kTwoPi;
    return value;
}

void polar(double x, double y, double* radius, double* angle) {
    *radius = std::hypot(x, y);
    *angle = std::atan2(y, x);
}

void tau_omega(double u, double v, double xi, double eta, double phi,
               double* tau, double* omega) {
    const double delta = mod2pi(u - v);
    const double a = std::sin(u) - std::sin(delta);
    const double b = std::cos(u) - std::cos(delta) - 1.0;
    const double first = std::atan2(eta * a - xi * b, xi * a + eta * b);
    const double second = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
    *tau = second < 0.0 ? mod2pi(first + kPi) : mod2pi(first);
    *omega = mod2pi(*tau - u + v - phi);
}

bool lp_sp_lp(double x, double y, double phi, double* t, double* u, double* v) {
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
    if (*t < -kZero) return false;
    *v = mod2pi(phi - *t);
    return *v >= -kZero;
}

bool lp_sp_rp(double x, double y, double phi, double* t, double* u, double* v) {
    double t1 = 0.0;
    double u1 = 0.0;
    polar(x + std::sin(phi), y - 1.0 - std::cos(phi), &u1, &t1);
    u1 *= u1;
    if (u1 < 4.0) return false;
    *u = std::sqrt(u1 - 4.0);
    const double theta = std::atan2(2.0, *u);
    *t = mod2pi(t1 + theta);
    *v = mod2pi(*t - phi);
    return *t >= -kZero && *v >= -kZero;
}

bool lp_rm_l(double x, double y, double phi, double* t, double* u, double* v) {
    double u1 = 0.0;
    double theta = 0.0;
    polar(x - std::sin(phi), y - 1.0 + std::cos(phi), &u1, &theta);
    if (u1 > 4.0) return false;
    *u = -2.0 * std::asin(0.25 * u1);
    *t = mod2pi(theta + 0.5 * *u + kPi);
    *v = mod2pi(phi - *t + *u);
    return *t >= -kZero && *u <= kZero;
}

bool lp_rup_lum_rm(double x, double y, double phi, double* t, double* u, double* v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
    if (rho > 1.0) return false;
    *u = std::acos(rho);
    tau_omega(*u, -*u, xi, eta, phi, t, v);
    return *t >= -kZero && *v <= kZero;
}

bool lp_rum_lum_rp(double x, double y, double phi, double* t, double* u, double* v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if (rho < 0.0 || rho > 1.0) return false;
    *u = -std::acos(rho);
    if (*u < -0.5 * kPi) return false;
    tau_omega(*u, *u, xi, eta, phi, t, v);
    return *t >= -kZero && *v >= -kZero;
}

bool lp_rm_sm_lm(double x, double y, double phi, double* t, double* u, double* v) {
    const double xi = x - std::sin(phi);
    const double eta = y - 1.0 + std::cos(phi);
    double rho = 0.0;
    double theta = 0.0;
    polar(xi, eta, &rho, &theta);
    if (rho < 2.0) return false;
    const double radius = std::sqrt(rho * rho - 4.0);
    *u = 2.0 - radius;
    *t = mod2pi(theta + std::atan2(radius, -2.0));
    *v = mod2pi(phi - 0.5 * kPi - *t);
    return *t >= -kZero && *u <= kZero && *v <= kZero;
}

bool lp_rm_sm_rm(double x, double y, double phi, double* t, double* u, double* v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    double rho = 0.0;
    double theta = 0.0;
    polar(-eta, xi, &rho, &theta);
    if (rho < 2.0) return false;
    *t = theta;
    *u = 2.0 - rho;
    *v = mod2pi(*t + 0.5 * kPi - phi);
    return *t >= -kZero && *u <= kZero && *v <= kZero;
}

bool lp_rm_slm_rp(double x, double y, double phi, double* t, double* u, double* v) {
    const double xi = x + std::sin(phi);
    const double eta = y - 1.0 - std::cos(phi);
    double rho = 0.0;
    double theta = 0.0;
    polar(xi, eta, &rho, &theta);
    if (rho < 2.0) return false;
    *u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (*u > kZero) return false;
    *t = mod2pi(std::atan2((4.0 - *u) * xi - 2.0 * eta,
                           -2.0 * xi + (*u - 4.0) * eta));
    *v = mod2pi(*t - phi);
    return *t >= -kZero && *v >= -kZero;
}

void consider(Path* best, const std::array<int, 5>& types,
              double a, double b = 0.0, double c = 0.0,
              double d = 0.0, double e = 0.0) {
    Path candidate{types, {a, b, c, d, e}};
    if (candidate.total_length() < best->total_length()) *best = candidate;
}

void csc(double x, double y, double phi, Path* path) {
    double t = 0.0, u = 0.0, v = 0.0;
    if (lp_sp_lp(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[14], t, u, v);
    if (lp_sp_lp(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[14], -t, -u, -v);
    if (lp_sp_lp(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[15], t, u, v);
    if (lp_sp_lp(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[15], -t, -u, -v);
    if (lp_sp_rp(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[12], t, u, v);
    if (lp_sp_rp(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[12], -t, -u, -v);
    if (lp_sp_rp(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[13], t, u, v);
    if (lp_sp_rp(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[13], -t, -u, -v);
}

void ccc(double x, double y, double phi, Path* path) {
    double t = 0.0, u = 0.0, v = 0.0;
    if (lp_rm_l(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[0], t, u, v);
    if (lp_rm_l(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[0], -t, -u, -v);
    if (lp_rm_l(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[1], t, u, v);
    if (lp_rm_l(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[1], -t, -u, -v);
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    if (lp_rm_l(xb, yb, phi, &t, &u, &v)) consider(path, kPathTypes[0], v, u, t);
    if (lp_rm_l(-xb, yb, -phi, &t, &u, &v)) consider(path, kPathTypes[0], -v, -u, -t);
    if (lp_rm_l(xb, -yb, -phi, &t, &u, &v)) consider(path, kPathTypes[1], v, u, t);
    if (lp_rm_l(-xb, -yb, phi, &t, &u, &v)) consider(path, kPathTypes[1], -v, -u, -t);
}

void cccc(double x, double y, double phi, Path* path) {
    double t = 0.0, u = 0.0, v = 0.0;
    if (lp_rup_lum_rm(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[2], t, u, -u, v);
    if (lp_rup_lum_rm(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[2], -t, -u, u, -v);
    if (lp_rup_lum_rm(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[3], t, u, -u, v);
    if (lp_rup_lum_rm(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[3], -t, -u, u, -v);
    if (lp_rum_lum_rp(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[2], t, u, u, v);
    if (lp_rum_lum_rp(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[2], -t, -u, -u, -v);
    if (lp_rum_lum_rp(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[3], t, u, u, v);
    if (lp_rum_lum_rp(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[3], -t, -u, -u, -v);
}

void ccsc(double x, double y, double phi, Path* path) {
    double t = 0.0, u = 0.0, v = 0.0;
    if (lp_rm_sm_lm(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[4], t, -0.5 * kPi, u, v);
    if (lp_rm_sm_lm(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[4], -t, 0.5 * kPi, -u, -v);
    if (lp_rm_sm_lm(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[5], t, -0.5 * kPi, u, v);
    if (lp_rm_sm_lm(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[5], -t, 0.5 * kPi, -u, -v);
    if (lp_rm_sm_rm(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[8], t, -0.5 * kPi, u, v);
    if (lp_rm_sm_rm(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[8], -t, 0.5 * kPi, -u, -v);
    if (lp_rm_sm_rm(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[9], t, -0.5 * kPi, u, v);
    if (lp_rm_sm_rm(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[9], -t, 0.5 * kPi, -u, -v);
    const double xb = x * std::cos(phi) + y * std::sin(phi);
    const double yb = x * std::sin(phi) - y * std::cos(phi);
    if (lp_rm_sm_lm(xb, yb, phi, &t, &u, &v)) consider(path, kPathTypes[6], v, u, -0.5 * kPi, t);
    if (lp_rm_sm_lm(-xb, yb, -phi, &t, &u, &v)) consider(path, kPathTypes[6], -v, -u, 0.5 * kPi, -t);
    if (lp_rm_sm_lm(xb, -yb, -phi, &t, &u, &v)) consider(path, kPathTypes[7], v, u, -0.5 * kPi, t);
    if (lp_rm_sm_lm(-xb, -yb, phi, &t, &u, &v)) consider(path, kPathTypes[7], -v, -u, 0.5 * kPi, -t);
    if (lp_rm_sm_rm(xb, yb, phi, &t, &u, &v)) consider(path, kPathTypes[10], v, u, -0.5 * kPi, t);
    if (lp_rm_sm_rm(-xb, yb, -phi, &t, &u, &v)) consider(path, kPathTypes[10], -v, -u, 0.5 * kPi, -t);
    if (lp_rm_sm_rm(xb, -yb, -phi, &t, &u, &v)) consider(path, kPathTypes[11], v, u, -0.5 * kPi, t);
    if (lp_rm_sm_rm(-xb, -yb, phi, &t, &u, &v)) consider(path, kPathTypes[11], -v, -u, 0.5 * kPi, -t);
}

void ccsc_csc(double x, double y, double phi, Path* path) {
    double t = 0.0, u = 0.0, v = 0.0;
    if (lp_rm_slm_rp(x, y, phi, &t, &u, &v)) consider(path, kPathTypes[16], t, -0.5 * kPi, u, -0.5 * kPi, v);
    if (lp_rm_slm_rp(-x, y, -phi, &t, &u, &v)) consider(path, kPathTypes[16], -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
    if (lp_rm_slm_rp(x, -y, -phi, &t, &u, &v)) consider(path, kPathTypes[17], t, -0.5 * kPi, u, -0.5 * kPi, v);
    if (lp_rm_slm_rp(-x, -y, phi, &t, &u, &v)) consider(path, kPathTypes[17], -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
}

Path get_path(double x, double y, double phi) {
    Path path;
    csc(x, y, phi, &path);
    ccc(x, y, phi, &path);
    cccc(x, y, phi, &path);
    ccsc(x, y, phi, &path);
    ccsc_csc(x, y, phi, &path);
    return path;
}

}  // namespace

bool shortest_reeds_shepp_path(const std::array<double, 3>& start,
                               const std::array<double, 3>& goal,
                               double turning_radius,
                               std::vector<ReedsSheppSegment>* output) {
    if (output == nullptr || !std::isfinite(turning_radius) || turning_radius <= 0.0) return false;
    output->clear();
    const double dx = goal[0] - start[0];
    const double dy = goal[1] - start[1];
    const double cosine = std::cos(start[2]);
    const double sine = std::sin(start[2]);
    const double x = (dx * cosine + dy * sine) / turning_radius;
    const double y = (-dx * sine + dy * cosine) / turning_radius;
    const double phi = mod2pi(goal[2] - start[2]);
    const Path path = get_path(x, y, phi);
    if (!std::isfinite(path.total_length())) return false;
    for (size_t index = 0; index < path.length.size(); ++index) {
        if (path.type[index] == kNoOp || std::abs(path.length[index]) <= 1e-12) continue;
        const int gear = path.length[index] < 0.0 ? -1 : 1;
        output->push_back({path.type[index], gear, std::abs(path.length[index]) * turning_radius});
    }
    return !output->empty();
}

}  // namespace pathplanning::kinodynamic
