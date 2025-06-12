#pragma once

#include <Eigen/Core>

struct NormalizedCCDPoly {
  float a, b, c, d;

  NormalizedCCDPoly(Eigen::Ref<const Eigen::Vector3f> x10,
                    Eigen::Ref<const Eigen::Vector3f> x20,
                    Eigen::Ref<const Eigen::Vector3f> x30,
                    Eigen::Ref<const Eigen::Vector3f> x40,
                    Eigen::Ref<const Eigen::Vector3f> x11,
                    Eigen::Ref<const Eigen::Vector3f> x21,
                    Eigen::Ref<const Eigen::Vector3f> x31,
                    Eigen::Ref<const Eigen::Vector3f> x41);
};

struct CCDPolySolution {
  int n;     // number of collision
  float t1;  // TOI 1
  float t2;  // TOI 2

  CCDPolySolution() : n(0), t1(0), t2(0) {}         // no collision
  CCDPolySolution(float t) : n(1), t1(t), t2(0) {}  // one collision
  CCDPolySolution(float t1, float t2)
      : n(2), t1(t1), t2(t2) {}  // two collision
};

class CCDPolynomialSolver {
  // ax^3 + bx^2 + cx + d
  float a_, b_, c_, d_;
  // time interval
  float t0_, t1_;

  float tol_;
  int max_iter_;
  float eps_;

  inline float eval(float x) const;
  inline float eval_derivative(float x) const;
  inline float newton(float x) const;
  inline CCDPolySolution try_clamp(float x) const;
  CCDPolySolution linear_ccd() const;
  CCDPolySolution quadratic_ccd() const;
  CCDPolySolution cubic_ccd() const;

 public:
  CCDPolySolution solve(const NormalizedCCDPoly& poly, float tol, int max_iter,
                        float eps);
  CCDPolySolution solve(float a, float b, float c, float d, float t0, float t1,
                        float tol, int max_iter, float eps);
};
