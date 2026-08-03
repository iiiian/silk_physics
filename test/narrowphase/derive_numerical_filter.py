#!/usr/bin/env python3
"""Derive the float error filters for Silk's cached TICCD graph.

All magnitudes and absolute errors are normalized by the per-axis scene bound
gamma = max(max_i(abs(x_i)), 1). The recurrence assumes IEEE-754 binary32,
round-to-nearest, gradual underflow, finite intermediates, t/u/v in [0, 1],
and minimum_separation <= gamma.

For an operation on computed values x_hat and y_hat, M is a proven bound on
the exact result and E is a bound on abs(computed - exact):

  add/sub: E' = (E_x + E_y) * (1 + unit_roundoff) + unit_roundoff * M
  multiply: propagate operand errors, then add the final rounding error

The M=2 bounds for q, s, and f use their geometric interpretation: each is a
difference between a point and a convex combination of points whose coordinate
magnitudes are at most gamma. This is why the result scales as gamma, not
gamma**3.
"""

from fractions import Fraction


UNIT_ROUNDOFF = Fraction(1, 2**24)
MACHINE_EPSILON = 2 * UNIT_ROUNDOFF


def add_error(x_error: Fraction, y_error: Fraction, magnitude: int) -> Fraction:
    pre_rounding = x_error + y_error
    return pre_rounding * (1 + UNIT_ROUNDOFF) + UNIT_ROUNDOFF * magnitude


def multiply_error(
    x_magnitude: int,
    x_error: Fraction,
    y_magnitude: int,
    y_error: Fraction,
    result_magnitude: int,
) -> Fraction:
    pre_rounding = (
        x_magnitude * y_error
        + y_magnitude * x_error
        + x_error * y_error
    )
    return (
        pre_rounding * (1 + UNIT_ROUNDOFF)
        + UNIT_ROUNDOFF * result_magnitude
    )


# c = x - y, dc = c(t1) - c(t0), p = dc * t, q = p + c(t0).
coefficient_error = add_error(Fraction(0), Fraction(0), magnitude=2)
coefficient_delta_error = add_error(
    coefficient_error, coefficient_error, magnitude=4
)
coefficient_product_error = multiply_error(
    4, coefficient_delta_error, 1, Fraction(0), result_magnitude=4
)
affine_coefficient_error = add_error(
    coefficient_product_error, coefficient_error, magnitude=2
)

# f = q0 + q1 * u + q2 * v. The partial and final exact sums both have
# magnitude at most 2 * gamma by their point-minus-convex-point meaning.
parameter_product_error = multiply_error(
    2, affine_coefficient_error, 1, Fraction(0), result_magnitude=2
)
partial_sum_error = add_error(
    affine_coefficient_error, parameter_product_error, magnitude=2
)
ee_evaluation_error = add_error(
    partial_sum_error, parameter_product_error, magnitude=2
)

# VF clipping can construct a diagonal parameter as fl(1 - u). Its absolute
# error is at most one unit roundoff because both operands lie in [0, 1].
diagonal_parameter_error = UNIT_ROUNDOFF
diagonal_product_error = multiply_error(
    2,
    affine_coefficient_error,
    1,
    diagonal_parameter_error,
    result_magnitude=2,
)
vf_evaluation_error = add_error(
    partial_sum_error, diagonal_product_error, magnitude=2
)

# The CUDA rejection path consumes the same scene filter but evaluates the
# original direct graph. Keep its coordinate-aware bounds here so changing the
# shared get_numerical_error() scaling cannot silently invalidate that path.
trajectory_delta_error = add_error(
    Fraction(0), Fraction(0), magnitude=2
)
trajectory_product_error = multiply_error(
    2, trajectory_delta_error, 1, Fraction(0), result_magnitude=2
)
trajectory_error = add_error(
    trajectory_product_error, Fraction(0), magnitude=1
)
direct_difference_error = add_error(
    trajectory_error, trajectory_error, magnitude=2
)
direct_parameter_product_error = multiply_error(
    2, direct_difference_error, 1, Fraction(0), result_magnitude=2
)
direct_edge_point_error = add_error(
    direct_parameter_product_error, trajectory_error, magnitude=1
)
cuda_ee_evaluation_error = add_error(
    direct_edge_point_error, direct_edge_point_error, magnitude=2
)
cuda_vf_triangle_sum_error = add_error(
    direct_parameter_product_error,
    direct_parameter_product_error,
    magnitude=2,
)
cuda_vf_triangle_point_error = add_error(
    cuda_vf_triangle_sum_error, trajectory_error, magnitude=1
)
cuda_vf_triangle_evaluation_error = add_error(
    trajectory_error, cuda_vf_triangle_point_error, magnitude=2
)
cuda_vf_cube_sum_error = add_error(
    direct_parameter_product_error,
    direct_parameter_product_error,
    magnitude=4,
)
cuda_vf_cube_point_error = add_error(
    cuda_vf_cube_sum_error, trajectory_error, magnitude=3
)
cuda_vf_cube_evaluation_error = add_error(
    trajectory_error, cuda_vf_cube_point_error, magnitude=4
)


def minimum_filter_in_eps(evaluation_error: Fraction) -> Fraction:
    """Include rounding in filter*gamma and filter*gamma + separation."""
    required = evaluation_error + UNIT_ROUNDOFF
    delivered_per_epsilon = (
        MACHINE_EPSILON * (1 - UNIT_ROUNDOFF) ** 2
    )
    return required / delivered_per_epsilon


ee_minimum_filter = minimum_filter_in_eps(ee_evaluation_error)
vf_minimum_filter = minimum_filter_in_eps(vf_evaluation_error)

EE_FILTER_IN_EPS = 32
VF_FILTER_IN_EPS = 34

assert Fraction(EE_FILTER_IN_EPS) > ee_minimum_filter
assert Fraction(VF_FILTER_IN_EPS) > vf_minimum_filter
assert cuda_ee_evaluation_error < ee_evaluation_error
assert cuda_vf_triangle_evaluation_error < vf_evaluation_error
assert cuda_vf_cube_evaluation_error < vf_evaluation_error


def in_eps(error: Fraction) -> float:
    return float(error / MACHINE_EPSILON)


print(f"EE evaluation bound: {in_eps(ee_evaluation_error):.12f} ε γ")
print(f"VF evaluation bound: {in_eps(vf_evaluation_error):.12f} ε γ")
print(f"EE required filter:  {float(ee_minimum_filter):.12f} ε γ")
print(f"VF required filter:  {float(vf_minimum_filter):.12f} ε γ")
print(f"EE selected filter:  {EE_FILTER_IN_EPS} ε γ")
print(f"VF selected filter:  {VF_FILTER_IN_EPS} ε γ")
print(
    "CUDA EE evaluation:  "
    f"{in_eps(cuda_ee_evaluation_error):.12f} ε γ"
)
print(
    "CUDA VF triangle:    "
    f"{in_eps(cuda_vf_triangle_evaluation_error):.12f} ε γ"
)
print(
    "CUDA VF cube:        "
    f"{in_eps(cuda_vf_cube_evaluation_error):.12f} ε γ"
)
