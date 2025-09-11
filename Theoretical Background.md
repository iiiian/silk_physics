# Theoretical Background

Don't worry about all these theoretical stuff too much. I know it's very overwhelming but I assure you we can implement our cuda solver without knowing, like, 99% of these stuff.

## The Core Framework - Projective Dynamics

There are countless algorithms to solve a physical system (think of cloth, softbody, etc..), but in the most general form all of them are trying to solve a minimization problem.

$$
\min_{x} E(x)
$$

$E$ here is the total energy of the system. In my solver, it's a combination of momentum energy, bending energy, elastic energy, and collision energy. $E$ is often non-trivial in its most precise form and is super expensive to solve, so some engine trades accuracy with performance by approximating $E$ using simpler functions.

The core algorithm of my engine -- **Projective Dynamics** -- does exactly that. It uses a two-step process to approximate $E$ with a quadratic function so it can be solved efficiently.

> [!NOTE] Projective Dynamics: Fusing Constraint Projections for Fast Simulation
> S. Bouaziz, S. Martin, T. Liu, L. Kavan, and M. Pauly, “Projective dynamics: fusing constraint projections for fast simulation,” ACM Trans. Graph., vol. 33, no. 4, p. 154:1-154:11, July 2014, doi: 10.1145/2601097.2601116.

**Key takeaways from this paper:**
- basic implicit Euler formulation
- local/global process
	- local projection through SVD decomposition
	- global matrix solve

This paper kind of glosses over the mathematical formulation of elastic and bending energy. The derivation of them is mathematically engaging and I can't remember all of them either. If you want to dig into that, I recommend 2 papers:

1. C. Jiang, C. Schroeder, J. Teran, A. Stomakhin, and A. Selle, “The material point method for simulating continuum materials,” in _ACM SIGGRAPH 2016 Courses_, Anaheim California: ACM, July 2016, pp. 1–52. doi: [10.1145/2897826.2927348](https://doi.org/10.1145/2897826.2927348).
2. R. Bridson, R. Fedkiw, and J. Anderson, “Robust treatment of collisions, contact and friction for cloth animation,” _ACM Trans. Graph._, vol. 21, no. 3, pp. 594–603, July 2002, doi: [10.1145/566654.566623](https://doi.org/10.1145/566654.566623). R. Tamstorf and E. Grinspun, “Discrete bending forces and their Jacobians,” _Graphical Models_, vol. 75, no. 6, pp. 362–370, Nov. 2013, doi: [10.1016/j.gmod.2013.07.001](https://doi.org/10.1016/j.gmod.2013.07.001).

The first one is not related to projective dynamics but its introduction section of Continuum Mechanics is impressively well-written and should explain the elastic part. As for bending energy, the second paper walks you through it step by step. These two are completely optional.

### When Collision Meets Projective Dynamics

All physics engines need convincing collision response, but doing it robustly is **incredibly hard**. This is especially true for cloth simulation since any penetration is obvious.

> [!NOTE] Penetration-free projective dynamics on the GPU
> L. Lan, G. Ma, Y. Yang, C. Zheng, M. Li, and C. Jiang, “Penetration-free projective dynamics on the GPU,” _ACM Trans. Graph._, vol. 41, no. 4, p. 69:1-69:16, July 2022, doi: [10.1145/3528223.3530069](https://doi.org/10.1145/3528223.3530069).

This paper expands on projective dynamics and propose a performant pipeline to detect and solve colliding cloth in GPU. ==This paper is extremely important as we will be implementing its A-jacobi GPU solver (Chap 5).==

**Key takeaways from this paper:**
- Nested loop structure
- barrier constraint is just the reflection
- GPU based A-jacobi solver

### The Full Collision Pipeline

The collision pipeline of my engine is composed of 3 main stages:

- broad-phase culling using KD tree
- narrow-phase continuous collision detection using tight inclusion
- compute collision reflection as response

I recommend 3 papers for this topic:

1. R. Bridson, R. Fedkiw, and J. Anderson, “Robust treatment of collisions, contact and friction for cloth animation,” _ACM Trans. Graph._, vol. 21, no. 3, pp. 594–603, July 2002, doi: [10.1145/566654.566623](https://doi.org/10.1145/566654.566623).
2. Y. R. Serpa and M. A. F. Rodrigues, “Flexible Use of Temporal and Spatial Reasoning for Fast and Scalable CPU Broad-Phase Collision Detection Using KD-Trees,” _Computer Graphics Forum_, vol. 38, no. 1, pp. 260–273, 2019, doi: [10.1111/cgf.13529](https://doi.org/10.1111/cgf.13529).
3. B. Wang, Z. Ferguson, T. Schneider, X. Jiang, M. Attene, and D. Panozzo, “A Large-scale Benchmark and an Inclusion-based Algorithm for Continuous Collision Detection,” _ACM Trans. Graph._, vol. 40, no. 5, p. 188:1-188:16, Sept. 2021, doi: [10.1145/3460775](https://doi.org/10.1145/3460775).

The first paper introduces you to the general structure of collision pipeline and derive the formula for exact collision detection and response. The second one is the spatial partitioning structure I implemented for my CPU broadphase culling. The third one explains how to do CCD (continuous collision detection) robustly.

You could try to port the collision pipeline to the GPU too if you are interested. But this is optional because I don't think we have that much time.
