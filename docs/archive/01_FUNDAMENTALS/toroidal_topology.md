# Toroidal Topology: S¹ × S¹

## Why Torus?

The 2-torus $T^2 = S^1 \times S^1$ is the fundamental geometry for:
- Recurrent memory systems
- Non-singular vector fields
- Periodic boundary conditions
- Action-angle variables in integrable systems

## Mathematical Properties

### 1. Periodic Boundaries
A trajectory exiting at $(x, y) = (L, y_0)$ reemerges at $(0, y_0)$.

### 2. Non-Singular Vector Fields (Hairy Ball Theorem)
Unlike $S^2$, the torus admits continuous, non-zero tangent vector fields everywhere.

### 3. Geodesics
Geodesics on $T^2$ embedded in $\mathbb{R}^3$ include helices—optimal paths.

## Coordinate System

$$\begin{aligned}
x &= (R + r\cos\phi)\cos\theta \\
y &= (R + r\cos\phi)\sin\theta \\
z &= r\sin\phi
\end{aligned}$$

Where:
- $R$ = major radius
- $r$ = minor radius
- $\theta, \phi \in [0, 2\pi)$ = angular coordinates

## Applications

- **Viterbi Algorithm**: Circular buffers (trellis on torus)
- **RNNs**: Gradient circulation without boundary dissipation
- **Phase Space**: Integrable systems live on invariant tori
- **Arkhe Network**: Handover chains form toroidal geodesics
