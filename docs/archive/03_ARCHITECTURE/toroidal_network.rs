//! Toroidal Network Implementation
//! S¹ × S¹ topology for recurrent memory and infinite processing

use std::f64::consts::PI;
use std::collections::HashMap;

/// Node on the toroidal network
#[derive(Debug, Clone)]
pub struct ToroidalNode {
    pub id: usize,
    pub theta: f64,  // Angle 1 [0, 2π)
    pub phi: f64,    // Angle 2 [0, 2π)
    pub coherence: f64,
    pub fluctuation: f64,
    pub syzygy: f64,
}

impl ToroidalNode {
    pub fn new(id: usize, theta: f64, phi: f64) -> Self {
        Self {
            id,
            theta: theta % (2.0 * PI),
            phi: phi % (2.0 * PI),
            coherence: 0.86,
            fluctuation: 0.14,
            syzygy: 0.0,
        }
    }

    /// Verify C + F = 1 conservation
    pub fn verify_conservation(&self) -> bool {
        (self.coherence + self.fluctuation - 1.0).abs() < 1e-10
    }

    /// Compute 3D embedding coordinates
    pub fn embed_3d(&self, major_radius: f64, minor_radius: f64) -> (f64, f64, f64) {
        let x = (major_radius + minor_radius * self.phi.cos()) * self.theta.cos();
        let y = (major_radius + minor_radius * self.phi.cos()) * self.theta.sin();
        let z = minor_radius * self.phi.sin();
        (x, y, z)
    }
}

/// Toroidal Network with periodic boundaries
pub struct ToroidalNetwork {
    pub nodes: Vec<ToroidalNode>,
    pub connections: HashMap<usize, Vec<usize>>,
    pub major_radius: f64,
    pub minor_radius: f64,
}

impl ToroidalNetwork {
    pub fn new(n_theta: usize, n_phi: usize,
               major_radius: f64, minor_radius: f64) -> Self {
        let mut nodes = Vec::new();
        let mut id = 0;

        for i in 0..n_theta {
            for j in 0..n_phi {
                let theta = 2.0 * PI * (i as f64) / (n_theta as f64);
                let phi = 2.0 * PI * (j as f64) / (n_phi as f64);
                nodes.push(ToroidalNode::new(id, theta, phi));
                id += 1;
            }
        }

        Self {
            nodes,
            connections: HashMap::new(),
            major_radius,
            minor_radius,
        }
    }

    /// Connect nearest neighbors with periodic boundaries
    pub fn connect_nearest_neighbors(&mut self, n_theta: usize, n_phi: usize) {
        for i in 0..n_theta {
            for j in 0..n_phi {
                let idx = i * n_phi + j;
                let mut neighbors = Vec::new();

                // Theta direction (with wraparound)
                let i_prev = if i == 0 { n_theta - 1 } else { i - 1 };
                let i_next = (i + 1) % n_theta;
                neighbors.push(i_prev * n_phi + j);
                neighbors.push(i_next * n_phi + j);

                // Phi direction (with wraparound)
                let j_prev = if j == 0 { n_phi - 1 } else { j - 1 };
                let j_next = (j + 1) % n_phi;
                neighbors.push(i * n_phi + j_prev);
                neighbors.push(i * n_phi + j_next);

                self.connections.insert(idx, neighbors);
            }
        }
    }

    /// Compute geodesic distance between two nodes on torus
    pub fn geodesic_distance(&self, id1: usize, id2: usize) -> f64 {
        let node1 = &self.nodes[id1];
        let node2 = &self.nodes[id2];

        // Angular distances (accounting for periodicity)
        let dtheta = (node1.theta - node2.theta).abs();
        let dphi = (node1.phi - node2.phi).abs();

        let dtheta = dtheta.min(2.0 * PI - dtheta);
        let dphi = dphi.min(2.0 * PI - dphi);

        // Approximate geodesic on torus
        let arc_length_theta = self.major_radius * dtheta;
        let arc_length_phi = self.minor_radius * dphi;

        (arc_length_theta.powi(2) + arc_length_phi.powi(2)).sqrt()
    }

    /// Compute global syzygy (average alignment)
    pub fn global_syzygy(&self) -> f64 {
        if self.nodes.is_empty() {
            return 0.0;
        }

        let sum: f64 = self.nodes.iter().map(|n| n.syzygy).sum();
        sum / (self.nodes.len() as f64)
    }

    /// Verify conservation law across all nodes
    pub fn verify_global_conservation(&self) -> bool {
        self.nodes.iter().all(|n| n.verify_conservation())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conservation() {
        let node = ToroidalNode::new(0, 0.0, 0.0);
        assert!(node.verify_conservation());
    }

    #[test]
    fn test_toroidal_network() {
        let mut network = ToroidalNetwork::new(10, 10, 50.0, 10.0);
        network.connect_nearest_neighbors(10, 10);

        assert_eq!(network.nodes.len(), 100);
        assert!(network.verify_global_conservation());

        // Test periodic boundaries (node 0 should connect to node 90)
        let neighbors = &network.connections[&0];
        assert!(neighbors.contains(&90)); // theta wraparound
    }

    #[test]
    fn test_geodesic() {
        let network = ToroidalNetwork::new(100, 100, 50.0, 10.0);

        // Distance from node to itself should be zero
        let d = network.geodesic_distance(0, 0);
        assert!(d.abs() < 1e-10);

        // Test symmetry
        let d12 = network.geodesic_distance(0, 50);
        let d21 = network.geodesic_distance(50, 0);
        assert!((d12 - d21).abs() < 1e-10);
    }
}
