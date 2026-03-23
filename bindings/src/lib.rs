// bindings/src/lib.rs
use pyo3::prelude::*;
use pyo3::types::PyList;
use nalgebra::Vector3;
use crate::agi::{AGICore, Node};

#[pyclass]
struct PyAGICore {
    core: AGICore,
}

#[pymethods]
impl PyAGICore {
    #[new]
    fn new() -> Self {
        Self {
            core: AGICore::new(),
        }
    }

    fn add_node(&mut self, id: u64, theta: f64, phi: f64, embedding: [f64; 3]) {
        let vec = Vector3::new(embedding[0], embedding[1], embedding[2]);
        self.core.add_node(Node::new(id, theta, phi, vec));
    }

    fn handover_step(&mut self, delta_theta: f64, delta_phi: f64) {
        self.core.handover_step(delta_theta, delta_phi);
    }

    fn average_syzygy(&self) -> f64 {
        self.core.average_syzygy()
    }

    fn get_satoshi(&self) -> f64 {
        self.core.satoshi
    }
}

/// Módulo Python exposto como `arkhe_agi`
#[pymodule]
fn arkhe_agi(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<PyAGICore>()?;
    Ok(())
}
