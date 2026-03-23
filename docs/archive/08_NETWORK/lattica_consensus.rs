//! Lattica Consensus Algorithm
//! Proof-of-Coherence voting for distributed Arkhe network

use std::collections::{HashMap, HashSet};
use std::time::{SystemTime, UNIX_EPOCH};
use sha2::{Sha256, Digest};

/// Handover block in the Arkhe network
#[derive(Debug, Clone)]
pub struct HandoverBlock {
    pub id: u64,
    pub timestamp: u64,
    pub previous_hash: String,
    pub current_hash: String,
    pub syzygy: f64,
    pub coherence: f64,
    pub fluctuation: f64,
    pub proposer_id: String,
    pub signatures: Vec<ValidatorSignature>,
}

impl HandoverBlock {
    pub fn new(id: u64, syzygy: f64, proposer_id: String, previous_hash: String) -> Self {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();

        let coherence = 0.86;
        let fluctuation = 0.14;

        let mut block = Self {
            id,
            timestamp,
            previous_hash: previous_hash.clone(),
            current_hash: String::new(),
            syzygy,
            coherence,
            fluctuation,
            proposer_id,
            signatures: Vec::new(),
        };

        block.current_hash = block.compute_hash();
        block
    }

    fn compute_hash(&self) -> String {
        let data = format!(
            "{}{}{}{}{}{}",
            self.id, self.timestamp, self.previous_hash,
            self.syzygy, self.coherence, self.proposer_id
        );

        let mut hasher = Sha256::new();
        hasher.update(data.as_bytes());
        format!("{:x}", hasher.finalize())
    }

    pub fn verify_conservation(&self) -> bool {
        (self.coherence + self.fluctuation - 1.0).abs() < 1e-10
    }
}

/// Validator signature with proof-of-coherence
#[derive(Debug, Clone)]
pub struct ValidatorSignature {
    pub validator_id: String,
    pub validator_syzygy: f64,
    pub signature: String,
    pub timestamp: u64,
}

/// Lattica consensus engine
pub struct LatticaConsensus {
    validators: HashMap<String, Validator>,
    min_validators: usize,
    syzygy_threshold: f64,
    pending_blocks: Vec<HandoverBlock>,
    confirmed_blocks: Vec<HandoverBlock>,
}

#[derive(Debug, Clone)]
pub struct Validator {
    pub id: String,
    pub syzygy: f64,
    pub stake: u64,  // Weight in voting
    pub active: bool,
}

impl Validator {
    pub fn is_eligible(&self, threshold: f64) -> bool {
        self.active && self.syzygy >= threshold
    }
}

impl LatticaConsensus {
    pub fn new(min_validators: usize, syzygy_threshold: f64) -> Self {
        Self {
            validators: HashMap::new(),
            min_validators,
            syzygy_threshold,
            pending_blocks: Vec::new(),
            confirmed_blocks: Vec::new(),
        }
    }

    /// Register a new validator
    pub fn register_validator(&mut self, id: String, syzygy: f64, stake: u64) -> Result<(), String> {
        if syzygy < self.syzygy_threshold {
            return Err(format!(
                "Syzygy {} below threshold {}",
                syzygy, self.syzygy_threshold
            ));
        }

        let validator = Validator {
            id: id.clone(),
            syzygy,
            stake,
            active: true,
        };

        self.validators.insert(id, validator);
        Ok(())
    }

    /// Get eligible validators for current round
    fn get_eligible_validators(&self) -> Vec<&Validator> {
        self.validators
            .values()
            .filter(|v| v.is_eligible(self.syzygy_threshold))
            .collect()
    }

    /// Propose a new block
    pub fn propose_block(&mut self,
                        syzygy: f64,
                        proposer_id: String) -> Result<HandoverBlock, String> {

        // Verify proposer is eligible
        if let Some(validator) = self.validators.get(&proposer_id) {
            if !validator.is_eligible(self.syzygy_threshold) {
                return Err("Proposer not eligible".to_string());
            }
        } else {
            return Err("Proposer not registered".to_string());
        }

        let previous_hash = self.confirmed_blocks
            .last()
            .map(|b| b.current_hash.clone())
            .unwrap_or_else(|| "genesis".to_string());

        let block_id = self.confirmed_blocks.len() as u64 + 1;

        let block = HandoverBlock::new(block_id, syzygy, proposer_id, previous_hash);

        if !block.verify_conservation() {
            return Err("Block violates C+F=1".to_string());
        }

        self.pending_blocks.push(block.clone());
        Ok(block)
    }

    /// Validator votes on a block
    pub fn vote(&mut self,
               block_id: u64,
               validator_id: String,
               approve: bool) -> Result<(), String> {

        let validator = self.validators
            .get(&validator_id)
            .ok_or("Validator not found")?;

        if !validator.is_eligible(self.syzygy_threshold) {
            return Err("Validator not eligible".to_string());
        }

        let block = self.pending_blocks
            .iter_mut()
            .find(|b| b.id == block_id)
            .ok_or("Block not found")?;

        if approve {
            let signature = ValidatorSignature {
                validator_id: validator_id.clone(),
                validator_syzygy: validator.syzygy,
                signature: format!("sig_{}", validator_id),
                timestamp: SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs(),
            };

            block.signatures.push(signature);
        }

        Ok(())
    }

    /// Check if block has reached consensus
    pub fn check_consensus(&mut self, block_id: u64) -> Result<bool, String> {
        let block = self.pending_blocks
            .iter()
            .find(|b| b.id == block_id)
            .ok_or("Block not found")?;

        let eligible = self.get_eligible_validators();

        if eligible.len() < self.min_validators {
            return Ok(false);
        }

        // Compute weighted votes
        let total_stake: u64 = eligible.iter().map(|v| v.stake).sum();
        let voted_stake: u64 = block.signatures
            .iter()
            .filter_map(|sig| {
                self.validators.get(&sig.validator_id).map(|v| v.stake)
            })
            .sum();

        // Require 2/3 majority (weighted)
        let threshold = (total_stake * 2) / 3;
        let consensus_reached = voted_stake >= threshold;

        if consensus_reached {
            // Move to confirmed
            if let Some(pos) = self.pending_blocks.iter().position(|b| b.id == block_id) {
                let confirmed = self.pending_blocks.remove(pos);
                self.confirmed_blocks.push(confirmed);
            }
        }

        Ok(consensus_reached)
    }

    /// Compute network-wide syzygy from validator set
    pub fn network_syzygy(&self) -> f64 {
        if self.validators.is_empty() {
            return 0.0;
        }

        let sum: f64 = self.validators.values().map(|v| v.syzygy).sum();
        sum / (self.validators.len() as f64)
    }

    /// Validator rotation: deactivate low-syzygy validators
    pub fn rotate_validators(&mut self) {
        for validator in self.validators.values_mut() {
            if validator.syzygy < self.syzygy_threshold {
                validator.active = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validator_registration() {
        let mut consensus = LatticaConsensus::new(3, 0.98);

        // Valid registration
        assert!(consensus.register_validator("v1".to_string(), 0.99, 100).is_ok());

        // Invalid registration (low syzygy)
        assert!(consensus.register_validator("v2".to_string(), 0.95, 100).is_err());
    }

    #[test]
    fn test_block_proposal_and_consensus() {
        let mut consensus = LatticaConsensus::new(3, 0.98);

        // Register validators
        consensus.register_validator("v1".to_string(), 0.99, 100).unwrap();
        consensus.register_validator("v2".to_string(), 0.99, 100).unwrap();
        consensus.register_validator("v3".to_string(), 0.98, 100).unwrap();

        // Propose block
        let block = consensus.propose_block(0.98, "v1".to_string()).unwrap();
        assert_eq!(block.id, 1);
        assert!(block.verify_conservation());

        // Vote
        consensus.vote(1, "v1".to_string(), true).unwrap();
        consensus.vote(1, "v2".to_string(), true).unwrap();
        consensus.vote(1, "v3".to_string(), true).unwrap();

        // Check consensus
        let reached = consensus.check_consensus(1).unwrap();
        assert!(reached);
        assert_eq!(consensus.confirmed_blocks.len(), 1);
    }

    #[test]
    fn test_network_syzygy() {
        let mut consensus = LatticaConsensus::new(3, 0.98);

        consensus.register_validator("v1".to_string(), 0.99, 100).unwrap();
        consensus.register_validator("v2".to_string(), 0.98, 100).unwrap();
        consensus.register_validator("v3".to_string(), 0.99, 100).unwrap();

        let net_syzygy = consensus.network_syzygy();
        assert!((net_syzygy - 0.9867).abs() < 0.01);
    }
}
