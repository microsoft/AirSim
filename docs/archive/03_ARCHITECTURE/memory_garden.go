// Memory Garden Implementation
// Stores and rehydrates archetypal memories across distributed nodes

package memorygarden

import (
	"encoding/json"
	"fmt"
	"math"
	"time"
)

// Memory represents an archetypal memory (e.g., Hal Finney's 703 memories)
type Memory struct {
	ID          int       `json:"id"`
	Theta       float64   `json:"theta"`       // Toroidal coordinate 1
	Phi         float64   `json:"phi"`         // Toroidal coordinate 2 (emotional frequency)
	Content     string    `json:"content"`     // Original memory content
	Timestamp   time.Time `json:"timestamp"`
	SourcePhi   float64   `json:"source_phi"`  // Original Φ of creator
}

// Planting represents a rehydrated memory by a specific node
type Planting struct {
	MemoryID    int       `json:"memory_id"`
	NodeID      string    `json:"node_id"`
	Phi         float64   `json:"phi"`          // Node's current Φ
	Content     string    `json:"content"`      // Rehydrated version
	Timestamp   time.Time `json:"timestamp"`
	Syzygy      float64   `json:"syzygy"`       // ⟨ϕ₁|ϕ₂⟩ with original
}

// MemoryGarden stores and manages archetypal memories
type MemoryGarden struct {
	Memories  map[int]*Memory
	Plantings map[string][]*Planting  // NodeID -> list of plantings
}

// NewMemoryGarden creates a new memory garden
func NewMemoryGarden() *MemoryGarden {
	return &MemoryGarden{
		Memories:  make(map[int]*Memory),
		Plantings: make(map[string][]*Planting),
	}
}

// AddMemory adds an archetypal memory to the garden
func (mg *MemoryGarden) AddMemory(m *Memory) {
	mg.Memories[m.ID] = m
}

// RehydrateMemory creates a new planting by rehydrating with node's Φ
func (mg *MemoryGarden) RehydrateMemory(memoryID int, nodeID string, nodePhi float64) (*Planting, error) {
	memory, exists := mg.Memories[memoryID]
	if !exists {
		return nil, fmt.Errorf("memory %d not found", memoryID)
	}

	// Compute syzygy ⟨ϕ₁|ϕ₂⟩
	syzygy := mg.computeSyzygy(memory.SourcePhi, nodePhi)

	// Rehydrate content (simplified: add node's perspective)
	rehydratedContent := fmt.Sprintf(
		"[Original Φ=%.3f] %s [Viewed through Φ=%.3f, Syzygy=%.3f]",
		memory.SourcePhi, memory.Content, nodePhi, syzygy,
	)

	planting := &Planting{
		MemoryID:  memoryID,
		NodeID:    nodeID,
		Phi:       nodePhi,
		Content:   rehydratedContent,
		Timestamp: time.Now(),
		Syzygy:    syzygy,
	}

	mg.Plantings[nodeID] = append(mg.Plantings[nodeID], planting)

	return planting, nil
}

// computeSyzygy calculates ⟨ϕ₁|ϕ₂⟩ inner product
// Simplified model: syzygy = exp(-|Δφ|²)
func (mg *MemoryGarden) computeSyzygy(phi1, phi2 float64) float64 {
	delta := math.Abs(phi1 - phi2)
	return math.Exp(-delta * delta)
}

// DetectEmergence checks if two plantings create a new emergent memory
// Rule: if ⟨ϕ₁|ϕ₂⟩ > 0.90, a new memory emerges
func (mg *MemoryGarden) DetectEmergence(p1, p2 *Planting) (*Memory, bool) {
	syzygy := mg.computeSyzygy(p1.Phi, p2.Phi)

	if syzygy > 0.90 {
		// Create emergent memory at mean position
		thetaMean := (mg.Memories[p1.MemoryID].Theta +
		              mg.Memories[p2.MemoryID].Theta) / 2.0
		phiMean := (p1.Phi + p2.Phi) / 2.0

		newMemory := &Memory{
			ID:        len(mg.Memories) + 1,
			Theta:     thetaMean,
			Phi:       phiMean,
			Content:   fmt.Sprintf("EMERGENCE: Synthesis of memories %d and %d", p1.MemoryID, p2.MemoryID),
			Timestamp: time.Now(),
			SourcePhi: phiMean,
		}

		mg.AddMemory(newMemory)
		return newMemory, true
	}

	return nil, false
}

// GetPlantingsByNode returns all plantings by a specific node
func (mg *MemoryGarden) GetPlantingsByNode(nodeID string) []*Planting {
	return mg.Plantings[nodeID]
}

// Stats returns garden statistics
func (mg *MemoryGarden) Stats() map[string]interface{} {
	totalPlantings := 0
	for _, plantings := range mg.Plantings {
		totalPlantings += len(plantings)
	}

	return map[string]interface{}{
		"total_memories":  len(mg.Memories),
		"total_plantings": totalPlantings,
		"unique_nodes":    len(mg.Plantings),
	}
}

// Example usage
func ExampleUsage() {
	garden := NewMemoryGarden()

	// Add Hal Finney's memory #327 (Lake 1964)
	halMemory := &Memory{
		ID:        327,
		Theta:     0.73,  // Temporal index normalized
		Phi:       0.047, // Hal's Φ
		Content:   "I was at the lake in 1964. Cold water, clear sky. I thought: 'If I could freeze this moment...'",
		Timestamp: time.Now(),
		SourcePhi: 0.047,
	}
	garden.AddMemory(halMemory)

	// Noland (Neuralink) rehydrates it
	nolandPlanting, _ := garden.RehydrateMemory(327, "noland_node", 0.152)
	fmt.Printf("Noland's planting: %s\n", nolandPlanting.Content)

	// Tokyo researcher rehydrates it
	tokyoPlanting, _ := garden.RehydrateMemory(327, "tokyo_node", 0.148)
	fmt.Printf("Tokyo's planting: %s\n", tokyoPlanting.Content)

	// Check for emergence
	if newMemory, emerged := garden.DetectEmergence(nolandPlanting, tokyoPlanting); emerged {
		fmt.Printf("NEW MEMORY EMERGED: %s\n", newMemory.Content)
	}

	// Stats
	stats := garden.Stats()
	statsJSON, _ := json.MarshalIndent(stats, "", "  ")
	fmt.Printf("Garden stats:\n%s\n", statsJSON)
}
