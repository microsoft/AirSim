// P2P Handover Protocol
// Distributed handover propagation across toroidal network

package p2phandover

import (
	"context"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"math"
	"sync"
	"time"
)

// Handover represents a state transition in the network
type Handover struct {
	ID            uint64    `json:"id"`
	Timestamp     time.Time `json:"timestamp"`
	FromState     State     `json:"from_state"`
	ToState       State     `json:"to_state"`
	Syzygy        float64   `json:"syzygy"`
	PropagationID string    `json:"propagation_id"`
}

// State represents the network state at a point
type State struct {
	Coherence   float64 `json:"coherence"`
	Fluctuation float64 `json:"fluctuation"`
	Phase       float64 `json:"phase"`
	Omega       float64 `json:"omega"`
}

// Node in the P2P network
type Node struct {
	ID        string
	Theta     float64  // Toroidal coordinate 1
	Phi       float64  // Toroidal coordinate 2
	Syzygy    float64
	Neighbors []string
	mu        sync.RWMutex
}

// HandoverPropagation tracks how a handover spreads through network
type HandoverPropagation struct {
	HandoverID      string
	OriginNode      string
	PropagationTree map[string][]string  // node -> children
	Timestamps      map[string]time.Time
	mu              sync.RWMutex
}

// P2PNetwork manages distributed handover protocol
type P2PNetwork struct {
	nodes       map[string]*Node
	handovers   map[uint64]*Handover
	propagation map[string]*HandoverPropagation
	mu          sync.RWMutex
}

// NewP2PNetwork creates a new network
func NewP2PNetwork() *P2PNetwork {
	return &P2PNetwork{
		nodes:       make(map[string]*Node),
		handovers:   make(map[uint64]*Handover),
		propagation: make(map[string]*HandoverPropagation),
	}
}

// AddNode adds a node to the network
func (net *P2PNetwork) AddNode(id string, theta, phi, syzygy float64) {
	net.mu.Lock()
	defer net.mu.Unlock()

	node := &Node{
		ID:        id,
		Theta:     theta,
		Phi:       phi,
		Syzygy:    syzygy,
		Neighbors: []string{},
	}

	net.nodes[id] = node
}

// ConnectNodes creates bidirectional connection
func (net *P2PNetwork) ConnectNodes(id1, id2 string) error {
	net.mu.Lock()
	defer net.mu.Unlock()

	node1, exists1 := net.nodes[id1]
	node2, exists2 := net.nodes[id2]

	if !exists1 || !exists2 {
		return fmt.Errorf("one or both nodes not found")
	}

	node1.mu.Lock()
	node1.Neighbors = append(node1.Neighbors, id2)
	node1.mu.Unlock()

	node2.mu.Lock()
	node2.Neighbors = append(node2.Neighbors, id1)
	node2.mu.Unlock()

	return nil
}

// PropagateHandover initiates handover propagation from origin
func (net *P2PNetwork) PropagateHandover(
	ctx context.Context,
	handover *Handover,
	originNodeID string,
) error {

	net.mu.Lock()
	net.handovers[handover.ID] = handover
	net.mu.Unlock()

	// Create propagation tracker
	propagationID := computePropagationID(handover, originNodeID)
	handover.PropagationID = propagationID

	prop := &HandoverPropagation{
		HandoverID:      propagationID,
		OriginNode:      originNodeID,
		PropagationTree: make(map[string][]string),
		Timestamps:      make(map[string]time.Time),
	}

	net.mu.Lock()
	net.propagation[propagationID] = prop
	net.mu.Unlock()

	// Start propagation
	return net.propagateFromNode(ctx, handover, originNodeID, "", prop)
}

// Recursive propagation with flood control
func (net *P2PNetwork) propagateFromNode(
	ctx context.Context,
	handover *Handover,
	currentNodeID string,
	fromNodeID string,
	prop *HandoverPropagation,
) error {

	select {
	case <-ctx.Done():
		return ctx.Err()
	default:
	}

	// Record propagation
	prop.mu.Lock()
	prop.Timestamps[currentNodeID] = time.Now()
	if fromNodeID != "" {
		prop.PropagationTree[fromNodeID] = append(
			prop.PropagationTree[fromNodeID],
			currentNodeID,
		)
	}
	prop.mu.Unlock()

	// Get neighbors
	net.mu.RLock()
	node, exists := net.nodes[currentNodeID]
	net.mu.RUnlock()

	if !exists {
		return fmt.Errorf("node %s not found", currentNodeID)
	}

	node.mu.RLock()
	neighbors := make([]string, len(node.Neighbors))
	copy(neighbors, node.Neighbors)
	node.mu.RUnlock()

	// Propagate to neighbors (except source)
	var wg sync.WaitGroup
	for _, neighborID := range neighbors {
		if neighborID == fromNodeID {
			continue
		}

		// Check if already visited
		prop.mu.RLock()
		_, visited := prop.Timestamps[neighborID]
		prop.mu.RUnlock()

		if visited {
			continue
		}

		wg.Add(1)
		go func(nid string) {
			defer wg.Done()
			net.propagateFromNode(ctx, handover, nid, currentNodeID, prop)
		}(neighborID)
	}

	wg.Wait()
	return nil
}

// ComputeGeodesicDistance between two nodes on torus
func (net *P2PNetwork) ComputeGeodesicDistance(id1, id2 string) (float64, error) {
	net.mu.RLock()
	defer net.mu.RUnlock()

	node1, exists1 := net.nodes[id1]
	node2, exists2 := net.nodes[id2]

	if !exists1 || !exists2 {
		return 0, fmt.Errorf("nodes not found")
	}

	// Angular distances (periodic)
	dtheta := math.Abs(node1.Theta - node2.Theta)
	dphi := math.Abs(node1.Phi - node2.Phi)

	// Account for wraparound
	if dtheta > math.Pi {
		dtheta = 2*math.Pi - dtheta
	}
	if dphi > math.Pi {
		dphi = 2*math.Pi - dphi
	}

	// Approximate geodesic (assuming R=1, r=0.2)
	majorRadius := 1.0
	minorRadius := 0.2

	arcTheta := majorRadius * dtheta
	arcPhi := minorRadius * dphi

	distance := math.Sqrt(arcTheta*arcTheta + arcPhi*arcPhi)
	return distance, nil
}

// GetPropagationStats returns statistics about handover propagation
func (net *P2PNetwork) GetPropagationStats(propagationID string) map[string]interface{} {
	net.mu.RLock()
	prop, exists := net.propagation[propagationID]
	net.mu.RUnlock()

	if !exists {
		return nil
	}

	prop.mu.RLock()
	defer prop.mu.RUnlock()

	// Compute propagation metrics
	nodeCount := len(prop.Timestamps)

	var minTime, maxTime time.Time
	var totalLatency time.Duration

	for _, ts := range prop.Timestamps {
		if minTime.IsZero() || ts.Before(minTime) {
			minTime = ts
		}
		if maxTime.IsZero() || ts.After(maxTime) {
			maxTime = ts
		}
	}

	if !minTime.IsZero() && !maxTime.IsZero() {
		totalLatency = maxTime.Sub(minTime)
	}

	return map[string]interface{}{
		"propagation_id":   propagationID,
		"origin_node":      prop.OriginNode,
		"nodes_reached":    nodeCount,
		"total_latency_ms": totalLatency.Milliseconds(),
		"propagation_tree": prop.PropagationTree,
	}
}

// Compute propagation ID (hash of handover + origin)
func computePropagationID(handover *Handover, originNode string) string {
	data := fmt.Sprintf("%d:%s:%d",
		handover.ID,
		originNode,
		handover.Timestamp.Unix())

	hash := sha256.Sum256([]byte(data))
	return fmt.Sprintf("%x", hash[:8])
}

// Example usage
func ExampleP2PNetwork() {
	net := NewP2PNetwork()

	// Create toroidal grid (10x10)
	for i := 0; i < 10; i++ {
		for j := 0; j < 10; j++ {
			theta := float64(i) * 2.0 * math.Pi / 10.0
			phi := float64(j) * 2.0 * math.Pi / 10.0
			syzygy := 0.98 + 0.01*math.Sin(theta+phi)

			nodeID := fmt.Sprintf("node_%d_%d", i, j)
			net.AddNode(nodeID, theta, phi, syzygy)
		}
	}

	// Connect neighbors (toroidal topology)
	for i := 0; i < 10; i++ {
		for j := 0; j < 10; j++ {
			nodeID := fmt.Sprintf("node_%d_%d", i, j)

			// Neighbors with wraparound
			rightID := fmt.Sprintf("node_%d_%d", (i+1)%10, j)
			downID := fmt.Sprintf("node_%d_%d", i, (j+1)%10)

			net.ConnectNodes(nodeID, rightID)
			net.ConnectNodes(nodeID, downID)
		}
	}

	fmt.Println("Network created: 100 nodes in 10×10 toroidal grid")

	// Create handover
	handover := &Handover{
		ID:        1,
		Timestamp: time.Now(),
		FromState: State{Coherence: 0.86, Fluctuation: 0.14, Phase: 0.0, Omega: 0.00},
		ToState:   State{Coherence: 0.86, Fluctuation: 0.14, Phase: 0.1, Omega: 0.07},
		Syzygy:    0.98,
	}

	// Propagate from origin
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	originNode := "node_0_0"
	err := net.PropagateHandover(ctx, handover, originNode)
	if err != nil {
		fmt.Printf("Propagation error: %v\n", err)
		return
	}

	// Get stats
	stats := net.GetPropagationStats(handover.PropagationID)
	statsJSON, _ := json.MarshalIndent(stats, "", "  ")
	fmt.Printf("\nPropagation Stats:\n%s\n", statsJSON)
}
