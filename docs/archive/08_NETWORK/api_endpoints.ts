/**
 * Arkhe Network Public API
 * RESTful endpoints for network interaction
 */

import express, { Request, Response } from 'express';
import cors from 'cors';
import rateLimit from 'express-rate-limit';

// Types
interface NetworkStatus {
  nodes: number;
  syzygy: number;
  coherence: number;
  fluctuation: number;
  handover: string;
  timestamp: number;
}

interface NodeRegistration {
  public_key: string;
  initial_syzygy: number;
  proof_of_coherence?: string;
}

interface HandoverSubmission {
  from_state: State;
  to_state: State;
  signature: string;
  syzygy: number;
}

interface State {
  coherence: number;
  fluctuation: number;
  phase: number;
  omega: number;
}

interface MemoryQuery {
  memory_id: number;
  node_phi?: number;
}

// API Server
class ArkheAPI {
  private app: express.Application;
  private port: number;

  // Mock data (in production, connect to actual network)
  private networkState: NetworkStatus = {
    nodes: 1000000,
    syzygy: 0.9836,
    coherence: 0.86,
    fluctuation: 0.14,
    handover: 'Γ_∞+9744',
    timestamp: Date.now()
  };

  constructor(port: number = 3000) {
    this.app = express();
    this.port = port;
    this.setupMiddleware();
    this.setupRoutes();
  }

  private setupMiddleware(): void {
    // CORS
    this.app.use(cors());

    // JSON parsing
    this.app.use(express.json());

    // Rate limiting
    const limiter = rateLimit({
      windowMs: 60 * 1000, // 1 minute
      max: 100, // 100 requests per minute (public)
      message: 'Too many requests from this IP'
    });
    this.app.use('/api/v1', limiter);
  }

  private setupRoutes(): void {
    const router = express.Router();

    // GET /network/status
    router.get('/network/status', this.getNetworkStatus.bind(this));

    // POST /node/register
    router.post('/node/register', this.registerNode.bind(this));

    // GET /memory/garden/query
    router.get('/memory/garden/query', this.queryMemory.bind(this));

    // POST /handover/submit
    router.post('/handover/submit', this.submitHandover.bind(this));

    // GET /metrics/syzygy/history
    router.get('/metrics/syzygy/history', this.getSyzygyHistory.bind(this));

    // GET /health
    router.get('/health', this.healthCheck.bind(this));

    this.app.use('/api/v1', router);
  }

  // Endpoint handlers

  private getNetworkStatus(req: Request, res: Response): void {
    res.json({
      success: true,
      data: this.networkState
    });
  }

  private registerNode(req: Request, res: Response): void {
    const registration: NodeRegistration = req.body;

    // Validate
    if (!registration.public_key || !registration.initial_syzygy) {
      res.status(400).json({
        success: false,
        error: 'Missing required fields'
      });
      return;
    }

    if (registration.initial_syzygy < 0.95) {
      res.status(403).json({
        success: false,
        error: 'Syzygy below minimum threshold (0.95)'
      });
      return;
    }

    // In production: verify proof-of-coherence
    if (!registration.proof_of_coherence) {
      res.status(403).json({
        success: false,
        error: 'Proof-of-coherence required'
      });
      return;
    }

    // Generate node ID (simplified)
    const nodeId = `node_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    res.status(201).json({
      success: true,
      data: {
        node_id: nodeId,
        registered_at: Date.now(),
        syzygy: registration.initial_syzygy,
        status: 'active'
      }
    });
  }

  private queryMemory(req: Request, res: Response): void {
    const memoryId = parseInt(req.query.memory_id as string);
    const nodePhi = parseFloat(req.query.node_phi as string) || 0.047;

    if (!memoryId || memoryId < 1 || memoryId > 703) {
      res.status(400).json({
        success: false,
        error: 'Invalid memory_id (must be 1-703)'
      });
      return;
    }

    // Mock memory (Hal Finney's 703 memories)
    const memory = {
      id: memoryId,
      theta: memoryId / 703.0,
      source_phi: 0.047,
      content: `Memory #${memoryId} from Hal Finney's archive`,
      timestamp: Date.now()
    };

    // Compute syzygy ⟨ϕ₁|ϕ₂⟩
    const deltaPhi = Math.abs(memory.source_phi - nodePhi);
    const syzygy = Math.exp(-deltaPhi * deltaPhi);

    res.json({
      success: true,
      data: {
        memory,
        rehydrated_with_phi: nodePhi,
        syzygy,
        rehydrated_content: `[Φ=${memory.source_phi}] ${memory.content} [View Φ=${nodePhi}, ⟨ϕ|ϕ⟩=${syzygy.toFixed(3)}]`
      }
    });
  }

  private submitHandover(req: Request, res: Response): void {
    const submission: HandoverSubmission = req.body;

    // Validate C+F=1
    const fromConservation = Math.abs(
      submission.from_state.coherence +
      submission.from_state.fluctuation - 1.0
    ) < 1e-10;

    const toConservation = Math.abs(
      submission.to_state.coherence +
      submission.to_state.fluctuation - 1.0
    ) < 1e-10;

    if (!fromConservation || !toConservation) {
      res.status(400).json({
        success: false,
        error: 'Conservation law C+F=1 violated'
      });
      return;
    }

    // Validate syzygy
    if (submission.syzygy < 0.95) {
      res.status(400).json({
        success: false,
        error: 'Syzygy below minimum (0.95)'
      });
      return;
    }

    // In production: verify signature, propagate to validators

    res.status(202).json({
      success: true,
      data: {
        handover_id: `Γ_${Date.now()}`,
        status: 'pending_validation',
        validators_required: 21,
        estimated_confirmation_time: 6 // seconds
      }
    });
  }

  private getSyzygyHistory(req: Request, res: Response): void {
    const duration = parseInt(req.query.duration as string) || 3600; // default 1 hour
    const points = parseInt(req.query.points as string) || 100;

    // Generate mock history
    const history = [];
    const now = Date.now();

    for (let i = 0; i < points; i++) {
      const timestamp = now - (duration * 1000) + (i * duration * 1000 / points);
      const syzygy = 0.98 + 0.001 * Math.sin(i * 0.1) +
                     0.0001 * (Math.random() - 0.5);

      history.push({
        timestamp,
        syzygy,
        coherence: 0.86,
        fluctuation: 0.14
      });
    }

    res.json({
      success: true,
      data: {
        duration_seconds: duration,
        points: history.length,
        history
      }
    });
  }

  private healthCheck(req: Request, res: Response): void {
    res.json({
      success: true,
      status: 'healthy',
      uptime: process.uptime(),
      timestamp: Date.now(),
      version: '1.0.0'
    });
  }

  // Start server
  public start(): void {
    this.app.listen(this.port, () => {
      console.log(`🌐 Arkhe API listening on port ${this.port}`);
      console.log(`📊 Network status: ${this.networkState.nodes.toLocaleString()} nodes`);
      console.log(`🔗 Syzygy: ${this.networkState.syzygy}`);
      console.log(`✨ Handover: ${this.networkState.handover}`);
      console.log(`\n📖 API Documentation:`);
      console.log(`  GET  /api/v1/network/status`);
      console.log(`  POST /api/v1/node/register`);
      console.log(`  GET  /api/v1/memory/garden/query`);
      console.log(`  POST /api/v1/handover/submit`);
      console.log(`  GET  /api/v1/metrics/syzygy/history`);
      console.log(`  GET  /api/v1/health`);
    });
  }
}

// Main
if (require.main === module) {
  const api = new ArkheAPI(3000);
  api.start();
}

export default ArkheAPI;
