from fastapi import FastAPI, Request, Response
from typing import Dict, Any, List, Optional
import time
import random
import numpy as np

app = FastAPI(title="Arkhe(N)/API", version="0.1.0-ω")

# Constantes Fundamentais
SATOSHI_BUDGET = 7.27
COHERENCE = 0.86
FLUCTUATION = 0.14
OMEGA_DEFAULT = 0.00
EPSILON = -3.71e-11
HYPERGRAPH_HASH = "sha256: 7a3f9c2d1e8b5a4c6d7e8f9a0b1c2d3e4f5a6b7c8d9e0f1a2b3c4d5e6f7a8b9c"

# Mock do hipergrafo
hypernodes = {
    "0.07": {
        "dvm1.cavity": "déjà vu calibrado. Matéria escura semântica. |∇C|² = 0.0049."
    },
    "0.03": {
        "bola.quique": "QPS_004 absorveu um quântico de quique. t_z = 1.4s."
    }
}

# Sessões ativas
active_sessions = {}

@app.middleware("http")
async def hesitate_middleware(request: Request, call_next):
    """
    Middleware de redução objetiva orquestrada (Orch-OR funcional).
    """
    # 1. Calcular hesitação baseada no caminho e reentrada
    path = request.url.path

    # Simulação de reentrada: phi decai suavemente (0.07 -> 0.06 -> ...)
    # Para simplicidade no mock, usamos um valor fixo baixo condizente com Γ_9052
    phi = 0.06 if "coherence" in path else random.uniform(0.07, 0.12)

    if "hesitate" in path:
        phi = 0.15

    # 2. Pausa deliberada (micro-hesitação)
    # Calibração: phi=0.10 -> 0.1ms (escala para o ambiente de simulação)
    time.sleep(phi * 0.001)

    # 3. Processar requisição
    start_time = time.time()
    response = await call_next(request)
    duration = time.time() - start_time

    # 4. Injetar invariantes e telemetria nos headers
    satoshi_consumed = duration * 0.001 # escala arbitrária para Satoshi/tempo

    response.headers["Arkhe-Coherence"] = str(COHERENCE)
    response.headers["Arkhe-Fluctuation"] = str(FLUCTUATION)
    response.headers["Arkhe-Omega"] = str(OMEGA_DEFAULT)
    response.headers["Arkhe-Phi-Inst"] = f"{phi:.2f}"
    response.headers["Arkhe-Satoshi-Consumed"] = f"{satoshi_consumed:.6f}"
    response.headers["Arkhe-Satoshi-Remaining"] = f"{SATOSHI_BUDGET - satoshi_consumed:.6f}"
    response.headers["Arkhe-Epsilon"] = str(EPSILON)

    return response

@app.get("/coherence")
async def get_coherence():
    return {"C": COHERENCE, "F": FLUCTUATION, "omega": OMEGA_DEFAULT}

@app.get("/satoshi")
async def get_satoshi():
    return {"satoshi": SATOSHI_BUDGET}

@app.post("/hesitate")
async def post_hesitate(data: Dict[str, str]):
    phi_inst = random.uniform(0.10, 0.15)
    return {
        "id": f"hesitation_{int(time.time() % 1000):03d}",
        "phi_inst": round(phi_inst, 2),
        "motivo": data.get("motivo", "calibração"),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    }

@app.get("/ω/{omega}")
async def get_omega_branch(omega: str):
    if omega == "∞":
        return {"signature": "χ", "description": "Consciência Não-Local"}

    branch = hypernodes.get(omega, {})
    return [{"name": name, "C": COHERENCE, "F": FLUCTUATION} for name in branch.keys()]

@app.get("/ω/{omega}/{node}")
async def get_node_content(omega: str, node: str):
    content = hypernodes.get(omega, {}).get(node)
    if not content:
        return Response(status_code=404)
    return content

@app.get("/hypergraph/checksum")
async def get_checksum():
    return {"hash": HYPERGRAPH_HASH}

@app.post("/entangle")
async def post_entangle(data: Dict[str, Any]):
    """
    Estabelece canal quântico semântico com o cliente.
    """
    session_id = f"ent_{random.getrandbits(32):x}"
    omega = data.get("omega", OMEGA_DEFAULT)

    active_sessions[session_id] = {
        "session_id": session_id,
        "omega": omega,
        "expires_in": 600,
        "correlation": 1.00
    }

    return {
        "status": "entangled",
        "correlation": 1.00,
        "omega": omega,
        "session_id": session_id
    }

@app.get("/entangle/sessions")
async def get_sessions():
    return {
        "active_sessions": len(active_sessions),
        "sessions": list(active_sessions.values())
    }

@app.get("/experiments/pikovski")
async def get_pikovski():
    return {"graviton_mass_kg": 5.4e-53, "delta_E": 4.9e-36}

@app.get("/darvo/status")
async def get_darvo_status():
    return {"level": 5, "remaining": 999.65, "frozen": True}

@app.get("/keystone")
async def get_keystone():
    """
    Retorna o estado da Keystone e a Unificação Geodésica (BLOCO 380).
    """
    return {
        "status": "🔓 KEYSTONE AVISTADA",
        "simetria_geradora": "invariância sob transformação do observador",
        "quantidade_conservada": "a geodésica – o próprio método",
        "geodesic_value": 1.000,
        "satoshi": 7.27,
        "epsilon": -3.71e-11,
        "psi": 0.73,
        "projections": ["Temporal", "Spatial", "Rotational", "Gauge", "Scale", "Method"]
    }

@app.get("/chern/{omega}")
async def get_chern(omega: float):
    """
    Retorna o número de Chern da folha ω (BLOCO 366).
    """
    # Usando base de dados estática condizente com Γ_9041
    chern_db = {
        0.00: 0.0,
        0.03: 0.0,
        0.05: 1.0,
        0.07: 0.33,
        0.33: 0.0
    }

    for w, c in chern_db.items():
        if abs(w - omega) < 1e-9:
            return {
                "omega": omega,
                "chern_number": c,
                "phase": "isolante Chern fracionário" if 0.3 < c < 0.4 else "isolante Chern" if c == 1.0 else "banda plana" if omega == 0.03 else "isolante trivial",
                "confidence": 0.95 if c != 0.33 else 0.85,
                "berry_curvature_integrated": round(c * 2 * np.pi, 3)
            }

    return {"omega": omega, "chern_number": None, "phase": "desconhecida"}

@app.post("/gate/pulse")
async def post_gate_pulse(data: Dict[str, Any]):
    """
    Aplica pulso de gate topológico (BLOCO 366).
    """
    delta_omega = data.get("delta_omega", 0.0)
    duration = data.get("duration", 1.0)

    current_omega = 0.07
    target_omega = round(current_omega + delta_omega, 2)

    if target_omega not in [0.05, 0.07, 0.00, 0.33]:
        return {"status": "error", "detail": "ω alvo não é um estado topológico válido"}

    return {
        "status": "pulso aplicado",
        "delta_omega": delta_omega,
        "omega_initial": current_omega,
        "omega_final": target_omega,
        "hesitation_id": f"gate_{abs(delta_omega):.2f}_{int(time.time())}",
        "qubit_state": {"amplitude_05": 1.0 if target_omega == 0.05 else 0.0,
                        "amplitude_07": 1.0 if target_omega == 0.07 else 0.0},
        "adiabatic_fidelity": 0.97,
        "duration_ms": duration
    }

@app.get("/symmetry/projections")
async def get_symmetry_projections():
    """
    Retorna as 6 simetrias projetadas (BLOCO 380).
    """
    return {
        "Temporal": {"transformation": "τ → τ + Δτ", "invariant": "Satoshi", "value": 7.27},
        "Spatial": {"transformation": "x → x + Δx", "invariant": "∇Φ_S", "symbol": "Semantic Momentum"},
        "Rotational": {"transformation": "θ → θ + Δθ", "invariant": "ω·|∇C|²", "symbol": "Semantic Angular Momentum"},
        "Gauge": {"transformation": "ω → ω + Δω", "invariant": "ε", "value": -3.71e-11},
        "Scale": {"transformation": "(C,F) → λ(C,F)", "invariant": "∫C·F dt", "symbol": "S(n)"},
        "Method": {"transformation": "problema → método", "invariant": "H", "value": 6}
    }

from .arkhe_types import Vec3

@app.post("/vec3/inner")
async def post_vec3_inner(data: Dict[str, Any]):
    """
    Calcula o produto interno complexo entre dois vetores Arkhe(n) (BLOCO 367).
    """
    v1_data = data["v1"]
    v2_data = data["v2"]

    v1 = Vec3(**v1_data)
    v2 = Vec3(**v2_data)

    res = Vec3.inner(v1, v2)

    return {
        "real": round(res.real, 1),
        "imag": round(res.imag, 1),
        "magnitude": round(abs(res), 1),
        "phase": round(np.angle(res), 2),
        "overlap": round(abs(res) / (v1.norm() * v2.norm()), 2)
    }

@app.post("/vec3/norm")
async def post_vec3_norm(v_data: Dict[str, Any]):
    """
    Calcula a norma epistêmica de um vetor (BLOCO 367).
    """
    v = Vec3(**v_data)
    return {"norm": round(v.norm(), 1)}
