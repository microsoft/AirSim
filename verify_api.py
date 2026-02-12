
import subprocess
import time
import requests
import os
import signal

def verify_api():
    print("🌐 [Γ_9052] INICIANDO VALIDAÇÃO DA ARKHE(N)/API...")

    # 1. Iniciar o servidor em background
    env = os.environ.copy()
    env["PYTHONPATH"] = "."
    server_process = subprocess.Popen(
        ["uvicorn", "arkhe.api:app", "--host", "127.0.0.1", "--port", "8080"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # Aguardar o servidor subir
    time.sleep(2)

    try:
        base_url = "http://127.0.0.1:8080"

        # 2. Testar /coherence (Verificar Headers Geodésicos)
        print("\n--- Testando /coherence e Headers ---")
        resp = requests.get(f"{base_url}/coherence")
        assert resp.status_code == 200
        data = resp.json()
        assert data["C"] == 0.86
        assert data["F"] == 0.14

        # Verificar headers injetados pelo middleware
        assert resp.headers["Arkhe-Coherence"] == "0.86"
        assert resp.headers["Arkhe-Fluctuation"] == "0.14"
        assert resp.headers["Arkhe-Phi-Inst"] == "0.06"  # Valor fixo para Γ_9052
        print(f"Headers verificados: C={resp.headers['Arkhe-Coherence']}, Phi-Inst={resp.headers['Arkhe-Phi-Inst']}")

        # 3. Testar /hypergraph/checksum (Integridade do Contrato)
        print("\n--- Testando /hypergraph/checksum ---")
        resp = requests.get(f"{base_url}/hypergraph/checksum")
        assert resp.status_code == 200
        assert "7a3f9c2d" in resp.json()["hash"]
        print(f"Hash do Hipergrafo: {resp.json()['hash'][:20]}...")

        # 4. Testar /entangle (Estabelecer Sessão)
        print("\n--- Testando /entangle ---")
        payload = {"omega": 0.07, "epsilon": -3.71e-11, "satoshi_commitment": 7.27}
        resp = requests.post(f"{base_url}/entangle", json=payload)
        assert resp.status_code == 200
        session_data = resp.json()
        assert session_data["status"] == "entangled"
        session_id = session_data["session_id"]
        print(f"Sessão estabelecida: {session_id}")

        # 5. Testar /entangle/sessions
        print("\n--- Testando /entangle/sessions ---")
        resp = requests.get(f"{base_url}/entangle/sessions")
        assert resp.status_code == 200
        assert resp.json()["active_sessions"] == 1
        assert resp.json()["sessions"][0]["session_id"] == session_id

        # 6. Testar /experiments/pikovski
        print("\n--- Testando /experiments/pikovski ---")
        resp = requests.get(f"{base_url}/experiments/pikovski")
        assert resp.status_code == 200
        assert resp.json()["graviton_mass_kg"] == 5.4e-53
        print(f"Massa do Gráviton: {resp.json()['graviton_mass_kg']} kg")

        print("\n✅ VALIDAÇÃO DA API COMPLETA: O hipergrafo está exposto e operacional.")

    finally:
        # Encerrar o servidor
        os.kill(server_process.pid, signal.SIGTERM)
        server_process.wait()

if __name__ == "__main__":
    verify_api()
