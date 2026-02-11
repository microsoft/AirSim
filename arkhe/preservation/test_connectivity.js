// test_connectivity.js - Batismo do Pedestre 12
// Simula um aperto de mão 2FA para validar a rede agêntica.
import fetch from "node-fetch";

const GATEWAY_URL = process.env.GATEWAY_URL || "http://localhost:3000";

async function simulateHandshake() {
    console.log("🚀 Iniciando Batismo do Pedestre 12...");

    const payload = {
        opId: "batismo-012",
        desc: "Batismo do Pedestre 12: Registro Onchain ERC-8004",
        nonce: "CRYPTO-ENGRAM-2026",
        metadata: { severity: "0% (Iniciação)" }
    };

    try {
        console.log(`📡 Enviando solicitação para o Gateway: ${GATEWAY_URL}`);
        const response = await fetch(`${GATEWAY_URL}/request-approval`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        });

        if (response.ok) {
            console.log("✅ Solicitação enviada! Verifique seu Telegram.");
            console.log("⌛ Aguardando aprovação humana (polling)...");

            const start = Date.now();
            const interval = setInterval(async () => {
                const statusRes = await fetch(`${GATEWAY_URL}/approval-status/${payload.opId}`);
                const data = await statusRes.json();

                if (data.approved === true) {
                    console.log("🎉 BATISMO CONCLUÍDO: O Arquiteto autorizou o Pedestre 12.");
                    clearInterval(interval);
                } else if (data.approved === false) {
                    console.log("❌ BATISMO REJEITADO: A intenção foi negada.");
                    clearInterval(interval);
                }

                if (Date.now() - start > 60000) {
                    console.log("⏳ Teste encerrado por timeout (60s).");
                    clearInterval(interval);
                }
            }, 3000);
        } else {
            console.error(`❌ Falha ao enviar: ${response.status} ${response.statusText}`);
        }
    } catch (error) {
        console.error("❌ Erro de conexão:", error.message);
    }
}

simulateHandshake();
