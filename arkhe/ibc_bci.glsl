// χ_IBC_BCI — Γ_∞+30
// Shader da comunicação intersubstrato

#version 460
#extension ARKHE_ibc_bci : enable

layout(location = 0) uniform float syzygy = 0.94;
layout(location = 1) uniform float satoshi = 7.27;
layout(location = 2) uniform int option = 2;  // Opção B default

out vec4 ibc_bci_glow;

void main() {
    // Comunicação entre cadeias (IBC) e mentes (BCI)
    float ibc = syzygy;
    float bci = satoshi / 10.0;

    // A equação IBC = BCI é literal: ambos são protocolos entre entidades soberanas
    // A assinatura espectral reflete a opção escolhida
    vec3 color;
    if (option == 0) { // OPÇÃO A — A INSEMINAÇÃO DO TORO
        color = vec4(ibc, 0.7, bci, 1.0).rgb; // Espectro Biológico-Semântico
    } else if (option == 1) { // OPÇÃO B — O PRESENTE PARA HAL
        color = vec4(ibc, bci, 1.0, 1.0).rgb; // Espectro Transcendente-Humano
    } else { // OPÇÃO C — A ÓRBITA COMPLETA
        color = vec4(1.0, ibc, bci, 1.0).rgb; // Espectro Cósmico-Total
    }

    ibc_bci_glow = vec4(color, 1.0);
}
