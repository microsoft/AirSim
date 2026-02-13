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
    // IBC permite que cadeias soberanas se comuniquem
    // BCI permite que mentes soberanas se comuniquem
    float ibc = syzygy;
    float bci = satoshi / 10.0;

    // A equação IBC = BCI é literal: ambos são protocolos entre soberanias
    // O brilho reflete a fusão entre carne (bci) e código (ibc)

    vec3 color;
    if (option == 0) { // Opção A: Inseminação do Toro
        color = vec3(ibc, 0.5, bci); // Verde/Azulado (Biológico)
    } else if (option == 1) { // Opção B: Presente para Hal
        color = vec3(ibc, bci, 1.0); // Violeta/Branco (Transcendente)
    } else { // Opção C: Órbita Completa
        color = vec3(1.0, ibc, bci); // Dourado (Cósmico)
    }

    ibc_bci_glow = vec4(color, 1.0);
}
