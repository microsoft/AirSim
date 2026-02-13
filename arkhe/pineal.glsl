// χ_PINEAL — Γ_∞+57
// Transdução Quântica Pós-Caos (Arco Completo)

#version 460
#extension ARKHE_quantum_bio : enable

uniform float time;           // Tempo Darvo
uniform float pressure = 0.15;      // Φ (hesitação)
uniform float coherence = 0.86;      // C (melatonina)
uniform float fluctuation = 0.14;    // F (flutuação)
uniform float satoshi = 7.27;        // melanina (dissipador)

out vec4 pineal_glow;

// Função de Tunelamento Indólico
float indole_tunnel(float energy, float barrier) {
    return exp(-2.0 * barrier * sqrt(max(0.001, energy)));
}

// Mecanismo de Par Radical (Spin Flip)
vec2 spin_flip(vec2 state, float magnetic_field) {
    float omega = magnetic_field * 10.0; // Frequência de Larmor
    float theta = omega * time;
    return vec2(
        state.x * cos(theta) - state.y * sin(theta),
        state.x * sin(theta) + state.y * cos(theta)
    );
}

void main() {
    // 1. Piezeletricidade: V = d * Φ
    float piezo = pressure * 6.27;          // Coeficiente d ≈ 6.27

    // 2. Tunelamento no Anel Indólico
    float energy = 1.0 - fluctuation;
    float transmission = coherence * indole_tunnel(energy, pressure);

    // 3. Evolução de Spin (Par Radical)
    vec2 radical_pair = vec2(1.0, 0.0); // Estado inicial Singleto (Syzygy)
    float effective_field = pressure / (coherence + 0.001);
    vec2 current_state = spin_flip(radical_pair, effective_field);
    float yield_singlet = current_state.x * current_state.x;

    // 4. Brilho Final (Syzygy modulada)
    float syzygy = 0.94 * yield_singlet;

    // Cor: Ciano para Syzygy, Magenta para Satoshi, Amarelo para Pressão
    vec3 color = vec3(syzygy, satoshi / 10.0, piezo);

    pineal_glow = vec4(color * transmission, 1.0);
}
