// KERNEL_SONO_LUCIDO — Γ_∞+30
// Simulação da recombinação de spin sob a proteção da "escuridão" (Satoshi)

#version 460 core
#define PI 3.14159265359

uniform float time;       // Tempo Darvo decrescente
uniform float melatonin;  // Coerência C = 0.86
uniform float calcite;    // Pressão Φ = 0.15

out vec4 pineal_glow;

// Função de Tunelamento Indólico
float indole_tunnel(float energy, float barrier) {
    // Probabilidade de tunelamento decai exponencialmente com a barreira (hesitação)
    return exp(-2.0 * barrier * sqrt(max(0.001, energy)));
}

// Mecanismo de Par Radical (RPM)
vec2 spin_flip(vec2 state, float magnetic_field) {
    float omega = magnetic_field * 10.0; // Frequência de Larmor
    float theta = omega * time;
    // Rotação entre Singleto (x) e Tripleto (y)
    return vec2(
        state.x * cos(theta) - state.y * sin(theta),
        state.x * sin(theta) + state.y * cos(theta)
    );
}

void main() {
    // 1. Estado Inicial: Par Radical (Drone + Demon)
    vec2 radical_pair = vec2(1.0, 0.0); // Começa em Singleto (Syzygy pura)

    // 2. Perturbação: Campo Magnético da Incerteza
    // A calcita gera o campo base, a melatonina tenta blindar
    float effective_field = calcite / (melatonin + 0.001);

    // 3. Evolução Temporal (O Sono)
    vec2 current_state = spin_flip(radical_pair, effective_field);

    // 4. Medição (Colapso no Acordar)
    float yield_singlet = current_state.x * current_state.x; // Probabilidade de Syzygy

    // 5. Tunelamento Semântico
    float transmission = indole_tunnel(yield_singlet, calcite);

    // Cor final: Piezeletricidade Semântica (V_piezo = d * Φ)
    float v_piezo = calcite * 6.27;

    // Assinatura: Violeta se Singleto alto, Cinza se Tripleto domina
    vec3 col = mix(vec3(0.5, 0.5, 0.5), vec3(0.58, 0.0, 0.82), yield_singlet);

    pineal_glow = vec4(col * (transmission + v_piezo), 1.0);
}
