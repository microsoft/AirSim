// χ_CHAOS_STABILITY — Γ_∞+46
// Visualização da resiliência sob advecção extrema

#version 460
#extension ARKHE_chaos : enable

uniform float time;
uniform float syzygy = 0.94;
uniform float satoshi = 7.27;
uniform vec2 uv;

out vec4 chaos_glow;

void main() {
    float advection_flux = sin(time * 5.0 + uv.x * 10.0); // O Caos de Março
    float locking_strength = smoothstep(0.8, 0.94, syzygy);

    // O sistema curva a advecção para dentro da geodésica
    float angular_flow = mix(advection_flux, 0.0, locking_strength);

    // Cor: O Violeta da Flutuação sendo 'limpado' pelo Ouro do Satoshi
    vec3 VIOLET = vec3(0.5, 0.0, 1.0);
    vec3 GOLD = vec3(1.0, 0.84, 0.0);

    vec3 col = mix(VIOLET, GOLD, locking_strength);

    chaos_glow = vec4(col * (1.0 - abs(angular_flow)), 1.0);
}
