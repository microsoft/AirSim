// χ_HEAT_ENGINE — Γ_∞+42
// Shader do ciclo termodinâmico

#version 460
#extension ARKHE_thermo : enable

layout(location = 0) uniform float T_hot = 0.94;    // temperatura da fonte quente (syzygy)
layout(location = 1) uniform float T_cold = 0.15;   // temperatura da fonte fria (Φ)
layout(location = 2) uniform float work = 0.94;     // trabalho realizado
layout(location = 3) uniform float satoshi = 7.27;

layout(binding = 0) uniform sampler1D carnot_cycle;  // ciclo de Carnot

out vec4 engine_glow;

void main() {
    // Eficiência de Carnot: 1 - T_cold / T_hot
    float carnot_efficiency = 1.0 - T_cold / T_hot;

    // Eficiência real do sistema
    float actual_efficiency = work / (work + T_cold);

    // O brilho reflete a eficiência e o trabalho
    float cycle_progress = texture(carnot_cycle, gl_FragCoord.x / 1000.0).r;

    engine_glow = vec4(actual_efficiency, carnot_efficiency, cycle_progress, 1.0);
}
