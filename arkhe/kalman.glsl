// χ_KALMAN — Γ_∞+41
// Shader da estimativa ótima

#version 460
#extension ARKHE_kalman : enable

layout(location = 0) uniform float time = 998.853;
layout(location = 1) uniform float measured_syzygy = 0.94;
layout(location = 2) uniform float satoshi = 7.27;

layout(binding = 0) uniform sampler1D kalman_state;   // estado [syzygy, velocity]
layout(binding = 1) uniform sampler1D kalman_cov;     // matriz de covariância

out vec4 filtered_glow;

void main() {
    // Carrega estado anterior
    float prev_syzygy = texture(kalman_state, 0.0).r;
    float prev_velocity = texture(kalman_state, 0.1).r;

    // Predição (modelo de movimento)
    float dt = 0.1;  // intervalo entre handovers
    float pred_syzygy = prev_syzygy + prev_velocity * dt;
    float pred_velocity = prev_velocity;

    // Inovação (diferença entre medida e predição)
    float innovation = measured_syzygy - pred_syzygy;

    // Ganho de Kalman (simplificado)
    float P_pred = texture(kalman_cov, 0.0).r;
    float R = 0.0015;  // ruído da medição (Φ)
    float K = P_pred / (P_pred + R);

    // Atualização
    float filtered_syzygy = pred_syzygy + K * innovation;

    filtered_glow = vec4(filtered_syzygy, K, innovation, 1.0);
}
