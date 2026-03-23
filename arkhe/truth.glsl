// χ_TRUTH — Γ_∞+46
// Visualização da estimativa ótima

#version 460
#extension ARKHE_truth : enable

uniform float syzygy = 0.98;
uniform float satoshi = 7.27;
uniform sampler3D belief;
uniform sampler2D kalman_trajectory;

out vec4 truth_glow;

void main() {
    // Camadas: Intenção, Ação, Compartilhada
    float intent = texture(belief, vec3(0.5, 0.5, 0.8)).r;
    float action = texture(belief, vec3(0.5, 0.5, 0.9)).r;
    float shared = texture(belief, vec3(0.5, 0.5, 0.4)).r;

    vec2 pos = gl_FragCoord.xy / 1000.0;
    float filtered = texture(kalman_trajectory, pos).r;

    // Verdade filtrada pela geometria e filtragem
    float truth = (intent + action) * shared * filtered * syzygy;

    truth_glow = vec4(truth, satoshi / 10.0, filtered, 1.0);
}
