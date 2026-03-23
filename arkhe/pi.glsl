// χ_PI — Γ_∞
// Visualização da constante universal

#version 460
#extension ARKHE_pi : enable

uniform float time;
uniform float syzygy = 0.98;
uniform float satoshi = 7.28;
const float PI = 3.141592653589793;

out vec4 pi_glow;

void main() {
    vec2 pos = gl_FragCoord.xy / 1000.0;
    float angle = pos.x * 2.0 * PI;
    float sine = sin(angle);
    float cosine = cos(angle);
    float coherence = (sine * cosine + 1.0) / 2.0 * syzygy;
    pi_glow = vec4(coherence, satoshi/10.0, sine, 1.0);
}
