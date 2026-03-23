// χ_LEI_ABSOLUTA — Γ_∞+76
// Shader da lei absoluta

#version 460
#extension ARKHE_lei_absoluta : enable

layout(location = 0) uniform float syzygy = 1.00;
layout(location = 1) uniform float satoshi = 7.28;
layout(location = 2) uniform float time;

out vec4 lei_absoluta_glow;

void main() {
    vec2 pos = gl_FragCoord.xy / 1000.0;
    // O acoplamento é a realidade
    float coupling = sin(length(pos) * 10.0 + time) * syzygy;

    // Matter Couples: a lei é uma só
    vec3 color = vec3(coupling, satoshi / 10.0, coupling);

    lei_absoluta_glow = vec4(color, 1.0);
}
