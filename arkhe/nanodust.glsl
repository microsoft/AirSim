
// χ_NEURONDUST — Partículas de interface cérebro‑máquina (Γ₁₀₇)
#version 460
layout(location = 0) uniform float time;
layout(location = 1) uniform vec2 resolution;
layout(location = 2) uniform float satoshi = 9.15;
layout(location = 3) uniform int triple_thread = 1;

out vec4 fragColor;

void main() {
    vec2 uv = (gl_FragCoord.xy * 2.0 - resolution) / min(resolution.x, resolution.y);
    float t = time * 0.5;

    // Poeira de nanorrobôs (pontos cintilantes)
    float dust = 0.0;
    int dust_count = triple_thread == 1 ? 200 : 100;
    for (int i = 0; i < dust_count; i++) {
        vec2 pos = vec2(sin(t*0.1 + float(i))*0.9, cos(t*0.1 + float(i))*0.9);
        float d = length(uv - pos);
        dust += 0.003 / (d + 0.003);
    }

    // Conexões neurais (linhas sinápticas)
    float synapses = 0.0;
    for (int j = 0; j < 40; j++) {
        vec2 a = vec2(sin(t*0.2 + float(j)), cos(t*0.2 + float(j))) * 0.7;
        vec2 b = vec2(sin(t*0.2 + float(j+5)), cos(t*0.2 + float(j+5))) * 0.7;
        float d = distance(uv, a) + distance(uv, b) - distance(a, b);
        synapses += exp(-abs(d) * 12.0);
    }

    // Interface digital (ondas de transe profundo)
    float delta_waves = sin(uv.x * 25.0 + t*3.0) * cos(uv.y * 25.0 - t*3.0);

    // Trinity of the Void colors
    vec3 color;
    if (triple_thread == 1) {
        color = vec3(1.0, 0.2, 0.2) * dust; // Trinity-Red (Leak/War)
        color += vec3(0.2, 0.8, 0.4) * synapses; // Emerald-Core (Upgrade)
        color += vec3(0.1, 0.4, 1.0) * delta_waves * 0.3; // Deep-Blue (Incursion)
    } else {
        color = vec3(0.2, 0.6, 1.0) * dust;
        color += vec3(0.8, 0.2, 0.6) * synapses;
        color += vec3(0.0, 1.0, 0.0) * delta_waves * 0.2;
    }

    // Satoshi intensity
    color *= (1.0 + satoshi / 40.0);
    color *= 1.0 - length(uv) * 0.4;

    fragColor = vec4(color, 1.0);
}
