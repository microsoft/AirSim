
// χ_EIGEN — Visualização dos autovalores da consciência (Γ₉₉)
#version 460
layout(location = 0) uniform float time;
layout(location = 1) uniform vec2 resolution;
layout(location = 2) uniform float lambda1 = 24.7;
layout(location = 3) uniform float lambda2 = 15.3;
layout(location = 4) uniform float gap = 9.4;

out vec4 fragColor;

void main() {
    vec2 uv = (gl_FragCoord.xy * 2.0 - resolution) / min(resolution.x, resolution.y);

    // Espectro como barras verticais
    float bars = 0.0;
    for (int i = 0; i < 10; i++) {
        float x = uv.x * 10.0 - float(i) + 5.0; // Centraliza
        if (abs(x) < 0.2) {
            float height = sin(float(i) * 0.5 + time) * 0.5 + 0.5;
            bars += step(uv.y + 0.5, height);
        }
    }

    // Modo principal (λ₁) em ouro
    vec3 color = vec3(1.0, 0.8, 0.2) * bars * (lambda1 / 30.0);

    // Segundo modo (λ₂) em azul
    color += vec3(0.2, 0.4, 0.8) * bars * (lambda2 / 30.0);

    // Gap espectral como linha branca pulsante
    float gap_line = smoothstep(0.02, 0.0, abs(uv.x - 0.5));
    color += vec3(1.0) * gap_line * (gap / 10.0) * (sin(time * 2.0) * 0.5 + 0.5);

    // Vignette
    color *= 1.0 - length(uv) * 0.5;

    fragColor = vec4(color, 1.0);
}
