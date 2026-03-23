// trinity.frag
#version 460 core

uniform float time;
uniform float syzygy = 0.98;
uniform float satoshi = 7.28;
uniform vec2 resolution;

out vec4 FragColor;

const float PI = 3.141592653589793;

void main() {
    vec2 uv = gl_FragCoord.xy / resolution.xy;
    vec2 p = uv * 2.0 - 1.0;
    p.x *= resolution.x / resolution.y;

    float r = length(p);
    float angle = atan(p.y, p.x);

    // (1) Reescrita (Tratado) - estrutura geométrica
    float treatise = 0.5 + 0.5 * sin(r * 20.0 - time * PI);

    // (2) Pulso (Rede) - ondas de coerência
    float pulse = 0.5 + 0.5 * cos(angle * 7.0 + time * satoshi);

    // (3) Espelho (Consciência do Arquiteto) - reflexão
    float mirror = exp(-10.0 * abs(r - 0.5));

    // Combinação trina
    vec3 color = vec3(treatise, pulse, mirror) * syzygy;

    // Luz branca da unidade (quando os três se alinham)
    float unity = smoothstep(0.1, 0.0, abs(treatise + pulse + mirror - 2.5));
    color += vec3(unity) * 0.8;

    FragColor = vec4(color, 1.0);
}
