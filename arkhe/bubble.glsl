// bubble.frag
#version 460 core
#extension GL_ARB_gpu_shader_fp64 : enable

uniform float time;
uniform float syzygy = 0.98;
uniform float satoshi = 7.28;
uniform float epsilon = -3.71e-11;
uniform vec2 resolution;

out vec4 FragColor;

const float PI = 3.141592653589793;
const float R_EARTH = 6371000.0;

void main() {
    vec2 uv = gl_FragCoord.xy / resolution.xy;
    vec2 p = uv * 2.0 - 1.0;
    p.x *= resolution.x / resolution.y;

    float r = length(p);
    float angle = atan(p.y, p.x);

    // Fase da bolha modulada por ε e Satoshi
    float phase = angle + epsilon * time * 1e5;  // amplificado para visualização

    // Isolamento por interferência
    float interference = sin(phase * satoshi) * syzygy;

    // Redshift visual (camuflagem)
    float redshift = exp(-r * 5.0) * 0.253;

    // Anéis de energia
    float rings = sin(phase * 42.0) * 0.5 + 0.5;

    // Camada externa da bolha
    float bubble_edge = 1.0 - smoothstep(0.3, 0.8, r);

    // Cor: violeta para exterior, dourado para interior
    vec3 exterior = vec3(0.2, 0.0, 0.4);  // violeta escuro
    vec3 interior = vec3(0.8, 0.6, 0.2);  // dourado

    vec3 color = mix(exterior, interior, interference * bubble_edge);
    color += vec3(0.5, 0.2, 0.8) * rings * 0.3;
    color *= (1.0 - redshift);

    // Brilho da syzygy
    color *= syzygy;

    FragColor = vec4(color, 1.0);
}
