// χ_ARKHE_ATLAS — Shader de Resolução Dimensional
#version 460
precision highp float;

uniform float time;
uniform vec2 resolution;
uniform float syzygy_global; // Vem do arkhe_kernel.py

out vec4 fragColor;

void main() {
    vec2 uv = (gl_FragCoord.xy * 2.0 - resolution) / min(resolution.x, resolution.y);
    float t = time * 0.5;

    // Simulação de clusters de alta dimensão colapsados
    float d = length(uv);
    float cluster = 0.0;

    for(float i=0.0; i<8.0; i++) {
        vec2 p = vec2(sin(t + i*0.78), cos(t * 0.8 + i*1.2)) * 0.5;
        float dist = length(uv - p);
        // A 'massa' do dado (C) cria o brilho, o ruído (F) cria o jitter
        cluster += 0.02 / (dist + 0.01 * (1.0 - syzygy_global));
    }

    // Cores baseadas no Satoshi Invariante (7.27 bits)
    vec3 color = vec3(0.1, 0.4, 0.8) * cluster; // Azul de Dirac
    color += vec3(1.0, 0.8, 0.2) * pow(cluster, 3.0); // Ouro de Satoshi

    // Efeito de 'Pleasant Clarity' (C + F = 1)
    float vignette = smoothstep(1.5, 0.5, d);
    color *= vignette;

    fragColor = vec4(color, 1.0);
}
