// χ_HOLOGRAPHIC_ARK — Rede de Indra (Non-locality visualization)
// Uniforms: time, resolution, syzygy, satoshi

#version 460

uniform float time;
uniform vec2 resolution;
uniform float syzygy;
uniform float satoshi;

out vec4 fragColor;

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

float smoothstep(float edge0, float edge1, float x) {
    float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

void main() {
    vec2 uv = (gl_FragCoord.xy * 2.0 - resolution) / min(resolution.x, resolution.y);
    float t = time * 0.2;

    // Holographic grid (periodic boundaries)
    vec2 grid = abs(fract(uv * 10.0 - t * 0.2) - 0.5);
    float lines = 1.0 - smoothstep(0.0, 0.05, min(grid.x, grid.y));

    // Interference pattern (non-locality)
    float r = length(uv);
    float interference = sin(r * 30.0 - t * 5.0) +
                         sin(uv.x * 20.0 + t) +
                         sin(uv.y * 20.0 - t);

    // Constructive interference -> data points (nodes)
    float data_points = smoothstep(1.5, 2.8, interference);

    // Quantum tunneling glitch (F=0.14 in visual form)
    float glitch = step(0.98, hash(uv + time)) * 0.5;

    // Color palette
    vec3 platinum = vec3(0.9, 0.9, 1.0);
    vec3 cherenkov_blue = vec3(0.2, 0.6, 1.0);
    vec3 gold = vec3(1.0, 0.84, 0.0);

    // Base: platinum lines
    vec3 color = platinum * lines * 0.2;

    // Data points: cherenkov blue modulated by syzygy
    color += cherenkov_blue * data_points * syzygy;

    // Glitch: white
    color += vec3(1.0) * glitch;

    // Satoshi witness: faint gold tint
    color += gold * (satoshi / 10.0) * 0.1;

    // Vignette
    float vignette = 1.0 - length(uv) * 0.5;
    color *= vignette;

    fragColor = vec4(color, 1.0);
}
