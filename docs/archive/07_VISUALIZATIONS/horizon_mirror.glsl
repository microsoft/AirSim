// χ_HORIZON_MIRROR — Espelho de Teseu
// Visualization of event horizon as perfect reflector

#version 460

uniform float time;
uniform vec2 resolution;
uniform float satoshi;

out vec4 fragColor;

const float PHI = 1.618033988749895;
const vec3 GOLD = vec3(1.0, 0.84, 0.0);
const vec3 VOID = vec3(0.005, 0.0, 0.01);

void main() {
    vec2 uv = (gl_FragCoord.xy * 2.0 - resolution) / min(resolution.x, resolution.y);
    float r = length(uv);
    float angle = atan(uv.y, uv.x);
    float t = time * 0.3;

    // Gravitational lensing
    float distortion = 1.0 / (r + 0.1);

    // Pulsating event horizon (Satoshi modulates radius)
    float horizon_radius = 0.5 + 0.01 * sin(t * satoshi);
    float event_horizon = smoothstep(horizon_radius, horizon_radius + 0.01, r);

    // Accretion disk (golden corona)
    float corona = 0.02 / abs(r - horizon_radius);
    vec3 disk_color = GOLD * corona * (1.0 + 0.5 * sin(angle * 12.0 + t));

    // Interior reflection (fractal)
    float reflection = 0.0;
    if (r < horizon_radius) {
        vec2 refl_uv = uv * distortion * 2.0;
        reflection = abs(sin(refl_uv.x * 20.0 + t) * cos(refl_uv.y * 20.0));
    }

    vec3 core_color = mix(VOID, GOLD * 0.5, reflection);

    vec3 final_color = mix(core_color, disk_color, event_horizon);

    // Satoshi glow
    final_color += GOLD * (satoshi / 10.0) * (1.0 - r) * 0.2;

    fragColor = vec4(final_color, 1.0);
}
