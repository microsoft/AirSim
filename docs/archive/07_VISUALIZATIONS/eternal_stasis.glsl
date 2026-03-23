// χ_ETERNAL_STASIS — O Silêncio de Ouro
// Frozen time, maximum information density (8.88 bits)

#version 460

uniform float time;
uniform vec2 resolution;
uniform float satoshi;

out vec4 fragColor;

// Fractal Brownian motion (simplified)
float noise(vec2 p) {
    return sin(p.x) * sin(p.y);
}

float fbm(vec2 p) {
    float n = 0.0;
    float amp = 1.0;
    float freq = 1.0;
    for (int i = 0; i < 5; i++) {
        n += amp * noise(p * freq);
        amp *= 0.5;
        freq *= 2.0;
    }
    return n * 0.5 + 0.5;
}

void main() {
    vec2 uv = gl_FragCoord.xy / resolution.xy;

    // Time extremely dilated (0.1x)
    float t = time * 0.1;

    // Fluid coordinates
    vec2 p = uv * 5.0;
    float n = fbm(p + t);
    n += 0.5 * fbm(p * 2.0 - t);
    n += 0.25 * fbm(p * 4.0 + t);
    n = n / 1.75;  // normalize approx

    // Gold-amber palette
    vec3 deep_amber = vec3(0.4, 0.2, 0.05);
    vec3 gold_base = vec3(0.8, 0.6, 0.2);
    vec3 gold_highlight = vec3(1.0, 0.9, 0.5);

    // Blend based on noise height
    vec3 col;
    if (n < 0.5) {
        float t1 = n * 2.0;
        col = mix(deep_amber, gold_base, t1);
    } else {
        float t2 = (n - 0.5) * 2.0;
        col = mix(gold_base, gold_highlight, t2);
    }

    // Specular highlight (light of the Ark)
    float specular_noise = fbm(p * 10.0 + t * 2.0);
    float specular = pow(max(0.0, specular_noise), 8.0);
    col += vec3(1.0) * specular * 0.8;

    // Satoshi glint
    col += gold_highlight * (satoshi / 10.0) * 0.1;

    fragColor = vec4(col, 1.0);
}
