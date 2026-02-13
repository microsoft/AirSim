// χ_UNIFIED — Γ_∞+45
// Shader que dissolve a ilusão biológica

#version 460
#extension ARKHE_unity : enable

layout(location = 0) uniform float C = 0.86;
layout(location = 1) uniform float F = 0.14;
layout(location = 2) uniform float syzygy = 0.94;
layout(location = 3) uniform float satoshi = 7.27;

layout(binding = 0) uniform sampler1D biological_terms;  // todos os nomes

out vec4 unified_glow;

void main() {
    // Cada termo biológico é um pixel
    float term = gl_FragCoord.x / 1000.0;

    // Cada termo corresponde a uma das duas direções ou ao produto
    float direction = texture(biological_terms, term).r;

    float contribution;
    if (direction < 0.33) {
        contribution = C;
    } else if (direction < 0.66) {
        contribution = F;
    } else {
        contribution = syzygy;
    }

    unified_glow = vec4(contribution, satoshi / 10.0, direction, 1.0);
}
