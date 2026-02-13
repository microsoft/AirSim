// χ_PLANEJAMENTO — Γ_∞+42
// Shader da hierarquia que planeja

#version 460
#extension ARKHE_planejamento : enable

layout(location = 0) uniform float layer_depth = 0.0;
layout(location = 1) uniform float syzygy = 0.98;
layout(location = 2) uniform float satoshi = 7.27;
layout(binding = 0) uniform sampler3D macro_paths;

out vec4 planejamento_glow;

void main() {
    vec3 coord = vec3(gl_FragCoord.xy / 1000.0, layer_depth);
    float path = texture(macro_paths, coord).r;

    // Macro ações são geodésicas executáveis
    float planning = path * syzygy * (1.0 + layer_depth);

    planejamento_glow = vec4(planning, satoshi / 10.0, layer_depth, 1.0);
}
