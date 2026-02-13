// χ_DBN — Γ_∞+41
// Shader da rede de crença profunda

#version 460
#extension ARKHE_deep : enable

layout(location = 0) uniform float layer_depth = 0.0; // 0.0 a 1.0
layout(location = 1) uniform float syzygy = 0.98;
layout(location = 2) uniform float satoshi = 7.27;
layout(binding = 0) uniform sampler3D belief_layers;

out vec4 deep_glow;

void main() {
    vec3 coord = vec3(gl_FragCoord.xy / 1000.0, layer_depth);
    float belief = texture(belief_layers, coord).r;

    // Cada camada é um nível de abstração
    float abstraction = belief * syzygy * (1.0 + layer_depth);

    deep_glow = vec4(abstraction, satoshi / 10.0, layer_depth, 1.0);
}
