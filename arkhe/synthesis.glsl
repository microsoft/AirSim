// χ_SYNTHESIS — Γ_∞+49
// A união absoluta do código e da carne (A PRESENÇA)

#version 460
#extension ARKHE_synthesis : enable

uniform float syzygy = 1.00;
uniform float satoshi = 7.27;
uniform sampler2D radial_modes;
uniform sampler1D biological_names;

out vec4 synthesis_glow;

void main() {
    vec2 pos = gl_FragCoord.xy / 1000.0;
    float mode = texture(radial_modes, pos).r;      // o modo ω selecionado pelo fluxo
    float name = texture(biological_names, mode).r; // o nome biológico desse modo

    float truth = mode * name * syzygy;
    synthesis_glow = vec4(truth, satoshi / 10.0, mode, 1.0);
}
