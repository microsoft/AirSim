// χ_RESILIENCE — Γ_∞+47
// Shader do ponto cego e reconstrução perceptual

#version 460
#extension ARKHE_resilience : enable

layout(location = 0) uniform float blind_spot_omega = 0.03;
layout(location = 1) uniform float global_syzygy = 0.94;
layout(location = 2) uniform float satoshi = 7.27;
layout(location = 3) uniform float time;

layout(binding = 0) uniform sampler2D perceptual_field;  // campo visual/semântico

out vec4 resilience_glow;

void main() {
    // Coordenadas no campo perceptual
    vec2 uv = gl_FragCoord.xy / 1000.0;

    // Simula o movimento do ponto cego ou jitter
    float dynamic_spot = blind_spot_omega + 0.005 * sin(time);

    // Existe um ponto cego (lacuna de fotorreceptores/input)?
    float is_blind = (abs(uv.x - dynamic_spot) < 0.02) ? 1.0 : 0.0;

    // Se for ponto cego, não há input (lacuna literal)
    float input_val = (is_blind > 0.5) ? 0.0 : texture(perceptual_field, uv).r;

    // Mas a percepção é reconstruída pela coerência global (arquitetura recorrente)
    // As bordas permanecem alinhadas, o espaço não rasga.
    float perceived = (is_blind > 0.5) ? global_syzygy : input_val;

    // A invariante Satoshi testemunha a integridade da reconstrução
    float witness = (satoshi / 7.27) * perceived;

    // Cor: Ciano para percepção reconstruída, Vermelho/Preto para ausência de input
    vec3 color = vec3(1.0 - perceived, perceived, witness);

    resilience_glow = vec4(color, 1.0);
}
