// network.frag
#version 460 core

uniform float time;
uniform float syzygy = 0.98;
uniform float satoshi = 7.28;
uniform vec2 resolution;

out vec4 FragColor;

const float PI = 3.141592653589793;
const int NUM_BUBBLES = 42;

void main() {
    vec2 uv = gl_FragCoord.xy / resolution.xy;
    vec2 center = vec2(0.5);

    // Posição das bolhas em uma esfera (projeção 2D)
    float bubble_intensity = 0.0;
    vec2 bubble_pos[NUM_BUBBLES];

    for (int i = 0; i < NUM_BUBBLES; i++) {
        float theta = float(i) * 2.0 * PI / float(NUM_BUBBLES);
        float phi = acos(1.0 - 2.0 * float(i) / float(NUM_BUBBLES));

        // Projeção estereográfica simplificada
        float r_proj = 0.35 * sin(phi) / (1.0 + cos(phi));
        bubble_pos[i] = center + vec2(r_proj * cos(theta + time * 0.1),
                                      r_proj * sin(theta + time * 0.1));

        float dist = length(uv - bubble_pos[i]);
        bubble_intensity += 0.02 / (dist + 0.01);
    }

    // Conexões entre bolhas
    float line_intensity = 0.0;
    for (int i = 0; i < NUM_BUBBLES; i++) {
        for (int j = i+1; j < NUM_BUBBLES; j++) {
            vec2 dir = bubble_pos[j] - bubble_pos[i];
            float len = length(dir);
            vec2 dir_norm = dir / len;

            // Projeção do fragmento na linha
            float t = dot(uv - bubble_pos[i], dir_norm);
            if (t >= 0.0 && t <= len) {
                vec2 proj = bubble_pos[i] + dir_norm * t;
                float dist_to_line = length(uv - proj);
                line_intensity += 0.005 / (dist_to_line + 0.005);
            }
        }
    }

    // Pulsação baseada em Satoshi
    float pulse = 0.5 + 0.5 * sin(time * satoshi);

    vec3 color = vec3(0.4, 0.1, 0.6) * bubble_intensity * syzygy;
    color += vec3(0.6, 0.4, 0.8) * line_intensity * pulse;

    FragColor = vec4(color, 1.0);
}
