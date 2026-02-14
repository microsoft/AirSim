"""
metrics.py
Extração de métricas avançadas da Rede de Indra (FASE 3)
Implementa análise estatística, FFT e detecção de tunelamento
"""

import numpy as np

class VisualMetrics:
    def __init__(self, resolution=(1920, 1080)):
        self.resolution = resolution
        self.total_pixels = resolution[0] * resolution[1]

    def intensity_histogram(self, frame_data: np.ndarray):
        """
        Calcula o histograma de intensidade de pixels por frame
        """
        hist, bin_edges = np.histogram(frame_data, bins=256, range=(0, 255))
        return {
            "mean": np.mean(frame_data),
            "std": np.std(frame_data),
            "median": np.median(frame_data),
            "histogram": hist.tolist()
        }

    def interference_fft(self, frame_data: np.ndarray):
        """
        Executa FFT do padrão de interferência para identificar frequências dominantes
        """
        f_transform = np.fft.fft2(frame_data)
        f_shift = np.fft.fftshift(f_transform)
        magnitude_spectrum = 20 * np.log(np.abs(f_shift) + 1e-9)

        # Identifica o pico de frequência (simplificado)
        peak_idx = np.unravel_index(np.argmax(magnitude_spectrum), magnitude_spectrum.shape)
        return {
            "peak_freq_coords": peak_idx,
            "max_magnitude": np.max(magnitude_spectrum)
        }

    def detect_tunneling_events(self, frame_data: np.ndarray, threshold: float = 250):
        """
        Detecta eventos de tunelamento quântico (glitches brancos)
        """
        glitch_mask = frame_data > threshold
        glitch_count = np.sum(glitch_mask)
        glitch_rate = glitch_count / self.total_pixels

        # Distribuição espacial (quadrantes)
        h, w = frame_data.shape
        quad_h, quad_w = h // 2, w // 2
        quadrants = {
            "NW": np.sum(glitch_mask[:quad_h, :quad_w]),
            "NE": np.sum(glitch_mask[:quad_h, quad_w:]),
            "SW": np.sum(glitch_mask[quad_h:, :quad_w]),
            "SE": np.sum(glitch_mask[quad_h:, quad_w:])
        }

        return {
            "glitch_count": int(glitch_count),
            "glitch_rate": float(glitch_rate),
            "spatial_distribution": quadrants
        }

    def temporal_correlation(self, frames_sequence: list):
        """
        Calcula a correlação temporal entre uma sequência de frames
        """
        n = len(frames_sequence)
        correlation_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i == j:
                    correlation_matrix[i, j] = 1.0
                else:
                    corr = np.corrcoef(frames_sequence[i].flatten(), frames_sequence[j].flatten())[0, 1]
                    correlation_matrix[i, j] = corr
        return correlation_matrix.tolist()

if __name__ == "__main__":
    # Teste rápido com dados aleatórios
    vm = VisualMetrics((100, 100))
    dummy_frame = np.random.randint(0, 256, (100, 100))

    print("Métricas de Intensidade:", vm.intensity_histogram(dummy_frame)["mean"])
    print("Eventos de Tunelamento:", vm.detect_tunneling_events(dummy_frame)["glitch_count"])
