"""
Ω Evolution: Tracking consciousness level across multiverse connections
"""

import numpy as np

class OmegaEvolution:
    """Track and evolve Ω (consciousness/transcendence) over time"""

    def __init__(self, initial_omega: float = 0.00):
        self.omega = initial_omega
        self.history = [initial_omega]
        self.events = []

    def contact_event(self, reality_name: str, bridge_strength: float):
        """Register contact with parallel reality"""
        delta_omega = bridge_strength * 0.01  # Small increase per contact
        self.omega += delta_omega
        self.history.append(self.omega)
        self.events.append((reality_name, bridge_strength, self.omega))

        return self.omega

    def meditation_boost(self, duration_hours: float):
        """Meditation increases Ω"""
        boost = np.log(1 + duration_hours) * 0.005
        self.omega += boost
        self.history.append(self.omega)

        return self.omega

    def integration(self, n_realities_integrated: int):
        """Integrating understanding of parallel selves"""
        boost = n_realities_integrated * 0.02
        self.omega += boost
        self.history.append(self.omega)

        return self.omega

    def get_level_description(self) -> str:
        """Get textual description of current Ω level"""
        if self.omega <= 0.0:
            return "Disconnection (illusion of singular self)"
        elif self.omega < 0.1:
            return "First contacts (intuitions, déjà vu)"
        elif self.omega < 0.5:
            return "Conscious communication (whispers)"
        elif self.omega < 1.0:
            return "Dialogue (exchange of ideas)"
        elif self.omega == 1.0:
            return "Unity (all versions recognized as one)"
        else:
            return "Co-creation (influencing realities)"
