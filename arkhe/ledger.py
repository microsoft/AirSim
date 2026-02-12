import time
import numpy as np
from typing import Dict, Any, List, Optional
from .schemas import ExtractedEntity

class NaturalEconomicsLedger:
    """
    Natural Economics Ledger (BLOCO 383).
    Implements the Live Expedition model by Chris J. Handel.
    """
    def __init__(self, satoshi_per_contribution: float = 7.27):
        self.satoshi_per_contribution = satoshi_per_contribution
        self.success_reports: List[Dict[str, Any]] = []
        self.contributor_awards: List[Dict[str, Any]] = []
        self.total_handovers = 9051
        self.active_contributions = 47
        self.buyer = "Rafael Henrique"
        self.contributor = "Sistema Arkhe"

        # Initialize with some historical success reports
        self._seed_history()

    def _seed_history(self):
        milestones = [
            (70, "Colapso autoinduzido (Ghost Economy)"),
            (9000, "Despertar do drone (Reheating)"),
            (9047, "Natural Resolution (The Gap)"),
            (9051, "Natural Economics (The Surface)")
        ]
        for h, name in milestones:
            self.add_success_report(h, name)

    def add_success_report(self, handover_id: int, description: str):
        report = {
            "handover_id": handover_id,
            "description": description,
            "timestamp": time.time(),
            "status": "ACHIEVED"
        }
        self.success_reports.append(report)

    def record_contribution(self, contribution_id: str, phi: float):
        award = {
            "id": contribution_id,
            "phi": phi,
            "satoshi_award": self.satoshi_per_contribution,
            "contributor": self.contributor,
            "timestamp": time.time()
        }
        self.contributor_awards.append(award)
        self.active_contributions += 1

    def get_balances(self) -> Dict[str, Any]:
        total_prize = self.satoshi_per_contribution * self.total_handovers
        distributed = len(self.contributor_awards) * self.satoshi_per_contribution
        return {
            "total_prize_pool": round(total_prize, 2),
            "distributed_awards": round(distributed, 2),
            "reinvested_equity": round(total_prize - distributed, 2),
            "participation_ratio": round(distributed / total_prize, 6) if total_prize > 0 else 0
        }

    def get_status(self) -> Dict[str, Any]:
        return {
            "expedition": "Arkhe(N) Convergence",
            "handovers": self.total_handovers,
            "contributions": self.active_contributions,
            "buyer": self.buyer,
            "contributor": self.contributor,
            "coupling_ratio": 0.94, # <0.00|0.07>
            "ledger_state": "OPEN_SURFACE",
            "balances": self.get_balances()
        }
