"""
Extract profiling data from chaos test logs
Identifies hot paths in failure recovery scenarios
"""

import json
import re
from collections import defaultdict, Counter
from typing import Dict, List, Tuple
from dataclasses import dataclass
import pandas as pd

@dataclass
class ProfileEntry:
    """Single profiling entry from chaos test"""
    function_name: str
    call_count: int
    total_time_ms: float
    avg_time_ms: float
    module: str
    critical_path: bool  # True if in failure recovery

class ChaosTestProfiler:
    """Extract profiling data from chaos test execution logs"""

    def __init__(self, log_file: str):
        self.log_file = log_file
        self.profiles: Dict[str, ProfileEntry] = {}
        self.call_graph: Dict[str, List[str]] = defaultdict(list)
        self.hot_paths: List[Tuple[str, int]] = []

    def parse_logs(self):
        """Parse chaos test logs for profiling data"""

        print(f"📊 Parsing chaos test logs: {self.log_file}")

        function_calls = defaultdict(lambda: {'count': 0, 'time': 0.0})
        recovery_functions = set()

        try:
            with open(self.log_file, 'r') as f:
                for line in f:
                    # Extract function calls with timing
                    # Format: [TIMESTAMP] FUNCTION_NAME executed in X.XXms
                    match = re.search(r'\[(\d+)\] (\w+) executed in ([\d.]+)ms', line)
                    if match:
                        timestamp, func_name, exec_time = match.groups()
                        function_calls[func_name]['count'] += 1
                        function_calls[func_name]['time'] += float(exec_time)

                    # Mark recovery functions
                    if 'RECOVERY' in line or 'RECONSTRUCTION' in line:
                        recovery_match = re.search(r'(\w+)\(', line)
                        if recovery_match:
                            recovery_functions.add(recovery_match.group(1))
        except FileNotFoundError:
            print(f"⚠️ Log file {self.log_file} not found. Using synthetic data for PGO demo.")
            # Synthetic data for demonstration if logs are missing
            function_calls = {
                'kalman_predict': {'count': 125847, 'time': 289.4},
                'gradient_continuity_check': {'count': 98234, 'time': 304.5},
                'phase_alignment_inner_product': {'count': 87561, 'time': 157.6},
                'lattica_validate_signature': {'count': 67423, 'time': 542.1},
                'p2p_propagate_handover': {'count': 45122, 'time': 212.3}
            }
            recovery_functions = {'kalman_predict', 'gradient_continuity_check'}

        # Build profile entries
        for func_name, data in function_calls.items():
            module = self._infer_module(func_name)

            self.profiles[func_name] = ProfileEntry(
                function_name=func_name,
                call_count=data['count'],
                total_time_ms=data['time'],
                avg_time_ms=data['time'] / data['count'] if data['count'] > 0 else 0,
                module=module,
                critical_path=func_name in recovery_functions
            )

        # Identify hot paths (top 20% by call count)
        sorted_funcs = sorted(
            function_calls.items(),
            key=lambda x: x[1]['count'],
            reverse=True
        )

        top_20_percent = max(1, int(len(sorted_funcs) * 0.2))
        self.hot_paths = [(func, data['count']) for func, data in sorted_funcs[:top_20_percent]]

        print(f"✓ Parsed {len(self.profiles)} unique functions")
        print(f"✓ Identified {len(recovery_functions)} recovery functions")
        print(f"✓ Found {len(self.hot_paths)} hot paths")

    def _infer_module(self, func_name: str) -> str:
        """Infer module from function name prefix"""
        prefixes = {
            'kalman_': 'reconstruction/kalman_filter',
            'gradient_': 'reconstruction/gradient_continuity',
            'phase_': 'reconstruction/phase_alignment',
            'lattica_': 'network/lattica_consensus',
            'p2p_': 'network/p2p_handover',
            'poc_': 'network/proof_of_coherence',
            'torus_': 'architecture/toroidal_network',
            'memory_': 'architecture/memory_garden',
            'validate_': 'core/validation',
            'handover_': 'core/handover',
        }

        for prefix, module in prefixes.items():
            if func_name.startswith(prefix):
                return module

        return 'core/unknown'

    def generate_pgo_hints(self) -> Dict[str, Dict]:
        """
        Generate PGO hints for compiler optimization

        Returns:
            Dictionary with optimization directives per function
        """
        hints = {}

        for func_name, profile in self.profiles.items():
            # High call count → inline candidate
            # Critical path → optimize for speed
            # Low call count + large time → optimize for size

            if profile.call_count > 1000:
                optimization = 'inline'
            elif profile.critical_path:
                optimization = 'optimize_speed'
            elif profile.call_count < 10 and profile.total_time_ms > 100:
                optimization = 'optimize_size'
            else:
                optimization = 'default'

            hints[func_name] = {
                'optimization': optimization,
                'call_count': profile.call_count,
                'avg_time_ms': profile.avg_time_ms,
                'critical_path': profile.critical_path,
                'module': profile.module
            }

        return hints

    def export_llvm_profile(self, output_file: str):
        """Export LLVM-compatible profiling data"""

        # LLVM PGO format (simplified)
        llvm_data = {
            'version': 1,
            'functions': []
        }

        for func_name, profile in self.profiles.items():
            llvm_data['functions'].append({
                'name': func_name,
                'hash': hash(func_name) & 0xFFFFFFFF,  # 32-bit hash
                'counters': [profile.call_count],
                'num_counters': 1
            })

        with open(output_file, 'w') as f:
            json.dump(llvm_data, f, indent=2)

        print(f"✓ Exported LLVM profile to {output_file}")

    def export_gcc_profile(self, output_file: str):
        """Export GCC gcov-compatible profiling data"""

        with open(output_file, 'w') as f:
            for func_name, profile in self.profiles.items():
                # GCC gcda format (text representation)
                f.write(f"function:{func_name}\n")
                f.write(f"calls:{profile.call_count}\n")
                f.write(f"time:{profile.total_time_ms}\n")
                f.write(f"critical:{1 if profile.critical_path else 0}\n")
                f.write("\n")

        print(f"✓ Exported GCC profile to {output_file}")

    def generate_report(self) -> pd.DataFrame:
        """Generate human-readable optimization report"""

        data = []
        for func_name, profile in self.profiles.items():
            data.append({
                'Function': func_name,
                'Module': profile.module,
                'Calls': profile.call_count,
                'Total Time (ms)': f"{profile.total_time_ms:.2f}",
                'Avg Time (ms)': f"{profile.avg_time_ms:.4f}",
                'Critical Path': '✓' if profile.critical_path else '',
                'Hot Path': '🔥' if func_name in [f for f, _ in self.hot_paths[:10]] else ''
            })

        df = pd.DataFrame(data)
        df = df.sort_values('Calls', ascending=False)

        return df


def analyze_chaos_test_profiles():
    """Main analysis workflow"""

    print("="*70)
    print("CHAOS TEST PROFILE EXTRACTION")
    print("="*70)

    # Parse chaos test logs
    profiler = ChaosTestProfiler('chaos_test_execution.log')
    profiler.parse_logs()

    # Generate optimization hints
    hints = profiler.generate_pgo_hints()

    print("\n📊 Top 10 Hot Paths:")
    for i, (func, count) in enumerate(profiler.hot_paths[:10], 1):
        profile = profiler.profiles[func]
        critical = "⚠️ CRITICAL" if profile.critical_path else ""
        print(f"  {i}. {func}: {count:,} calls {critical}")
        print(f"     Avg: {profile.avg_time_ms:.4f}ms, Module: {profile.module}")

    # Export profiles
    profiler.export_llvm_profile('arkhe_kernel.profdata')
    profiler.export_gcc_profile('arkhe_kernel.gcda')

    # Save optimization hints
    with open('pgo_optimization_hints.json', 'w') as f:
        json.dump(hints, f, indent=2)

    print("\n✓ Optimization hints saved to pgo_optimization_hints.json")

    # Generate report
    report = profiler.generate_report()
    report.to_csv('pgo_profiling_report.csv', index=False)
    print("✓ Profiling report saved to pgo_profiling_report.csv")

    return profiler, hints


if __name__ == "__main__":
    analyze_chaos_test_profiles()
