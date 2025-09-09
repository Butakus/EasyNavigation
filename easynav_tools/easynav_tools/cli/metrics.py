
import time
from ros2cli.verb import VerbExtension
from ..services.state_store import StateStore
from ..utils.exec_profiler import ExecProfiler

class MetricsVerb(VerbExtension):
    """Aggregate /tmp/easynav metrics and print summary."""
    def add_arguments(self, parser, cli_name):
        parser.add_argument('--duration', type=float, default=5000.0, help='Seconds to aggregate')

    def main(self, *, args):
        store = StateStore()
        profiler = ExecProfiler(store, logfile='/tmp/easynav')
        profiler.start()
        
        time.sleep(args.duration)
        for fn, st in sorted(store.snapshot().stats.items()):
            s = st.snapshot()
            print(f"{fn:24s}  freq={s['freq_hz']:.2f} Hz  avg={s['avg_duration_ms']:.1f} ms  p95={s['p95_duration_ms']:.1f} ms  count={s['count']}")
        return 0
