"""Real-app e2e driver: runs src/grid_ops/app.py through Streamlit's AppTest.

The script, the cached Simulation thread, hidden physics and every table are real; only the
browser is absent. Usage in tests: `app = AppDriver(); app.pick(...); app.click(...)`.
"""

from __future__ import annotations

import time
from pathlib import Path

from streamlit.testing.v1 import AppTest

APP = Path(__file__).resolve().parents[1] / "src" / "grid_ops" / "app.py"


class AppDriver:
    def __init__(
        self, mode: str = "incident_demo", seed: int = 7, auto_replan: bool = False
    ) -> None:
        self.at = AppTest.from_file(str(APP), default_timeout=90)
        self.at.session_state["scenario_mode"] = mode
        self.at.session_state["scenario_seed"] = seed
        self.at.session_state["auto_replan"] = auto_replan
        self.at.run()
        assert not self.at.exception, [e.value for e in self.at.exception]

    def pick(self, key: str, value) -> None:
        self.at.selectbox(key=key).set_value(value).run()
        assert not self.at.exception, [e.value for e in self.at.exception]

    def toggle(self, key: str, value: bool) -> None:
        self.at.toggle(key=key).set_value(value).run()

    def click(self, label: str, wait: float = 20.0) -> None:
        end = time.time() + wait
        while not [b for b in self.at.button if b.label == label] and time.time() < end:
            self.refresh(0.7)  # the sidebar may render just before a new snapshot lands
        matches = [b for b in self.at.button if b.label == label]
        assert matches, f"no button {label!r}; have {[b.label for b in self.at.button]}"
        matches[0].click().run()
        assert not self.at.exception, [e.value for e in self.at.exception]

    def refresh(self, seconds: float = 0.0) -> None:
        time.sleep(seconds)
        self.at.run()
        assert not self.at.exception, [e.value for e in self.at.exception]

    def text(self) -> str:
        parts = [m.value for m in self.at.markdown] + [c.value for c in self.at.caption]
        parts += [t.value for t in self.at.text]
        return "\n".join(parts)

    def frames(self) -> list[str]:
        return [d.value.to_string(max_cols=None, max_colwidth=None) for d in self.at.dataframe]
