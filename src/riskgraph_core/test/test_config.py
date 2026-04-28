import textwrap
from riskgraph_core.config import load_config, Config


def test_load_minimal_config(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text(textwrap.dedent("""
        weights:
          geometry: 1.0
          semantic: 2.0
          risk: 3.0
        decay_half_life_s: 600.0
        store_path: ":memory:"
    """).strip())
    cfg = load_config(str(p))
    assert isinstance(cfg, Config)
    assert cfg.weights.geometry == 1.0
    assert cfg.weights.semantic == 2.0
    assert cfg.weights.risk == 3.0
    assert cfg.weights.decay_half_life_s == 600.0
    assert cfg.store_path == ":memory:"


def test_load_config_uses_defaults_when_missing(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("store_path: /tmp/x.sqlite\n")
    cfg = load_config(str(p))
    assert cfg.store_path == "/tmp/x.sqlite"
    # Defaults are non-zero so a partial config still produces a working scorer.
    assert cfg.weights.geometry > 0.0
    assert cfg.weights.risk > 0.0
