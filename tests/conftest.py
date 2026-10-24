import warnings

from grid_ops.knowledge import LandingPolicy

warnings.filterwarnings("ignore")


def permissive(sim):
    """Opt a test simulation into uncertain endpoints so earlier tickets' checks stay focused."""
    sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
    return sim
