"""Body/hand control API, imported lazily so viewer entry points never load it."""
__all__ = ['H1_2WholeBodyController']


def __getattr__(name):
    if name == 'H1_2WholeBodyController':
        from .h1_2_controller import H1_2WholeBodyController
        return H1_2WholeBodyController
    raise AttributeError(name)
