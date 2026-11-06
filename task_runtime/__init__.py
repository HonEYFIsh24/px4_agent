"""Staged task ownership above existing perception and flight skills."""
__all__ = ['TaskOrchestrator']


def __getattr__(name):
    # Evidence utilities are also imported by the legacy semantic map. Do not
    # import the orchestration/model stack just to compare two observations.
    if name == 'TaskOrchestrator':
        from .orchestrator import TaskOrchestrator
        return TaskOrchestrator
    raise AttributeError(name)
