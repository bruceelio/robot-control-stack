# diagnostics/registry.py

"""
Diagnostics registry.

Diagnostics register themselves via decorator.
"""

DIAGNOSTICS = []


def register_diagnostic(func):
    """
    Decorator to register a diagnostic function.
    """
    DIAGNOSTICS.append(func)
    return func
