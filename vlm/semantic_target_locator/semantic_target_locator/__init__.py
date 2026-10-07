"""Interface-first semantic target localization package."""

from .contracts import TargetQuery
from .locator import LocatorConfig, SemanticTargetLocator

__all__ = ["LocatorConfig", "SemanticTargetLocator", "TargetQuery"]
