#!/usr/bin/env python3
"""
Configuration management for FOC simulation.

Loads and validates simulation config from JSON file.
"""

import json
from pathlib import Path


def load_config(config_path):
    """
    Load configuration from JSON file.

    Args:
        config_path: Path to config.json file.

    Returns:
        Configuration dictionary.

    Raises:
        FileNotFoundError: If config file does not exist.
        json.JSONDecodeError: If config file is invalid JSON.
    """
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f'Config file not found: {config_path}')

    with open(config_path, 'r') as f:
        config = json.load(f)

    return config


def get_project_paths(project_root=None):
    """
    Get standard project paths.

    Args:
        project_root: Project root directory. If None, inferred from this module.

    Returns:
        Dictionary with 'project_root', 'config', 'ltspice', 'src', 'doc', 'tst' paths.
    """
    if project_root is None:
        project_root = Path(__file__).resolve().parent.parent

    project_root = Path(project_root)

    return {
        'project_root': project_root,
        'config': project_root / 'config.json',
        'ltspice': project_root / 'ltspice',
        'src': project_root / 'src',
        'doc': project_root / 'doc',
        'tst': project_root / 'tst',
    }


def load_config_with_paths(project_root=None):
    """
    Load configuration and return config with resolved netlist path.

    Args:
        project_root: Project root directory. If None, inferred from this module.

    Returns:
        Tuple of (config, paths) dictionaries.
    """
    paths = get_project_paths(project_root)
    config = load_config(paths['config'])

    # Resolve netlist path relative to project root
    netlist_rel = config['ltspice']['netlist_path']
    config['_netlist_path_resolved'] = paths['project_root'] / netlist_rel

    return config, paths
