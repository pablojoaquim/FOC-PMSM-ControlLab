# ******************************************************************************
#
# @file config.py
#
# ******************************************************************************
# @copyright Copyright (c) 2026 - Pablo Joaquim
#             MIT License: https://opensource.org/licenses/MIT
# ******************************************************************************
#
# @section DESC DESCRIPTION:
#   Configuration loading and project-path resolution helpers.
#
# @section ABBR ABBREVIATIONS:
#   - FOC: Field-Oriented Control.
#
# @section TRACE TRACEABILITY INFO:
#   - Design Document(s):
#     - doc/foc_motor_control_technical.md
#
#   - Requirements Document(s):
#     - config.json
#
#   - Applicable Standards (in order of precedence: highest first):
#     - MIT License
#
# ******************************************************************************


# ******************************************************************************
# * import modules
# ******************************************************************************
import json
from pathlib import Path


# ******************************************************************************
# * Objects Declarations
# ******************************************************************************


# ******************************************************************************
# * Object and variables Definitions
# ******************************************************************************


# ******************************************************************************
# * Function Definitions
# ******************************************************************************
# ******************************************************************************
# * @fn         load_config
# * @brief      Load simulation configuration from the JSON file.
# * @param [in] config_path - Path to the configuration file.
# * @return     Configuration dictionary parsed from JSON.
# ******************************************************************************
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


# ******************************************************************************
# * @fn         get_project_paths
# * @brief      Build canonical project paths used by the application.
# * @param [in] project_root - Optional root path; inferred when None.
# * @return     Dictionary with resolved project paths.
# ******************************************************************************
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


# ******************************************************************************
# * @fn         load_config_with_paths
# * @brief      Load configuration and attach resolved netlist path.
# * @param [in] project_root - Optional root path; inferred when None.
# * @return     Tuple containing configuration and path dictionaries.
# ******************************************************************************
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
