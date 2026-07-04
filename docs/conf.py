"""Sphinx configuration for the state-space control framework docs.

Built by .github/workflows/docs.yml and deployed to GitHub Pages.
The Python packages are imported straight from ../packages/<pkg> (no
installation needed); ROS 2 and Pinocchio imports are mocked so the API
reference builds on any plain-Python machine.
"""

import os
import sys

# -- Path setup: make the four Python packages importable ---------------------

_HERE = os.path.dirname(__file__)
for _pkg in ('urdf_state_space', 'state_space_control',
             'state_space_setup_assistant', 'state_space_response_viz'):
    sys.path.insert(0, os.path.abspath(
        os.path.join(_HERE, '..', 'packages', _pkg)))

# -- Project information -------------------------------------------------------

project = "Kontrol'Em"
author = 'Rahgir Rafi'
copyright = '2026, Rahgir Rafi'
release = '1.0.0'

# -- General configuration -----------------------------------------------------

extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
]

source_suffix = {'.rst': 'restructuredtext', '.md': 'markdown'}
myst_enable_extensions = ['colon_fence', 'deflist', 'fieldlist', 'dollarmath']
myst_heading_anchors = 3

templates_path = []
exclude_patterns = ['_build', 'README.md', 'Thumbs.db', '.DS_Store']

# -- Autodoc -------------------------------------------------------------------

# Heavy / ROS-only dependencies are mocked: the API reference documents
# signatures and docstrings, it never runs the dynamics or a ROS graph.
autodoc_mock_imports = [
    'pinocchio',
    'rclpy',
    'rcl_interfaces',
    'std_msgs',
    'std_srvs',
    'sensor_msgs',
    'geometry_msgs',
    'tf2_ros',
    'ament_index_python',
    'xacro',
    'control',
    'slycot',
]
autodoc_member_order = 'bysource'
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'scipy': ('https://docs.scipy.org/doc/scipy/', None),
}

# -- HTML output ----------------------------------------------------------------

html_theme = 'furo'
html_title = "Kontrol'Em — state-space control for ROS 2"
html_static_path = []
html_theme_options = {
    'source_repository': 'https://github.com/rahgirrafi/kontrolem/',
    'source_branch': 'main',
    'source_directory': 'docs/',
}
