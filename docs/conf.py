# -- Path setup --------------------------------------------------------------

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'library')))


# -- Project information -----------------------------------------------------

project = 'uav-neo-library'
copyright = '2026, MIT.'
author = 'BWSI UAV Neo Drone'


# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'myst_parser'
]

autodoc_mock_imports = [
    'numpy',
    'nptyping',
    'cv2',
    'pandas',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

pygments_style = 'default'
pygments_style_dark = 'lightbulb'

# -- Options for HTML output -------------------------------------------------

html_permalinks_icon = '<span>#</span>'
html_theme = 'sphinxawesome_theme'
html_static_path = ['_static']
