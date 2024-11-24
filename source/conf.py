# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ROS2 Basics'
copyright = '2024, Luca Sidoti Pinto'
author = 'Luca Sidoti Pinto'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx_rtd_theme', 'sphinx_copybutton', 'sphinx_code_tabs', 'sphinx_new_tab_link', 'sphinx_togglebutton']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'logo_only': False,
    'collapse_navigation': False,
    'sticky_navigation': True,
    'includehidden': True,
    'navigation_depth': 2,
    'titles_only': False
}
html_static_path = ['_static', 'downloads']
html_css_files = ['custom.css']
html_js_files = ['custom.js']
