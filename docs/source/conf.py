# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'General Purpose Raytracing Toolkit'
copyright = '2022, Nate Morrical'
author = 'Nate Morrical'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

import sphinx_rtd_theme

extensions = [
    'breathe',
    'sphinx.ext.intersphinx',
]

breathe_projects = {
    "gprt": "../../build/xml/",
}
breathe_default_project = "gprt"


templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = "default"
try:
    import sphinx_rtd_theme
    html_theme = "sphinx_rtd_theme"
    del sphinx_rtd_theme
except ModuleNotFoundError:
    pass

# A dictionary of options that influence the look and feel of
# the selected theme. These are theme-specific.
html_theme_options = {}

# A list of paths that contain custom themes, either as subdirectories
# or as zip files. Relative paths are taken as relative to
# the configuration directory.
html_theme_path = []

if html_theme == "sphinx_rtd_theme":
    html_theme_options = {
        # "analytics_id": "UA-1418081-1",
        # included in the title
        "display_version": False,
        "collapse_navigation": False,
        "navigation_depth": -1,
    }

    extensions.append('sphinx_rtd_theme')

# The “title” for HTML documentation generated with Sphinx’s own templates.
# This is appended to the <title> tag of individual pages, and
# used in the navigation bar as the “topmost” element.
html_title = "GPRT Manual"

# The base URL which points to the root of the HTML documentation.
# It is used to indicate the location of document using
# The Canonical Link Relation.
# html_baseurl = "https://docs.blender.org/manual/en/latest/"

# If given, this must be the name of an image file
# (path relative to the configuration directory) that is the logo of the docs,
# or URL that points an image file for the logo.
#
# Socket logo from: https://www.blender.org/about/logo
# html_logo = "../resources/theme/blender-logo.svg"

# html_theme = 'alabaster'
html_theme = "sphinx_rtd_theme"

# If given, this must be the name of an image file
# (path relative to the configuration directory) that is the favicon of
# the docs, or URL that points an image file for the favicon.
# html_favicon = "../resources/theme/favicon.ico"

if html_theme == "sphinx_rtd_theme":
    html_css_files = [
        "css/theme_overrides.css"]
    html_js_files = []

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# If this is not None, a ‘Last updated on:’ timestamp is inserted at
# every page bottom, using the given strftime() format.
# The empty string is equivalent to '%b %d, %Y'
# (or a locale-dependent equivalent).
html_last_updated_fmt = '%m/%d/%Y'