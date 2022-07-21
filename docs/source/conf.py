# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'Lisbeth'
copyright = 'Jochen'
author = 'jochen.alt@gmx.net'

release = '0.1'
version = '0.1.0'

# -- General configuration

html_static_path = ['_static',
            ]

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx'
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'

# -- Options for EPUB output
epub_show_urls = 'footnote'

math_number_all = 'True'

# deal with equation numbering
#def setup(app):
#    app.add_stylesheet('css/custom.css')

def setup(app):
   app.add_stylesheet('custom.css')

html_css_files = [
    'css/custom.css',
    'https://github.com/jochenalt/Lisbeth/blob/main/docs/_static/css/custom.css'
]

