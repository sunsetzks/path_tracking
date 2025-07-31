from setuptools import setup, find_packages

def read(filename):
    import os
    path = os.path.join(os.path.dirname(__file__), filename)
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()

setup(
    name="pure_python_dubins",
    version="1.0.0",
    description="Pure Python implementation of Dubins path planning algorithms",
    long_description=read('README.md'),
    long_description_content_type="text/markdown",
    author="Andrew Walker (original), Roo (pure Python rewrite)",
    author_email="walker.ab@gmail.com",
    url="https://github.com/AndrewWalker/pydubins",
    license="MIT",
    packages=find_packages(),
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
        'Operating System :: OS Independent',
        'Topic :: Scientific/Engineering :: Mathematics',
        'Topic :: Scientific/Engineering :: Physics',
    ],
    python_requires='>=3.7',
    install_requires=[
        # No external dependencies required - pure Python implementation
    ],
    extras_require={
        'dev': [
            'pytest>=6.0',
            'pytest-cov>=2.0',
            'flake8>=3.8',
            'black>=21.0',
        ],
    },
    keywords='dubins path planning robotics autonomous vehicles',
    project_urls={
        'Bug Reports': 'https://github.com/AndrewWalker/pydubins/issues',
        'Source': 'https://github.com/AndrewWalker/pydubins',
        'Documentation': 'https://github.com/AndrewWalker/pydubins/blob/main/README.rst',
    },
)