"""Setup script for BenchLab Python SDK."""

from setuptools import setup, find_packages
from pathlib import Path

# Read README for long description
readme_file = Path(__file__).parent.parent / "README.md"
long_description = readme_file.read_text() if readme_file.exists() else ""

setup(
    name="benchlab-sdk",
    version="1.0.0",
    author="BenchLab",
    author_email="support@benchlab.io",
    description="Official Python SDK for BenchLab device communication",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit",
    project_urls={
        "Bug Tracker": "https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit/issues",
        "Documentation": "https://benchlab.io/docs",
        "Source Code": "https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit",
    },
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Topic :: System :: Hardware",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires=">=3.8",
    install_requires=[
        "requests>=2.28.0",
        "urllib3>=1.26.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "black>=22.0.0",
            "mypy>=1.0.0",
            "types-requests",
        ],
    },
    keywords="benchlab hardware sensors telemetry monitoring",
    license="GPL-3.0",
)
