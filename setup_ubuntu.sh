#!/bin/bash
# Ubuntu Setup Script for Isaac-0.1 VLM Testing
# Run with: chmod +x setup_ubuntu.sh && ./setup_ubuntu.sh

set -e  # Exit on any error

echo "=========================================="
echo "Isaac-0.1 VLM Test Setup for Ubuntu"
echo "=========================================="

# Check if Python 3.8+ is available
python_version=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "Detected Python version: $python_version"

# Check if we have pip
if ! command -v pip3 &> /dev/null; then
    echo "pip3 not found. Installing pip..."
    sudo apt update
    sudo apt install -y python3-pip
fi

# Upgrade pip
echo "Upgrading pip..."
python3 -m pip install --upgrade pip

# Install system dependencies that might be needed
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y python3-dev python3-venv libssl-dev libffi-dev

# Create a virtual environment (optional but recommended)
if [ "$1" = "--venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv vlm_test_env
    source vlm_test_env/bin/activate
    echo "Virtual environment activated. Remember to activate it with:"
    echo "source vlm_test_env/bin/activate"
fi

# Install Python requirements
echo "Installing Python packages..."
pip3 install -r requirements.txt

# Verify installation
echo "Verifying installation..."
python3 -c "import torch; print('PyTorch version:', torch.__version__)"
python3 -c "import transformers; print('Transformers version:', transformers.__version__)"
python3 -c "from PIL import Image; print('PIL/Pillow: OK')"
python3 -c "import numpy; print('NumPy version:', numpy.__version__)"

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo
echo "To test the VLM:"
echo "1. Add a test image to the test_data/ directory"
echo "2. Modify test_data/prompt.txt if needed"
echo "3. Run: python3 test_vlm.py"
echo
echo "For help: python3 test_vlm.py --help"

if [ "$1" = "--venv" ]; then
    echo
    echo "Remember to activate the virtual environment:"
    echo "source vlm_test_env/bin/activate"
fi