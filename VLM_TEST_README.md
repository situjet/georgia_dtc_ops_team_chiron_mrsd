# Isaac-0.1 VLM Test Setup (CPU-only Ubuntu)

This directory contains a CPU-optimized test script for the Isaac-0.1 Vision-Language Model, designed for Ubuntu deployment.

## Files
- `test_vlm.py` - Main test script for the VLM
- `requirements.txt` - Python dependencies
- `test_data/` - Directory for test images and prompts
- `Isaac-0.1/` - Model files and configuration

## Quick Start

### 1. Install Dependencies

**Option A: Automatic Ubuntu Setup**
```bash
chmod +x setup_ubuntu.sh
./setup_ubuntu.sh
```

**Option B: Manual Installation**
```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3-pip python3-dev python3-venv

# Install Python packages
pip3 install -r requirements.txt
```

**Option C: Using Virtual Environment (Recommended)**
```bash
./setup_ubuntu.sh --venv
source vlm_test_env/bin/activate
```

### 2. Prepare Test Data
Place a test image in the `test_data/` directory (any common format: jpg, png, etc.)
The script will automatically find it, or you can specify the path.

The `test_data/prompt.txt` file already contains a default prompt, but you can modify it.

### 3. Run the Test
```bash
# Automatic mode (uses files from test_data/)
python test_vlm.py

# Manual mode (specify your own files)
python test_vlm.py --image_path /path/to/your/image.jpg --prompt_file /path/to/your/prompt.txt
```

## Features
- **CPU-optimized**: Specifically tuned for CPU-only execution on Ubuntu
- **Automatic file detection**: Finds test files automatically in test_data/
- **Performance timing**: Measures model loading and inference time
- **Memory efficient**: Uses optimized settings for CPU memory usage
- **Flexible input**: Supports various image formats and custom prompts
- **Easy deployment**: Includes Ubuntu setup script for quick installation

## Output
The script will display:
- Model loading time
- Image processing details
- Inference timing
- Complete model response

## Example Usage
```bash
$ python3 test_vlm.py
Using device: cpu (CPU-only mode)
Model path: ./Isaac-0.1
Loading Isaac-0.1 model...
Loading tokenizer...
Loading config...
Initializing processor...
Loading model...
Model loaded successfully in 15.23 seconds
Looking for test files in ./test_data directory...
Loaded image: test_data/test_image.jpg (Size: (1024, 768))
Loaded prompt from: test_data/prompt.txt
Prompt: Describe what you see in this image...
Processing image and prompt...
Generation completed in 2.34 seconds
Total processing time: 2.45 seconds

==============================================================
RESULTS
==============================================================
Image: test_data/test_image.jpg
Prompt: Describe what you see in this image...

Model Response:
----------------------------------------
[Model's detailed description of the image]
==============================================================
```

## Troubleshooting
- Ensure all model files are present in the `Isaac-0.1/` directory
- Check that `modular_isaac.py` is available and importable
- Verify that your image file is in a supported format
- For Ubuntu: Run the setup script if you encounter dependency issues
- If memory errors occur, try with a smaller image or reduce max_new_tokens
- Ensure Python 3.8+ is installed (`python3 --version`)

## Custom Prompts
You can create different prompt files to test various capabilities:
- Object detection: "What objects can you identify in this image?"
- Spatial reasoning: "Describe the spatial relationships between objects"
- OCR tasks: "Read any text visible in this image"
- Detailed analysis: "Provide a comprehensive analysis of this image"