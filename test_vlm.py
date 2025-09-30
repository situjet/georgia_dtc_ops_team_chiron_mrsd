#!/usr/bin/env python3
"""
Isaac-0.1 VLM Test Script

A distribution-agnostic script to test the Isaac-0.1 Vision-Language Model.
This script loads an image and processes it with a text prompt to evaluate
the model's efficacy and response time.

Requirements:
- Python 3.8+
- PyTorch
- Transformers
- Pillow (PIL)
- NumPy

Usage:
    python test_vlm.py --image_path /path/to/image.jpg --prompt_file /path/to/prompt.txt
    
    Or place files in the test_data directory:
    - test_data/test_image.jpg (or .png)
    - test_data/prompt.txt
    
    Then run: python test_vlm.py
"""

import os
import sys
import argparse
import time
from pathlib import Path
from typing import Optional

import torch
from PIL import Image
import numpy as np

# Add the Isaac-0.1 directory to Python path to import the custom modules
script_dir = Path(__file__).parent.absolute()
isaac_dir = script_dir / "Isaac-0.1"
if isaac_dir.exists():
    sys.path.insert(0, str(isaac_dir))

try:
    from transformers import AutoTokenizer, AutoConfig, AutoModelForCausalLM
    from modular_isaac import IsaacProcessor
except ImportError as e:
    print(f"Error importing required modules: {e}")
    print("Please ensure you have transformers and the Isaac-0.1 model files available.")
    sys.exit(1)


class VLMTester:
    def __init__(self, model_path: str = None):
        """Initialize the VLM tester with the Isaac-0.1 model.
        
        Args:
            model_path: Path to the Isaac-0.1 model directory. If None, uses ./Isaac-0.1
        """
        self.model_path = model_path or str(isaac_dir)
        # Use GPU if available, fallback to CPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        print(f"Using device: {self.device}")
        if self.device.type == "cuda":
            print(f"GPU: {torch.cuda.get_device_name(0)}")
        print(f"Model path: {self.model_path}")
        
        # Initialize model components
        self.tokenizer = None
        self.processor = None
        self.model = None
        
    def load_model(self):
        """Load the Isaac-0.1 model, tokenizer, and processor."""
        print("Loading Isaac-0.1 model...")
        start_time = time.time()
        
        try:
            # Load tokenizer
            print("Loading tokenizer...")
            self.tokenizer = AutoTokenizer.from_pretrained(
                self.model_path, 
                trust_remote_code=True, 
                use_fast=False
            )
            
            # Load config
            print("Loading config...")
            config = AutoConfig.from_pretrained(
                self.model_path, 
                trust_remote_code=True
            )
            
            # Initialize processor
            print("Initializing processor...")
            self.processor = IsaacProcessor(tokenizer=self.tokenizer, config=config)
            
            # Load model
            print("Loading model...")
            self.model = AutoModelForCausalLM.from_pretrained(
                self.model_path, 
                trust_remote_code=True,
                torch_dtype=torch.float16 if self.device.type == "cuda" else torch.float32,
                low_cpu_mem_usage=True if self.device.type == "cpu" else False
            ).to(self.device)
            
            load_time = time.time() - start_time
            print(f"Model loaded successfully in {load_time:.2f} seconds")
            
        except Exception as e:
            print(f"Error loading model: {e}")
            raise
    
    def load_image(self, image_path: str) -> Image.Image:
        """Load and validate an image file.
        
        Args:
            image_path: Path to the image file
            
        Returns:
            PIL Image object
        """
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")
        
        try:
            image = Image.open(image_path).convert("RGB")
            print(f"Loaded image: {image_path} (Size: {image.size})")
            return image
        except Exception as e:
            raise ValueError(f"Error loading image {image_path}: {e}")
    
    def load_prompt(self, prompt_file: str) -> str:
        """Load prompt text from a file.
        
        Args:
            prompt_file: Path to the text file containing the prompt
            
        Returns:
            Prompt text as string
        """
        if not os.path.exists(prompt_file):
            raise FileNotFoundError(f"Prompt file not found: {prompt_file}")
        
        try:
            with open(prompt_file, 'r', encoding='utf-8') as f:
                prompt = f.read().strip()
            print(f"Loaded prompt from: {prompt_file}")
            print(f"Prompt: {prompt[:100]}{'...' if len(prompt) > 100 else ''}")
            return prompt
        except Exception as e:
            raise ValueError(f"Error loading prompt file {prompt_file}: {e}")
    
    def process_image_and_prompt(self, image: Image.Image, prompt: str) -> str:
        """Process an image with a text prompt using the VLM.
        
        Args:
            image: PIL Image object
            prompt: Text prompt string
            
        Returns:
            Generated response from the model
        """
        print("Processing image and prompt...")
        start_time = time.time()
        
        try:
            # Prepare simple conversation with image token
            conversation = [
                {
                    "role": "user",
                    "content": f"{self.processor.vision_token} {prompt}"
                }
            ]
            
            # Apply chat template to get formatted text
            formatted_text = self.processor.apply_chat_template(
                conversation, 
                add_generation_prompt=True, 
                tokenize=False
            )
            
            print(f"Formatted text: {formatted_text[:200]}...")  # Debug output
            
            # Process the formatted text and images using the processor
            inputs = self.processor(
                text=formatted_text,
                images=[image],
                return_tensors="pt"
            )
            
            # Move inputs to device
            inputs = {k: v.to(self.device) if hasattr(v, 'to') else v for k, v in inputs.items()}
            
            # Generate response
            with torch.no_grad():
                generation_start = time.time()
                outputs = self.model.generate(
                    **inputs,
                    max_new_tokens=512 if self.device.type == "cuda" else 256,
                    do_sample=True,
                    temperature=0.7,
                    top_p=0.9,
                    pad_token_id=self.tokenizer.eos_token_id,
                    use_cache=True
                )
                generation_time = time.time() - generation_start
            
            # Decode response
            input_length = inputs['input_ids'].shape[1] if 'input_ids' in inputs else 0
            response = self.tokenizer.decode(
                outputs[0][input_length:], 
                skip_special_tokens=True
            )
            
            total_time = time.time() - start_time
            
            print(f"Generation completed in {generation_time:.2f} seconds")
            print(f"Total processing time: {total_time:.2f} seconds")
            
            return response.strip()
            
        except Exception as e:
            print(f"Error during processing: {e}")
            raise
    
    def run_test(self, image_path: str, prompt_file: str):
        """Run a complete VLM test with timing information.
        
        Args:
            image_path: Path to the test image
            prompt_file: Path to the prompt text file
        """
        print("="*60)
        print("Isaac-0.1 VLM Test")
        print("="*60)
        
        # Load model if not already loaded
        if self.model is None:
            self.load_model()
        
        # Load image and prompt
        image = self.load_image(image_path)
        prompt = self.load_prompt(prompt_file)
        
        print("\n" + "-"*40)
        print("Running inference...")
        print("-"*40)
        
        # Process and get response
        response = self.process_image_and_prompt(image, prompt)
        
        print("\n" + "="*60)
        print("RESULTS")
        print("="*60)
        print(f"Image: {image_path}")
        print(f"Prompt: {prompt}")
        print(f"\nModel Response:")
        print("-"*40)
        print(response)
        print("="*60)


def find_test_files(test_dir: str = "test_data"):
    """Find test image and prompt files in the specified directory.
    
    Args:
        test_dir: Directory to search for test files
        
    Returns:
        Tuple of (image_path, prompt_file_path) or (None, None) if not found
    """
    test_path = Path(test_dir)
    if not test_path.exists():
        return None, None
    
    # Look for image files
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp']
    image_path = None
    
    for ext in image_extensions:
        for img_file in test_path.glob(f"*{ext}"):
            image_path = str(img_file)
            break
        if image_path:
            break
    
    # Look for prompt file
    prompt_file = None
    for prompt_name in ['prompt.txt', 'test_prompt.txt', 'query.txt']:
        prompt_path = test_path / prompt_name
        if prompt_path.exists():
            prompt_file = str(prompt_path)
            break
    
    return image_path, prompt_file


def main():
    parser = argparse.ArgumentParser(
        description="Test Isaac-0.1 VLM with an image and text prompt"
    )
    parser.add_argument(
        '--image_path', 
        type=str, 
        help='Path to the test image file'
    )
    parser.add_argument(
        '--prompt_file', 
        type=str, 
        help='Path to the text file containing the prompt'
    )
    parser.add_argument(
        '--model_path', 
        type=str, 
        default=None,
        help='Path to the Isaac-0.1 model directory (default: ./Isaac-0.1)'
    )
    
    args = parser.parse_args()
    
    # Try to find test files if not specified
    if not args.image_path or not args.prompt_file:
        print("Looking for test files in ./test_data directory...")
        found_image, found_prompt = find_test_files()
        
        image_path = args.image_path or found_image
        prompt_file = args.prompt_file or found_prompt
        
        if not image_path or not prompt_file:
            print("Error: Could not find test files.")
            print("Please either:")
            print("1. Specify --image_path and --prompt_file arguments")
            print("2. Create a test_data directory with:")
            print("   - An image file (jpg, png, etc.)")
            print("   - A prompt.txt file with your text prompt")
            sys.exit(1)
    else:
        image_path = args.image_path
        prompt_file = args.prompt_file
    
    try:
        # Initialize and run the tester
        tester = VLMTester(model_path=args.model_path)
        tester.run_test(image_path, prompt_file)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Test failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()