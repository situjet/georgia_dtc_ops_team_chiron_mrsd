#!/usr/bin/env python3
"""
Isaac-0.1 VLM Test Script with Bounding Box Detection

A test script for the Isaac-0.1 Vision-Language Model with GPU support
and bounding box detection/visualization capabilities.

Requirements:
- Python 3.8+
- PyTorch (with CUDA support if available)
- Transformers
- Pillow (PIL)
- NumPy

Usage:
    python test_vlm_gpu.py --image_path /path/to/image.jpg --prompt_file /path/to/prompt.txt
    
    Or place files in the test_data directory:
    - test_data/test_image.jpg (or .png)
    - test_data/prompt.txt
    
    Then run: python test_vlm_gpu.py
"""

import os
import sys
import argparse
import time
import re
from pathlib import Path
from typing import Optional

import torch
from PIL import Image, ImageDraw, ImageFont
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
    
    def parse_bounding_boxes(self, response: str, image_size: tuple) -> list:
        """Parse bounding boxes from the VLM response.
        
        Args:
            response: The model's text response
            image_size: (width, height) of the original image
            
        Returns:
            List of bounding box dictionaries with 'bbox', 'center', and 'label'
        """
        # Look for various coordinate patterns in the response
        boxes = []
        
        # Pattern for <click>x, y</click> format
        click_pattern = r'<click>(\d+),\s*(\d+)</click>'
        clicks = re.findall(click_pattern, response)
        
        # Pattern for <point>x, y</point> format  
        point_pattern = r'<point>(\d+),\s*(\d+)</point>'
        points = re.findall(point_pattern, response)
        
        # Pattern for coordinate pairs like (x, y)
        coord_pattern = r'\((\d+),\s*(\d+)\)'
        coords = re.findall(coord_pattern, response)
        
        # Pattern for bounding box format [x1, y1, x2, y2]
        bbox_pattern = r'\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]'
        bboxes = re.findall(bbox_pattern, response)
        
        image_width, image_height = image_size
        
        # Process click/point coordinates as center points
        for i, (x, y) in enumerate(clicks + points + coords):
            x, y = int(x), int(y)
            # Create a small bounding box around the point
            box_size = min(image_width, image_height) // 20  # 5% of smaller dimension
            x1 = max(0, x - box_size // 2)
            y1 = max(0, y - box_size // 2)
            x2 = min(image_width, x + box_size // 2)
            y2 = min(image_height, y + box_size // 2)
            
            boxes.append({
                'bbox': (x1, y1, x2, y2),
                'center': (x, y),
                'label': f'Point {i+1}',
                'type': 'point'
            })
        
        # Process explicit bounding boxes
        for i, (x1, y1, x2, y2) in enumerate(bboxes):
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            boxes.append({
                'bbox': (x1, y1, x2, y2),
                'center': (center_x, center_y),
                'label': f'Box {i+1}',
                'type': 'bbox'
            })
        
        # If no explicit coordinates found, try to extract from text descriptions
        if not boxes:
            # Look for text like "at coordinates", "located at", etc.
            text_coord_pattern = r'(?:at|coordinates?|located at|position)\s*[\(\[]?\s*(\d+)[,\s]+(\d+)\s*[\)\]]?'
            text_coords = re.findall(text_coord_pattern, response, re.IGNORECASE)
            
            for i, (x, y) in enumerate(text_coords):
                x, y = int(x), int(y)
                box_size = min(image_width, image_height) // 15
                x1 = max(0, x - box_size // 2)
                y1 = max(0, y - box_size // 2)
                x2 = min(image_width, x + box_size // 2)
                y2 = min(image_height, y + box_size // 2)
                
                boxes.append({
                    'bbox': (x1, y1, x2, y2),
                    'center': (x, y),
                    'label': f'Location {i+1}',
                    'type': 'text_coord'
                })
        
        return boxes
    
    def create_result_image(self, image_path: str, boxes: list, response: str) -> str:
        """Create a result image with bounding boxes and centers marked.
        
        Args:
            image_path: Path to the original image
            boxes: List of bounding box dictionaries
            response: The model's text response
            
        Returns:
            Path to the saved result image
        """
        # Load the original image
        image = Image.open(image_path).convert("RGB")
        draw = ImageDraw.Draw(image)
        
        # Try to load a font, fallback to default if not available
        try:
            font = ImageFont.truetype("arial.ttf", 20)
            small_font = ImageFont.truetype("arial.ttf", 16)
        except:
            try:
                font = ImageFont.load_default()
                small_font = ImageFont.load_default()
            except:
                font = None
                small_font = None
        
        # Define colors for different types
        colors = {
            'point': '#FF0000',      # Red for points
            'bbox': '#00FF00',       # Green for bounding boxes
            'text_coord': '#0000FF'  # Blue for text coordinates
        }
        
        # Draw bounding boxes and centers
        for i, box in enumerate(boxes):
            bbox = box['bbox']
            center = box['center']
            label = box['label']
            box_type = box.get('type', 'point')
            color = colors.get(box_type, '#FF0000')
            
            # Draw bounding box rectangle
            draw.rectangle(bbox, outline=color, width=3)
            
            # Draw center point
            center_size = 5
            center_bbox = [
                center[0] - center_size, center[1] - center_size,
                center[0] + center_size, center[1] + center_size
            ]
            draw.ellipse(center_bbox, fill=color)
            
            # Draw crosshair at center
            cross_size = 10
            draw.line([
                center[0] - cross_size, center[1],
                center[0] + cross_size, center[1]
            ], fill=color, width=2)
            draw.line([
                center[0], center[1] - cross_size,
                center[0], center[1] + cross_size
            ], fill=color, width=2)
            
            # Draw label
            if font:
                # Get text bbox for background
                try:
                    text_bbox = draw.textbbox((0, 0), label, font=font)
                    text_width = text_bbox[2] - text_bbox[0]
                    text_height = text_bbox[3] - text_bbox[1]
                except:
                    # Fallback for older PIL versions
                    try:
                        text_width, text_height = draw.textsize(label, font=font)
                    except:
                        text_width, text_height = 100, 20  # Fallback dimensions
                
                # Position label above the box
                label_x = bbox[0]
                label_y = max(0, bbox[1] - text_height - 5)
                
                # Draw background rectangle for text
                bg_bbox = [
                    label_x - 2, label_y - 2,
                    label_x + text_width + 2, label_y + text_height + 2
                ]
                draw.rectangle(bg_bbox, fill='white', outline=color)
                
                # Draw text
                draw.text((label_x, label_y), label, fill=color, font=font)
            
            # Add coordinate text
            coord_text = f"({center[0]}, {center[1]})"
            if small_font:
                coord_y = bbox[3] + 5
                draw.text((bbox[0], coord_y), coord_text, fill=color, font=small_font)
        
        # Save result image
        original_path = Path(image_path)
        result_path = original_path.parent / f"{original_path.stem}_result{original_path.suffix}"
        image.save(result_path)
        
        return str(result_path)
    
    def run_test(self, image_path: str, prompt_file: str):
        """Run a complete VLM test with timing information and bounding box detection.
        
        Args:
            image_path: Path to the test image
            prompt_file: Path to the prompt text file
        """
        print("="*60)
        print("Isaac-0.1 VLM Test with Bounding Box Detection")
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
        
        # Parse bounding boxes from the response
        boxes = self.parse_bounding_boxes(response, image.size)
        
        print("\n" + "="*60)
        print("RESULTS")
        print("="*60)
        print(f"Image: {image_path}")
        print(f"Prompt: {prompt}")
        print(f"\nModel Response:")
        print("-"*40)
        print(response)
        
        if boxes:
            print("\n" + "-"*40)
            print("DETECTED BOUNDING BOXES:")
            print("-"*40)
            for i, box in enumerate(boxes):
                bbox = box['bbox']
                center = box['center']
                label = box['label']
                box_type = box['type']
                print(f"{label} ({box_type}):")
                print(f"  Bounding Box: ({bbox[0]}, {bbox[1]}) -> ({bbox[2]}, {bbox[3]})")
                print(f"  Center: ({center[0]}, {center[1]})")
                print(f"  Size: {bbox[2]-bbox[0]} x {bbox[3]-bbox[1]} pixels")
            
            # Create result image with bounding boxes
            try:
                result_image_path = self.create_result_image(image_path, boxes, response)
                print(f"\nResult image saved: {result_image_path}")
            except Exception as e:
                print(f"\nError creating result image: {e}")
        else:
            print("\nNo bounding boxes or coordinates detected in the response.")
        
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
        description="Test Isaac-0.1 VLM with an image and text prompt, with bounding box detection"
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