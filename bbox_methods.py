# Bounding Box Detection and Visualization Helper Methods
# Add these methods to the VLMTester class

def parse_bounding_boxes(self, response: str, image_size: tuple) -> list:
    """Parse bounding boxes from the VLM response.
    
    Args:
        response: The model's text response
        image_size: (width, height) of the original image
        
    Returns:
        List of bounding box dictionaries with 'bbox', 'center', and 'label'
    """
    import re
    
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
    try:
        from PIL import ImageDraw, ImageFont
    except ImportError:
        print("PIL ImageDraw/ImageFont not available for visualization")
        return image_path
    
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
    from pathlib import Path
    original_path = Path(image_path)
    result_path = original_path.parent / f"{original_path.stem}_result{original_path.suffix}"
    image.save(result_path)
    
    return str(result_path)