import freetype
import numpy as np

def generate_text_data(
    input_string: str,
    font_file: str = './fonts/6x8.otb',
    font_width: int = 6,
    font_height: int = 8
) -> bytearray:    
    def char_to_unicode(c: str) -> str:
        """Handles CP437 encoding for vintage fonts"""
        try:
            byte = c.encode('cp437')[0]
            return bytes([byte]).decode('cp437') if byte > 127 else c
        except UnicodeEncodeError:
            return '?'  # Fallback character

    # Calculate total width considering optimized spacing for font size
    space_width = max(1, font_width // 3)  # Adaptive space width based on font size
    total_width = sum(space_width if c == ' ' else font_width for c in input_string)
    image = np.zeros((font_height, total_width), dtype=np.uint8)
    
    # Initialize FreeType rendering
    face = freetype.Face(font_file)
    face.set_pixel_sizes(0, font_height)
    
    x = 0  # Current horizontal position
    for char in input_string:
        if char == ' ':
            x += space_width
            continue        
        
        unicode_char = char_to_unicode(char)
        face.load_char(unicode_char, freetype.FT_LOAD_RENDER | freetype.FT_LOAD_TARGET_MONO)
        bitmap = face.glyph.bitmap
        
        if bitmap.width == font_width and bitmap.rows == font_height:
            target = np.zeros((font_height, font_width), dtype=np.uint8)
            
            # Handle bitmap buffer correctly for different font widths
            bytes_per_row = (bitmap.width + 7) // 8  # Calculate bytes needed per row
            
            for row in range(font_height):
                row_offset = row * bytes_per_row
                for bit in range(font_width):
                    byte_index = row_offset + (bit // 8)
                    bit_index = 7 - (bit % 8)
                    
                    if byte_index < len(bitmap.buffer):
                        byte = bitmap.buffer[byte_index]
                        target[row, bit] = 255 if byte & (1 << bit_index) else 0
                        
            image[:, x:x+font_width] = target
        
        x += font_width  # Advance position by character width
  


  
    raw_data = bytearray()
    for col in range(image.shape[1]):
        value = 0
        for bit in range(font_height):
            value |= (1 if image[bit, col] else 0) << bit
        raw_data.append(value)
    
    return raw_data