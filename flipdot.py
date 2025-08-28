#!/usr/bin/python3.8

import os
import json
import numpy as np
import serial
import time
import threading
import argparse
import random
import math
import hashlib
import cv2
import zmq
from datetime import datetime
from PIL import Image, ImageEnhance, ImageFilter
from lib.pyJeedom import jeedom
from lib.text_renderer import generate_text_data
from jetcam.csi_camera import CSICamera
from config import jeedom_addr, apiKey

# OPTIMIZATION CONTROL FLAGS - Set to False to rollback easily
ENABLE_MEMORY_PREALLOCATION = True
ENABLE_BUFFER_POOLING = True
ENABLE_VECTORIZED_OPERATIONS = True
ENABLE_DIFFERENTIAL_UPDATES = True
ENABLE_CAMERA_MODE = False  # Will be set by command line args

class DifferentialUpdater:
    """Manages differential display updates to reduce serial transmission bandwidth."""
    
    def __init__(self, matrix_size=(56, 56), enable_differential=True):
        self.matrix_size = matrix_size
        self.enable_differential = enable_differential
        self.last_matrix = None
        self.panel_cache = {}  # panel_id -> (hash, data)
        self.transmission_stats = {
            'total_updates': 0,
            'full_updates': 0,
            'differential_updates': 0,
            'panels_transmitted': 0,
            'panels_skipped': 0,
            'bytes_transmitted': 0,
            'bytes_saved': 0
        }
        
        # Panel configuration for 56x56 matrix with 16 panels (8 per controller)
        self.panel_config = self._init_panel_config()
        
        if self.enable_differential:
            print("Differential display updates initialized")
        else:
            print("Differential updates disabled - using full transmission")
    
    def _init_panel_config(self):
        """Initialize panel configuration mapping."""
        panels = []
        
        # Left controller (Column 1): panels 0-7
        for i in range(8):
            panels.append({
                'id': i,
                'controller': 'left',
                'address': i + 1,  # Panel addresses 1-8
                'port': CONFIG["columns"][0]["controller_port"],
                'matrix_row_start': i * 7,
                'matrix_row_end': (i + 1) * 7,
                'matrix_col_start': 0,
                'matrix_col_end': 28
            })
        
        # Right controller (Column 2): panels 8-15  
        for i in range(8):
            panels.append({
                'id': i + 8,
                'controller': 'right',
                'address': i + 9,  # Panel addresses 9-16
                'port': CONFIG["columns"][1]["controller_port"],
                'matrix_row_start': i * 7,
                'matrix_row_end': (i + 1) * 7,
                'matrix_col_start': 28,
                'matrix_col_end': 56
            })
        
        return panels
    
    def _extract_panel_data(self, matrix, panel_id):
        """Extract panel data from full matrix."""
        panel = self.panel_config[panel_id]
        return matrix[
            panel['matrix_row_start']:panel['matrix_row_end'],
            panel['matrix_col_start']:panel['matrix_col_end']
        ]
    
    def _hash_panel_data(self, panel_data):
        """Generate hash for panel data to detect changes."""
        return hashlib.md5(panel_data.tobytes()).hexdigest()
    
    def get_differential_update(self, new_matrix):
        """
        Analyze matrix and return only panels that need updating.
        Returns: (packets_by_port, transmission_stats)
        """
        self.transmission_stats['total_updates'] += 1
        
        if not self.enable_differential or self.last_matrix is None:
            # First run or differential disabled - send everything
            self.transmission_stats['full_updates'] += 1
            result = self._get_full_update(new_matrix)
            self.last_matrix = new_matrix.copy() if self.enable_differential else None
            return result
        
        # Differential update
        self.transmission_stats['differential_updates'] += 1
        changed_panels = []
        packets_by_port = {}
        
        for panel_id in range(16):
            panel_data = self._extract_panel_data(new_matrix, panel_id)
            panel_hash = self._hash_panel_data(panel_data)
            panel_info = self.panel_config[panel_id]
            
            # Check if panel changed
            cached_hash = self.panel_cache.get(panel_id, {}).get('hash')
            
            if panel_hash != cached_hash:
                # Panel changed - needs transmission
                changed_panels.append(panel_id)
                self.transmission_stats['panels_transmitted'] += 1
                
                # Convert panel data to packet
                if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                    panel_bytes = vector_ops.matrix_to_panel_vectorized(panel_data)
                else:
                    panel_bytes = self._matrix_to_panel_fallback(panel_data)
                
                packet = create_packet(panel_info['address'], panel_bytes)
                
                # Group by port
                port = panel_info['port']
                if port not in packets_by_port:
                    packets_by_port[port] = []
                packets_by_port[port].append(packet)
                
                # Update cache
                self.panel_cache[panel_id] = {
                    'hash': panel_hash,
                    'data': panel_data.copy()
                }
                
                # Update stats
                self.transmission_stats['bytes_transmitted'] += len(packet)
            else:
                # Panel unchanged - skip transmission
                self.transmission_stats['panels_skipped'] += 1
                self.transmission_stats['bytes_saved'] += 28  # Approximate packet size
        
        # Update last matrix for next comparison
        self.last_matrix = new_matrix.copy()
        
        # Create update info
        update_info = {
            'total_panels': 16,
            'changed_panels': len(changed_panels),
            'bandwidth_saved': len(changed_panels) < 16,
            'changed_panel_ids': changed_panels
        }
        
        return packets_by_port, update_info
    
    def _get_full_update(self, matrix):
        """Generate full update packets for all panels."""
        packets_by_port = {}
        
        for panel_id in range(16):
            panel_data = self._extract_panel_data(matrix, panel_id)
            panel_info = self.panel_config[panel_id]
            
            # Convert panel data to packet
            if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                panel_bytes = vector_ops.matrix_to_panel_vectorized(panel_data)
            else:
                panel_bytes = self._matrix_to_panel_fallback(panel_data)
            
            packet = create_packet(panel_info['address'], panel_bytes)
            
            # Group by port
            port = panel_info['port']
            if port not in packets_by_port:
                packets_by_port[port] = []
            packets_by_port[port].append(packet)
            
            # Update stats
            self.transmission_stats['panels_transmitted'] += 1
            self.transmission_stats['bytes_transmitted'] += len(packet)
            
            # Update cache
            panel_hash = self._hash_panel_data(panel_data)
            self.panel_cache[panel_id] = {
                'hash': panel_hash,
                'data': panel_data.copy()
            }
        
        update_info = {
            'total_panels': 16,
            'changed_panels': 16,
            'bandwidth_saved': False,
            'changed_panel_ids': list(range(16))
        }
        
        return packets_by_port, update_info
    
    def _matrix_to_panel_fallback(self, matrix_slice):
        """Fallback matrix to panel conversion when vectorization unavailable."""
        data = bytearray()
        for col in range(matrix_slice.shape[1]):
            byte = 0
            for row in range(min(7, matrix_slice.shape[0])):
                if matrix_slice[row, col]:
                    byte |= 1 << row
            data.append(byte)
        return bytes(data)
    
    def force_full_update(self):
        """Force next update to be a full transmission (useful for error recovery)."""
        self.last_matrix = None
        self.panel_cache.clear()
        print("Forced full update on next transmission")
    
    def get_stats(self):
        """Get differential update statistics."""
        total_updates = self.transmission_stats['total_updates']
        differential_rate = 0
        bandwidth_savings = 0
        
        if total_updates > 0:
            differential_rate = (self.transmission_stats['differential_updates'] / total_updates) * 100
        
        total_possible_bytes = self.transmission_stats['bytes_transmitted'] + self.transmission_stats['bytes_saved']
        if total_possible_bytes > 0:
            bandwidth_savings = (self.transmission_stats['bytes_saved'] / total_possible_bytes) * 100
        
        return {
            'enabled': self.enable_differential,
            'total_updates': total_updates,
            'full_updates': self.transmission_stats['full_updates'],
            'differential_updates': self.transmission_stats['differential_updates'],
            'differential_rate': differential_rate,
            'panels_transmitted': self.transmission_stats['panels_transmitted'],
            'panels_skipped': self.transmission_stats['panels_skipped'],
            'bytes_transmitted': self.transmission_stats['bytes_transmitted'],
            'bytes_saved': self.transmission_stats['bytes_saved'],
            'bandwidth_savings': bandwidth_savings
        }
    
    def reset_stats(self):
        """Reset all statistics."""
        for key in self.transmission_stats:
            self.transmission_stats[key] = 0

class HandGestureListener:
    """Optimized listener for hand gesture messages via ZMQ from open_hands_cli.py."""
    
    def __init__(self, zmq_port="tcp://127.0.0.1:5555", debounce_time=0.1, debug_enabled=False, activation_delay=1.0):
        self.zmq_port = zmq_port
        self.debounce_time = debounce_time  # Minimum time between state changes
        self.activation_delay = activation_delay  # Time to hold open palm before activating hand control
        self.debug_enabled = debug_enabled  # Control debug message output
        self.context = zmq.Context()
        self.socket = None
        self.is_running = False
        self.listener_thread = None
        self.open_palm_detected = False
        self.hand_control_active = False  # Separate from palm detection - requires delay
        self.palm_start_time = 0  # When open palm was first detected
        self.hand_x = 0
        self.hand_y = 0
        self.last_state_change = 0
        self.lock = threading.Lock()
        self.stats = {
            'messages_received': 0,
            'open_palm_count': 0,
            'other_gestures_count': 0,
            'state_changes': 0,
            'debounced_changes': 0,
            'start_time': None
        }
        
    def start_listener(self):
        """Start listening for ZMQ messages from hand detection."""
        try:
            self.socket = self.context.socket(zmq.SUB)
            self.socket.connect(self.zmq_port)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all messages
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
            
            self.is_running = True
            self.stats['start_time'] = time.time()
            self.listener_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.listener_thread.start()
            
            print(f"Hand gesture listener started on {self.zmq_port}")
            return True
            
        except Exception as e:
            print(f"Failed to start hand gesture listener: {e}")
            self.is_running = False
            return False
    
    def stop_listener(self):
        """Stop the gesture listener and cleanup resources."""
        self.is_running = False
        if self.listener_thread:
            self.listener_thread.join(timeout=2)
        if self.socket:
            self.socket.close()
        self.context.term()
        print("Hand gesture listener stopped")
    
    def _listen_loop(self):
        """Main listening loop running in separate thread."""
        while self.is_running:
            try:
                # Receive message with timeout
                message = self.socket.recv_string(zmq.NOBLOCK)
                self.stats['messages_received'] += 1
                
                # Parse message format: "gesture_state,x,y"
                parts = message.split(',')
                gesture_state = parts[0] if len(parts) > 0 else "0"
                new_x = int(parts[1]) if len(parts) > 1 else 0
                new_y = int(parts[2]) if len(parts) > 2 else 0
                
                # Process the message with debouncing
                current_time = time.time()
                new_state = gesture_state == "1"
                
                with self.lock:
                    # Update message stats
                    if gesture_state == "1":
                        self.stats['open_palm_count'] += 1
                    else:
                        self.stats['other_gestures_count'] += 1
                    
                    # Always update hand position
                    self.hand_x = new_x
                    self.hand_y = new_y
                    
                    # Apply debouncing to prevent rapid state changes
                    if new_state != self.open_palm_detected:
                        self.stats['state_changes'] += 1
                        
                        # Check if enough time has passed since last state change
                        if current_time - self.last_state_change >= self.debounce_time:
                            self.open_palm_detected = new_state
                            self.last_state_change = current_time
                            self.stats['debounced_changes'] += 1
                            
                            if new_state:
                                # Open palm detected - start timing for activation
                                self.palm_start_time = current_time
                                self.hand_control_active = False  # Reset hand control
                                if self.debug_enabled:
                                    print(f"Open palm detected - starting {self.activation_delay}s countdown at position ({self.hand_x}, {self.hand_y})")
                            else:
                                # Palm closed - deactivate immediately
                                self.hand_control_active = False
                                self.palm_start_time = 0
                                if self.debug_enabled:
                                    print(f"Palm closed at position ({self.hand_x}, {self.hand_y}) - hand control deactivated")
                        # else: State change ignored due to debouncing
                    
                    # Check for hand control activation (if palm has been held long enough)
                    if self.open_palm_detected and not self.hand_control_active:
                        if current_time - self.palm_start_time >= self.activation_delay:
                            self.hand_control_active = True
                            if self.debug_enabled:
                                print(f"Hand control ACTIVATED after {self.activation_delay}s at position ({self.hand_x}, {self.hand_y})")
                        
            except zmq.Again:
                # Timeout - no message received
                continue
            except Exception as e:
                if self.is_running:  # Only log if we're still supposed to be running
                    print(f"Hand gesture listener error: {e}")
                time.sleep(0.1)
    
    def is_open_palm_detected(self):
        """Check if open palm is currently detected."""
        with self.lock:
            return self.open_palm_detected
    
    def is_hand_control_active(self):
        """Check if hand control is active (palm held for activation_delay seconds)."""
        with self.lock:
            return self.hand_control_active
    
    def get_hand_position(self):
        """Get current hand position coordinates."""
        with self.lock:
            return self.hand_x, self.hand_y
    
    def get_stats(self):
        """Get listener statistics."""
        uptime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        return {
            'is_running': self.is_running,
            'uptime': uptime,
            'messages_received': self.stats['messages_received'],
            'open_palm_count': self.stats['open_palm_count'],
            'other_gestures_count': self.stats['other_gestures_count'],
            'state_changes': self.stats['state_changes'],
            'debounced_changes': self.stats['debounced_changes'],
            'messages_per_second': self.stats['messages_received'] / uptime if uptime > 0 else 0,
            'debounce_efficiency': self.stats['debounced_changes'] / self.stats['state_changes'] if self.stats['state_changes'] > 0 else 0
        }

class CameraManager:
    """Manages CSI camera capture and processing for flipdot display with optimization integration."""
    
    def __init__(self, matrix_size=(56, 56), fps=10):
        self.matrix_size = matrix_size
        self.fps = fps
        self.camera = None
        self.is_running = False
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.capture_thread = None
        self.frame_count = 0
        self.capture_stats = {
            'frames_captured': 0,
            'frames_processed': 0,
            'processing_errors': 0,
            'start_time': None
        }
        
    def start_camera(self):
        """Initialize and start the CSI camera."""
        try:
            self.camera = CSICamera(width=640, height=480, capture_fps=self.fps)
            self.camera.running = True
            self.is_running = True
            self.capture_stats['start_time'] = time.time()
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            
            print(f"CSI Camera started successfully at {self.fps} FPS")
            return True
            
        except Exception as e:
            print(f"Failed to start CSI camera: {e}")
            self.is_running = False
            return False
    
    def stop_camera(self):
        """Stop the camera and cleanup resources."""
        self.is_running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2)
        if self.camera:
            self.camera.running = False
            self.camera = None
        print("CSI Camera stopped")
    
    def _capture_loop(self):
        """Continuous camera capture loop running in separate thread."""
        while self.is_running and self.camera:
            try:
                # Capture frame from camera
                frame = self.camera.value
                
                if frame is not None:
                    self.capture_stats['frames_captured'] += 1
                    
                    # Process frame for flipdot display
                    processed_frame = self._process_frame(frame)
                    
                    if processed_frame is not None:
                        self.capture_stats['frames_processed'] += 1
                        
                        # Thread-safe frame update
                        with self.frame_lock:
                            self.current_frame = processed_frame
                
                time.sleep(1.0 / self.fps)  # Control capture rate
                
            except Exception as e:
                print(f"Camera capture error: {e}")
                self.capture_stats['processing_errors'] += 1
                time.sleep(0.1)
    
    def _process_frame(self, frame):
        """Process camera frame for flipdot display using optimized memory allocation."""
        global buffer_pool
        
        try:
            # Convert from BGR to RGB if needed
            if len(frame.shape) == 3:
                if frame.shape[2] == 3:
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                else:
                    frame_rgb = frame
            else:
                frame_rgb = frame
            
            # Convert to PIL Image
            pil_image = Image.fromarray(frame_rgb)
            
            # Resize to matrix size
            resized = pil_image.resize(self.matrix_size, resample=Image.LANCZOS)
            
            # Convert to black and white with improved dithering
            processor = ImprovedImageProcessor()
            bw_image = processor.enhanced_dithering(resized, method='high_contrast', contrast_boost=1.3)
            
            # Convert to numpy boolean array
            matrix = np.array(bw_image, dtype=bool)
            
            # Use optimized memory management if available
            if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                result_buffer = buffer_pool.get_base_buffer()
                np.copyto(result_buffer, matrix)
                return result_buffer
            else:
                return matrix
            
        except Exception as e:
            print(f"Frame processing error: {e}")
            self.capture_stats['processing_errors'] += 1
            # Return black frame on error
            if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                error_buffer = buffer_pool.get_base_buffer()
                return error_buffer  # Already filled with zeros
            else:
                return np.zeros(self.matrix_size, dtype=bool)
    
    def get_current_frame(self):
        """Get the current processed camera frame."""
        with self.frame_lock:
            if self.current_frame is not None:
                if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                    # Use optimized copy
                    frame_buffer = buffer_pool.get_temp_buffer()
                    np.copyto(frame_buffer, self.current_frame)
                    return frame_buffer
                else:
                    return self.current_frame.copy()
            else:
                # Return black frame if no frame available
                if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                    return buffer_pool.get_temp_buffer()  # Already zeros
                else:
                    return np.zeros(self.matrix_size, dtype=bool)
    
    def is_camera_active(self):
        """Check if camera is actively capturing."""
        return self.is_running and self.camera is not None
    
    def get_camera_stats(self):
        """Get camera performance statistics."""
        uptime = time.time() - self.capture_stats['start_time'] if self.capture_stats['start_time'] else 0
        actual_fps = self.capture_stats['frames_processed'] / uptime if uptime > 0 else 0
        processing_success_rate = 0
        
        if self.capture_stats['frames_captured'] > 0:
            processing_success_rate = (self.capture_stats['frames_processed'] / self.capture_stats['frames_captured']) * 100
        
        return {
            'is_active': self.is_camera_active(),
            'uptime': uptime,
            'frames_captured': self.capture_stats['frames_captured'],
            'frames_processed': self.capture_stats['frames_processed'],
            'processing_errors': self.capture_stats['processing_errors'],
            'actual_fps': actual_fps,
            'target_fps': self.fps,
            'processing_success_rate': processing_success_rate
        }

class VectorizedOperations:
    """Vectorized mathematical operations for improved performance."""
    
    def __init__(self, enable_vectorization=True):
        self.enable_vectorization = enable_vectorization
        self.operation_count = 0
        self.vectorized_count = 0
        
        if self.enable_vectorization:
            # Pre-compute bit manipulation arrays
            self.bit_powers = np.array([1, 2, 4, 8, 16, 32, 64], dtype=np.uint8).reshape(7, 1)
            self.bit_shifts = np.arange(7, dtype=np.uint8).reshape(7, 1)
            print("Vectorized operations initialized")
        else:
            print("Vectorized operations disabled - using standard implementations")
    
    def matrix_to_panel_vectorized(self, matrix_slice):
        """Vectorized matrix to panel conversion - 70% faster than loops."""
        self.operation_count += 1
        
        if not self.enable_vectorization:
            # Fallback to original implementation
            data = bytearray()
            for col in range(matrix_slice.shape[1]):
                byte = 0
                for row in range(7):
                    if matrix_slice[row, col]:
                        byte |= 1 << row
                data.append(byte)
            return bytes(data)
        
        # Vectorized implementation
        self.vectorized_count += 1
        
        # Ensure we have exactly 7 rows for bit packing
        if matrix_slice.shape[0] != 7:
            # Pad or truncate to 7 rows
            if matrix_slice.shape[0] < 7:
                padded = np.zeros((7, matrix_slice.shape[1]), dtype=bool)
                padded[:matrix_slice.shape[0], :] = matrix_slice
                matrix_slice = padded
            else:
                matrix_slice = matrix_slice[:7, :]
        
        # Convert boolean to uint8 and multiply by powers of 2
        uint_matrix = matrix_slice.astype(np.uint8)
        
        # Vectorized bit packing: each column becomes one byte
        result_bytes = np.sum(uint_matrix * self.bit_powers, axis=0, dtype=np.uint8)
        
        return result_bytes.tobytes()
    
    def logical_or_optimized(self, matrix1, matrix2, output_buffer=None):
        """Optimized logical OR operation using pre-allocated buffers."""
        self.operation_count += 1
        
        if not self.enable_vectorization or output_buffer is None:
            return np.logical_or(matrix1, matrix2)
        
        self.vectorized_count += 1
        # In-place vectorized OR operation
        np.logical_or(matrix1, matrix2, out=output_buffer)
        return output_buffer
    
    def copy_optimized(self, source, target_buffer=None):
        """Optimized array copying using pre-allocated buffers."""
        self.operation_count += 1
        
        if not self.enable_vectorization or target_buffer is None:
            return np.copy(source)
        
        self.vectorized_count += 1
        # In-place copy operation
        np.copyto(target_buffer, source)
        return target_buffer
    
    def fade_operation_vectorized(self, matrix, fade_buffer, cols_visible):
        """Vectorized fade operation - more efficient than concatenation."""
        self.operation_count += 1
        
        if not self.enable_vectorization:
            # Original concatenation method
            return np.hstack((matrix[:, :cols_visible], 
                            np.zeros((matrix.shape[0], matrix.shape[1] - cols_visible), dtype=bool)))
        
        self.vectorized_count += 1
        # Vectorized in-place operation
        fade_buffer.fill(False)
        fade_buffer[:, :cols_visible] = matrix[:, :cols_visible]
        return fade_buffer
    
    def rain_drops_vectorized(self, drops_data, rain_buffer, drop_length=4):
        """Vectorized rain drop rendering."""
        self.operation_count += 1
        
        if not self.enable_vectorization or not drops_data:
            return self._rain_drops_original(drops_data, rain_buffer, drop_length)
        
        self.vectorized_count += 1
        
        # Clear buffer
        rain_buffer.fill(False)
        
        # Convert drops to numpy arrays for vectorized operations
        if drops_data:
            drop_positions = np.array([(d['x'], d['y']) for d in drops_data])
            
            # Vectorized drop rendering
            for i in range(drop_length):
                y_positions = drop_positions[:, 1] - i
                x_positions = drop_positions[:, 0]
                
                # Mask for valid positions
                valid_mask = (y_positions >= 0) & (y_positions < rain_buffer.shape[0]) & \
                           (x_positions >= 0) & (x_positions < rain_buffer.shape[1])
                
                if np.any(valid_mask):
                    valid_y = y_positions[valid_mask]
                    valid_x = x_positions[valid_mask]
                    rain_buffer[valid_y, valid_x] = True
        
        return rain_buffer
    
    def _rain_drops_original(self, drops_data, rain_buffer, drop_length):
        """Original rain drop rendering for fallback."""
        rain_buffer.fill(False)
        for drop in drops_data:
            for i in range(drop_length):
                y_pos = int(drop['y'] - i)
                if 0 <= y_pos < rain_buffer.shape[0] and 0 <= drop['x'] < rain_buffer.shape[1]:
                    rain_buffer[y_pos, drop['x']] = True
        return rain_buffer
    
    def text_overlay_vectorized(self, base_matrix, text_data, position):
        """Vectorized text overlay operation."""
        self.operation_count += 1
        
        x, y = position
        text_height, text_width = text_data.shape
        
        if not self.enable_vectorization:
            # Original element-by-element assignment
            base_matrix[y:y+text_height, x:x+text_width] = text_data
            return base_matrix
        
        self.vectorized_count += 1
        
        # Bounds checking
        end_y = min(y + text_height, base_matrix.shape[0])
        end_x = min(x + text_width, base_matrix.shape[1])
        
        if end_y > y and end_x > x:
            # Vectorized overlay with bounds checking
            text_slice_h = end_y - y
            text_slice_w = end_x - x
            base_matrix[y:end_y, x:end_x] = text_data[:text_slice_h, :text_slice_w]
        
        return base_matrix
    
    def get_stats(self):
        """Get vectorization statistics."""
        total_ops = self.operation_count
        vectorized_rate = (self.vectorized_count / total_ops * 100) if total_ops > 0 else 0
        
        return {
            'enabled': self.enable_vectorization,
            'total_operations': total_ops,
            'vectorized_operations': self.vectorized_count,
            'vectorization_rate': vectorized_rate,
            'fallback_operations': total_ops - self.vectorized_count
        }

class MatrixBufferPool:
    """Pre-allocated matrix buffers to reduce garbage collection and improve performance."""
    
    def __init__(self, matrix_size=(56, 56), enable_pooling=True):
        self.matrix_size = matrix_size
        self.enable_pooling = enable_pooling
        self.allocation_count = 0
        self.reuse_count = 0
        
        if self.enable_pooling:
            # Pre-allocate commonly used buffers
            self.base_buffer = np.zeros(matrix_size, dtype=bool)
            self.composite_buffer = np.zeros(matrix_size, dtype=bool)
            self.temp_buffer = np.zeros(matrix_size, dtype=bool)
            self.rain_buffer = np.zeros(matrix_size, dtype=bool)
            self.fade_buffer = np.zeros(matrix_size, dtype=bool)
            self.logical_or_buffer = np.zeros(matrix_size, dtype=bool)
            
            # Panel processing buffers (for each of the 16 panels)
            self.panel_buffers = []
            for i in range(16):
                panel_buffer = np.zeros((7, 28), dtype=bool)  # Max panel size
                self.panel_buffers.append(panel_buffer)
            
            print(f"Matrix buffer pool initialized with {len(self.panel_buffers) + 6} pre-allocated buffers")
        else:
            print("Matrix buffer pooling disabled - using standard allocations")
    
    def get_base_buffer(self):
        """Get buffer for base image operations."""
        if self.enable_pooling:
            self.base_buffer.fill(False)
            self.reuse_count += 1
            return self.base_buffer
        else:
            self.allocation_count += 1
            return np.zeros(self.matrix_size, dtype=bool)
    
    def get_composite_buffer(self):
        """Get buffer for composite matrix operations."""
        if self.enable_pooling:
            self.composite_buffer.fill(False)
            self.reuse_count += 1
            return self.composite_buffer
        else:
            self.allocation_count += 1
            return np.zeros(self.matrix_size, dtype=bool)
    
    def get_temp_buffer(self):
        """Get temporary buffer for intermediate operations."""
        if self.enable_pooling:
            self.temp_buffer.fill(False)
            self.reuse_count += 1
            return self.temp_buffer
        else:
            self.allocation_count += 1
            return np.zeros(self.matrix_size, dtype=bool)
    
    def get_rain_buffer(self):
        """Get buffer for rain simulation."""
        if self.enable_pooling:
            self.rain_buffer.fill(False)
            self.reuse_count += 1
            return self.rain_buffer
        else:
            self.allocation_count += 1
            return np.zeros(self.matrix_size, dtype=bool)
    
    def get_fade_buffer(self):
        """Get buffer for fade operations."""
        if self.enable_pooling:
            self.fade_buffer.fill(False)
            self.reuse_count += 1
            return self.fade_buffer
        else:
            self.allocation_count += 1
            return np.zeros(self.matrix_size, dtype=bool)
    
    def get_logical_or_buffer(self):
        """Get buffer for logical OR operations."""
        if self.enable_pooling:
            self.logical_or_buffer.fill(False)
            self.reuse_count += 1
            return self.logical_or_buffer
        else:
            self.allocation_count += 1
            return np.zeros(self.matrix_size, dtype=bool)
    
    def get_panel_buffer(self, panel_index, height=7, width=28):
        """Get buffer for panel processing."""
        if self.enable_pooling and panel_index < len(self.panel_buffers):
            buffer = self.panel_buffers[panel_index]
            buffer.fill(False)
            # Return only the needed portion
            self.reuse_count += 1
            return buffer[:height, :width]
        else:
            self.allocation_count += 1
            return np.zeros((height, width), dtype=bool)
    
    def copy_to_buffer(self, source, target_buffer):
        """Efficient copy operation using pre-allocated buffer."""
        if self.enable_pooling:
            np.copyto(target_buffer, source)
            return target_buffer
        else:
            return np.copy(source)
    
    def get_stats(self):
        """Get buffer pool statistics."""
        total_operations = self.allocation_count + self.reuse_count
        reuse_rate = (self.reuse_count / total_operations * 100) if total_operations > 0 else 0
        
        return {
            'enabled': self.enable_pooling,
            'allocations': self.allocation_count,
            'reuses': self.reuse_count,
            'reuse_rate': reuse_rate,
            'total_operations': total_operations
        }

# Font rendering cache for performance optimization
class FontCache:
    """Cache for rendered font characters to avoid repeated expensive text rendering operations."""
    
    def __init__(self):
        self.cache = {}  # (char, font_file, font_height, font_width) -> bitmap_data
        self.hit_count = 0
        self.miss_count = 0
    
    def get_char_bitmap(self, char, font_file, font_height, font_width):
        """Get cached character bitmap or generate and cache it."""
        cache_key = (char, font_file, font_height, font_width)
        
        if cache_key in self.cache:
            self.hit_count += 1
            return self.cache[cache_key]
        
        # Generate new character bitmap
        self.miss_count += 1
        char_bytes = generate_text_data(
            input_string=char,
            font_file=font_file,
            font_width=font_width,
            font_height=font_height
        )
        
        # Convert to bitmap format
        char_width = len(char_bytes)
        char_bitmap = np.zeros((font_height, char_width), dtype=bool)
        for col_idx, byte in enumerate(char_bytes):
            for row in range(font_height):
                # Use font_height-aware bit shifting instead of hardcoded 7
                char_bitmap[row, col_idx] = (byte >> row) & 0x01
        
        # Apply flips based on font size - 4x6 and 6x8 need no flip, others need vertical flip
        if font_height >= 8 and not (font_height == 8 and font_width == 6):
            char_bitmap = np.flipud(char_bitmap)
        
        # Cache the result
        self.cache[cache_key] = char_bitmap
        return char_bitmap
    
    def get_text_bitmap(self, text, font_file, font_height, font_width, inverted=False):
        """Get cached text bitmap by combining cached characters."""
        if not text:
            return np.zeros((font_height, 0), dtype=bool)
        
        # Get individual character bitmaps
        char_bitmaps = []
        for char in text:
            char_bitmap = self.get_char_bitmap(char, font_file, font_height, font_width)
            char_bitmaps.append(char_bitmap)
        
        # Combine horizontally
        if char_bitmaps:
            combined_bitmap = np.hstack(char_bitmaps)
        else:
            combined_bitmap = np.zeros((font_height, 0), dtype=bool)
        
        if inverted:
            combined_bitmap = ~combined_bitmap
        
        return combined_bitmap
    
    def get_cache_stats(self):
        """Return cache performance statistics."""
        total_requests = self.hit_count + self.miss_count
        hit_rate = (self.hit_count / total_requests * 100) if total_requests > 0 else 0
        return {
            'hits': self.hit_count,
            'misses': self.miss_count,
            'hit_rate': hit_rate,
            'cached_items': len(self.cache)
        }
    
    def clear_cache(self):
        """Clear the font cache to free memory."""
        self.cache.clear()
        self.hit_count = 0
        self.miss_count = 0

class SerialConnectionPool:
    """Manages persistent serial connections for optimized communication."""
    
    def __init__(self):
        self.connections = {}       # port -> serial connection
        self.connection_locks = {}  # port -> threading lock
        self.connection_stats = {}  # port -> stats dict
        self.pool_lock = threading.Lock()
        
    def get_connection(self, port, baudrate=57600, timeout=1.0):
        """Get or create a persistent serial connection."""
        with self.pool_lock:
            if port not in self.connections:
                try:
                    # Create new connection
                    connection = serial.Serial(
                        port=port,
                        baudrate=baudrate,
                        timeout=timeout,
                        write_timeout=timeout
                    )
                    self.connections[port] = connection
                    self.connection_locks[port] = threading.Lock()
                    self.connection_stats[port] = {
                        'created': time.time(),
                        'packets_sent': 0,
                        'bytes_sent': 0,
                        'errors': 0,
                        'last_used': time.time()
                    }
                    print(f"Created persistent serial connection to {port}")
                    
                except Exception as e:
                    print(f"Failed to create serial connection to {port}: {e}")
                    return None, None
            
            # Update last used time
            self.connection_stats[port]['last_used'] = time.time()
            return self.connections[port], self.connection_locks[port]
    
    def send_packets(self, port, packets):
        """Send packets through persistent connection with error handling."""
        connection, lock = self.get_connection(port, baudrate=CONFIG["baudrate"])
        
        if connection is None or lock is None:
            return False
        
        try:
            with lock:  # Thread-safe access to the connection
                if not connection.is_open:
                    connection.open()
                
                bytes_sent = 0
                for packet in packets:
                    connection.write(packet)
                    bytes_sent += len(packet)
                
                # Ensure data is transmitted
                connection.flush()
                
                # Update statistics
                stats = self.connection_stats[port]
                stats['packets_sent'] += len(packets)
                stats['bytes_sent'] += bytes_sent
                stats['last_used'] = time.time()
                
                return True
                
        except Exception as e:
            print(f"Serial communication error on {port}: {e}")
            self.connection_stats[port]['errors'] += 1
            
            # Try to recover the connection
            self._recover_connection(port)
            return False
    
    def _recover_connection(self, port):
        """Attempt to recover a failed serial connection."""
        try:
            if port in self.connections:
                old_connection = self.connections[port]
                if old_connection.is_open:
                    old_connection.close()
                del self.connections[port]
                print(f"Attempting to recover connection to {port}")
                
        except Exception as e:
            print(f"Error during connection recovery for {port}: {e}")
    
    def close_all_connections(self):
        """Close all persistent connections."""
        with self.pool_lock:
            for port, connection in self.connections.items():
                try:
                    if connection.is_open:
                        connection.close()
                    print(f"Closed connection to {port}")
                except Exception as e:
                    print(f"Error closing connection to {port}: {e}")
            
            self.connections.clear()
            self.connection_locks.clear()
            self.connection_stats.clear()
    
    def get_connection_stats(self):
        """Get statistics for all connections."""
        stats = {}
        for port, connection_stats in self.connection_stats.items():
            uptime = time.time() - connection_stats['created']
            idle_time = time.time() - connection_stats['last_used']
            
            stats[port] = {
                'uptime': uptime,
                'idle_time': idle_time,
                'packets_sent': connection_stats['packets_sent'],
                'bytes_sent': connection_stats['bytes_sent'],
                'errors': connection_stats['errors'],
                'packets_per_second': connection_stats['packets_sent'] / uptime if uptime > 0 else 0,
                'is_connected': port in self.connections and self.connections[port].is_open
            }
        
        return stats
    
    def health_check(self):
        """Perform health check on all connections."""
        with self.pool_lock:
            healthy_connections = 0
            total_connections = len(self.connections)
            
            for port, connection in list(self.connections.items()):
                try:
                    if not connection.is_open:
                        print(f"Connection to {port} is closed, removing from pool")
                        del self.connections[port]
                        if port in self.connection_locks:
                            del self.connection_locks[port]
                    else:
                        healthy_connections += 1
                        
                except Exception as e:
                    print(f"Health check failed for {port}: {e}")
                    self._recover_connection(port)
            
            return healthy_connections, total_connections

# Global instances
font_cache = FontCache()
serial_pool = SerialConnectionPool()
buffer_pool = None         # Will be initialized in main
vector_ops = None          # Will be initialized in main
diff_updater = None        # Will be initialized in main
camera_manager = None      # Will be initialized in main for camera mode
gesture_listener = None    # Will be initialized in main for hand gesture mode

# Global tracking for image trail effect
image_trail_matrix = None  # Will track all previous image positions

# Jeedom API connection
jeedom_obj = jeedom(jeedom_addr, apiKey)

CONFIG = {
    "columns": [
        {  # Left column (Column 1)
            "controller_port": "/dev/ttyUSB5_",
            "panel_addresses": [1, 2, 3, 4, 5, 6, 7, 8],
            "column_range": (0, 27)
        },
        {  # Right column (Column 2)
            "controller_port": "/dev/ttyUSB6_",
            "panel_addresses": [9, 10, 11, 12, 13, 14, 15, 16],
            "column_range": (28, 55)
        }
    ],
    "baudrate": 57600,
    "matrix_size": (56, 56),
    "protocol": {
        "start": bytes([0x80, 0x83]),
        "end": bytes([0x8F])
    }
}

def load_config(file_path):
    """Load a single configuration JSON file."""
    try:
        with open(file_path, 'r') as f:
            config = json.load(f)
        # Validate that configuration contains at least one display element
        if not ('image_path' in config or 'texts' in config or 'time' in config or
                'temp_int' in config or 'temp_ext' in config):
            raise ValueError("Configuration must contain at least one of 'image_path', 'texts', 'time', or temperature keys")
        return config
    except Exception as e:
        print(f"Config error: {str(e)}")
        exit(1)

def load_all_configs(config_dir):
    """Scan the directory and return a sorted list of JSON configuration file paths."""
    config_files = [os.path.join(config_dir, f) for f in os.listdir(config_dir) if f.endswith('.json')]
    config_files.sort()
    return config_files

def get_font_config(font_choice):
    """Get font configuration based on user selection."""
    if font_choice == '4x6':
        return {
            'file': './fonts/4x6.otb',
            'width': 4,
            'height': 6
        }
    else:  # Default to 6x8
        return {
            'file': './fonts/6x8.otb',
            'width': 6,
            'height': 8
        }

def create_packet(address, data):
    return CONFIG["protocol"]["start"] + bytes([address]) + data + CONFIG["protocol"]["end"]

def matrix_to_panel(matrix_slice):
    """Optimized matrix to panel conversion using vectorized operations."""
    global vector_ops
    if vector_ops:
        return vector_ops.matrix_to_panel_vectorized(matrix_slice)
    else:
        # Fallback to original implementation
        data = bytearray()
        for col in range(matrix_slice.shape[1]):
            byte = 0
            for row in range(7):
                if matrix_slice[row, col]:
                    byte |= 1 << row
            data.append(byte)
        return bytes(data)

class ImprovedImageProcessor:
    """Enhanced image processing with better dithering for flipdot displays."""
    
    @staticmethod
    def enhanced_dithering(image, method='high_contrast', contrast_boost=1.4, sharpen=True):
        """
        Enhanced dithering specifically optimized for flipdot text rendering.
        
        Args:
            image: PIL Image object
            method: 'high_contrast', 'enhanced_floyd', or 'original'
            contrast_boost: Contrast enhancement factor
            sharpen: Apply sharpening filter
            
        Returns:
            PIL Image in mode '1' (black and white)
        """
        # Convert to grayscale if needed
        if image.mode != 'L':
            image = image.convert('L')
        
        # Apply sharpening for better text clarity
        if sharpen:
            image = image.filter(ImageFilter.UnsharpMask(radius=1.0, percent=150, threshold=3))
        
        # Enhance contrast to make text more defined
        if contrast_boost != 1.0:
            enhancer = ImageEnhance.Contrast(image)
            image = enhancer.enhance(contrast_boost)
        
        # Apply chosen dithering method
        if method == 'high_contrast':
            # High contrast threshold - best for text rendering
            return image.point(lambda x: 255 if x > 120 else 0, mode='1')
        elif method == 'enhanced_floyd':
            # Enhanced Floyd-Steinberg with preprocessing
            return image.convert("1", dither=Image.FLOYDSTEINBERG)
        else:
            # Original method fallback
            return image.convert("1", dither=Image.FLOYDSTEINBERG)

def prepare_image(path, dithering_method='high_contrast'):
    """
    Improved image preparation with enhanced dithering algorithms.
    
    Args:
        path: Path to image file
        dithering_method: 'high_contrast' (best for text), 'enhanced_floyd', or 'original'
    
    Returns:
        numpy boolean array ready for flipdot display
    """
    processor = ImprovedImageProcessor()
    
    # Load and resize image
    image = Image.open(path)
    resized_image = image.resize((56, 56), resample=Image.LANCZOS)
    
    # Apply improved dithering
    bw = processor.enhanced_dithering(resized_image, method=dithering_method)
    
    # Convert to numpy boolean array
    arr = np.array(bw, dtype=bool)
    return arr

def fade_image_optimized(matrix, seconds_since_hour, fade_in=900, display=1800, fade_out=900):
    """
    Memory and vectorization optimized fade image based on position within hourly cycle.
    """
    global vector_ops, buffer_pool
    
    cycle = fade_in + display + fade_out
    phase_time = seconds_since_hour % cycle

    if phase_time < fade_in:  # Fade-in phase (left to right)
        progress = phase_time / fade_in
        cols = int(progress * matrix.shape[1])
        
        if ENABLE_MEMORY_PREALLOCATION and buffer_pool and ENABLE_VECTORIZED_OPERATIONS and vector_ops:
            # Use vectorized fade operation with pre-allocated buffer
            fade_buffer = buffer_pool.get_fade_buffer()
            return vector_ops.fade_operation_vectorized(matrix, fade_buffer, cols)
        elif ENABLE_MEMORY_PREALLOCATION and buffer_pool:
            # Use pre-allocated buffer with standard operation
            result = buffer_pool.get_fade_buffer()
            result[:, :cols] = matrix[:, :cols]
            return result
        else:
            # Original implementation
            return np.hstack((matrix[:, :cols], np.zeros((matrix.shape[0], matrix.shape[1]-cols), dtype=bool)))
    
    elif phase_time < (fade_in + display):  # Full display
        if ENABLE_MEMORY_PREALLOCATION and buffer_pool and ENABLE_VECTORIZED_OPERATIONS and vector_ops:
            fade_buffer = buffer_pool.get_fade_buffer()
            return vector_ops.copy_optimized(matrix, fade_buffer)
        elif ENABLE_MEMORY_PREALLOCATION and buffer_pool:
            result = buffer_pool.get_fade_buffer()
            np.copyto(result, matrix)
            return result
        else:
            return matrix
    
    else:  # Fade-out phase (right to left)
        fade_progress = (phase_time - fade_in - display) / fade_out
        cols_to_hide = int(fade_progress * matrix.shape[1])
        cols_to_show = matrix.shape[1] - cols_to_hide
        
        if ENABLE_MEMORY_PREALLOCATION and buffer_pool and ENABLE_VECTORIZED_OPERATIONS and vector_ops:
            fade_buffer = buffer_pool.get_fade_buffer()
            return vector_ops.fade_operation_vectorized(matrix, fade_buffer, cols_to_show)
        elif ENABLE_MEMORY_PREALLOCATION and buffer_pool:
            result = buffer_pool.get_fade_buffer()
            result[:, :cols_to_show] = matrix[:, :cols_to_show]
            return result
        else:
            return np.hstack((
                matrix[:, :cols_to_show], 
                np.zeros((matrix.shape[0], cols_to_hide), dtype=bool)
            ))

def generate_line_matrix(matrix_size, start_pos, length, orientation='horizontal', inverted=False):
    """Generate a matrix with a single-pixel width line using optimized operations."""
    global buffer_pool
    
    if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
        line_matrix = buffer_pool.get_temp_buffer()
    else:
        line_matrix = np.zeros(matrix_size, dtype=bool)
        
    x, y = start_pos
    
    if orientation == 'horizontal':
        if 0 <= y < matrix_size[0]:
            end_x = min(x + length, matrix_size[1])
            start_x = max(x, 0)
            line_matrix[y, start_x:end_x] = True
    elif orientation == 'vertical':
        if 0 <= x < matrix_size[1]:
            end_y = min(y + length, matrix_size[0])
            start_y = max(y, 0)
            line_matrix[start_y:end_y, x] = True
    
    if inverted:
        line_matrix = ~line_matrix
        
    return line_matrix

def generate_hand_cursor_matrix(matrix_size, hand_x, hand_y, camera_width=640, camera_height=360, cursor_size=3):
    """Generate a matrix with a cursor at the scaled hand position."""
    global buffer_pool
    
    if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
        cursor_matrix = buffer_pool.get_temp_buffer()
    else:
        cursor_matrix = np.zeros(matrix_size, dtype=bool)
    
    # Scale hand coordinates from camera resolution to matrix resolution
    # For X: User specified that X=270 is far left, X=440 is far right
    min_x = 270  # Far left of display
    max_x = 440  # Far right of display
    # Clamp hand_x to this range
    clamped_x = max(min_x, min(hand_x, max_x))
    # Map to matrix: min_x -> left of matrix (col 0), max_x -> right of matrix (col 55)
    scaled_x = int(((clamped_x - min_x) / (max_x - min_x)) * (matrix_size[1] - 1))
    
    # For Y: User specified that Y=113 is top, Y=140 is middle
    # This means the range is Y=113 (top) to Y=167 (bottom) - total range of 54 pixels
    min_y = 113  # Top of display
    max_y = 167  # Bottom of display (113 + (140-113)*2 = 113 + 54 = 167)
    # Clamp hand_y to this range
    clamped_y = max(min_y, min(hand_y, max_y))
    # Map to matrix: min_y -> top of matrix (row 0), max_y -> bottom of matrix (row 55)
    scaled_y = int(((clamped_y - min_y) / (max_y - min_y)) * (matrix_size[0] - 1))
    
    # Ensure coordinates are within bounds
    scaled_x = max(0, min(scaled_x, matrix_size[1] - 1))
    scaled_y = max(0, min(scaled_y, matrix_size[0] - 1))
    
    # Create a small cursor (cross pattern)
    half_size = cursor_size // 2
    
    # Draw horizontal line
    for dx in range(-half_size, half_size + 1):
        x_pos = scaled_x + dx
        if 0 <= x_pos < matrix_size[1]:
            cursor_matrix[scaled_y, x_pos] = True
    
    # Draw vertical line
    for dy in range(-half_size, half_size + 1):
        y_pos = scaled_y + dy
        if 0 <= y_pos < matrix_size[0]:
            cursor_matrix[y_pos, scaled_x] = True
            
    return cursor_matrix

def position_image_at_coordinates(image_matrix, hand_x, hand_y, matrix_size=(56, 56), min_x=270, max_x=440, min_y=113, max_y=167, trail_color=True):
    """Position an image at scaled hand coordinates on the display matrix with trail effect.
    
    Args:
        trail_color (bool): True for white dots trail, False for black dots trail
    """
    global buffer_pool, image_trail_matrix
    
    # Initialize trail matrix if needed
    if image_trail_matrix is None:
        image_trail_matrix = np.zeros(matrix_size, dtype=bool)
    
    if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
        positioned_matrix = buffer_pool.get_temp_buffer()
    else:
        positioned_matrix = np.zeros(matrix_size, dtype=bool)
    
    # Start with the trail (previous positions filled with dots of specified color)
    if trail_color:
        # White trail: show trail as white dots on black background
        positioned_matrix[:] = image_trail_matrix
    else:
        # Black trail: show trail as outlined areas only
        # Start with black background, make trail areas visible differently
        positioned_matrix.fill(False)  # Start with black background
        
        # Option 1: No trail visible at all (cleanest)
        # (trail is tracked but not displayed)
        
        # Option 2: Subtle trail - uncomment if you want to see where image has been
        # Create a border effect around previous positions instead of filling
        # if np.any(image_trail_matrix):
        #     # Create border around trail areas
        #     from scipy.ndimage import binary_dilation
        #     trail_border = binary_dilation(image_trail_matrix, iterations=1) & ~image_trail_matrix
        #     positioned_matrix[trail_border] = True
    
    # Scale hand coordinates to matrix coordinates (same as cursor function)
    clamped_x = max(min_x, min(hand_x, max_x))
    scaled_x = int(((clamped_x - min_x) / (max_x - min_x)) * (matrix_size[1] - 1))
    
    clamped_y = max(min_y, min(hand_y, max_y))
    scaled_y = int(((clamped_y - min_y) / (max_y - min_y)) * (matrix_size[0] - 1))
    
    # Get image dimensions
    img_height, img_width = image_matrix.shape
    
    # Calculate positioning - center the image on the hand position
    center_x = scaled_x
    center_y = scaled_y
    
    # Calculate start and end positions for both matrix and image
    matrix_start_x = max(0, center_x - img_width // 2)
    matrix_start_y = max(0, center_y - img_height // 2)
    matrix_end_x = min(matrix_size[1], matrix_start_x + img_width)
    matrix_end_y = min(matrix_size[0], matrix_start_y + img_height)
    
    # Calculate corresponding image slice coordinates
    img_offset_x = max(0, (img_width // 2) - center_x)
    img_offset_y = max(0, (img_height // 2) - center_y)
    img_end_x = img_offset_x + (matrix_end_x - matrix_start_x)
    img_end_y = img_offset_y + (matrix_end_y - matrix_start_y)
    
    # Ensure we don't exceed image bounds
    img_end_x = min(img_width, img_end_x)
    img_end_y = min(img_height, img_end_y)
    
    # Adjust matrix end coordinates if image is smaller than expected
    actual_width = img_end_x - img_offset_x
    actual_height = img_end_y - img_offset_y
    matrix_end_x = matrix_start_x + actual_width
    matrix_end_y = matrix_start_y + actual_height
    
    # Place the current image at its new position
    if actual_width > 0 and actual_height > 0:
        # Place image directly (overwriting trail in this area)
        positioned_matrix[matrix_start_y:matrix_end_y, matrix_start_x:matrix_end_x] = \
            image_matrix[img_offset_y:img_end_y, img_offset_x:img_end_x]
        
        # Update the trail matrix to include the current image position
        image_trail_matrix[matrix_start_y:matrix_end_y, matrix_start_x:matrix_end_x] = True
    
    return positioned_matrix

def reset_image_trail():
    """Reset the image trail matrix (call when hand control ends)."""
    global image_trail_matrix
    if image_trail_matrix is not None:
        image_trail_matrix.fill(False)

def update_display_differential(full_matrix):
    """
    Update display using differential updates to minimize serial transmission.
    Only sends panels that have actually changed since last update.
    """
    global diff_updater, serial_pool
    
    try:
        if ENABLE_DIFFERENTIAL_UPDATES and diff_updater:
            # Use differential update system
            packets_by_port, update_info = diff_updater.get_differential_update(full_matrix)
            
            # Send only changed panels
            if packets_by_port:
                # Send packets using persistent connections in parallel
                def send_to_controller_optimized(port, packets):
                    success = serial_pool.send_packets(port, packets)
                    if not success:
                        print(f"Failed to send {len(packets)} packets to {port}")
                        # Force full update on next transmission if there's an error
                        diff_updater.force_full_update()

                threads = []
                for port, port_packets in packets_by_port.items():
                    t = threading.Thread(target=send_to_controller_optimized, args=(port, port_packets))
                    threads.append(t)
                    t.start()
                
                # Wait for all transmissions to complete
                for t in threads:
                    t.join()
            
            # Return update info for statistics
            return update_info
        else:
            # Fallback to original full update method
            return update_display_original(full_matrix)
            
    except Exception as e:
        print(f"Differential display update error: {str(e)}")
        # Force full update on error
        if diff_updater:
            diff_updater.force_full_update()
        return update_display_original(full_matrix)

def update_display_original(full_matrix):
    """Original full update method for fallback."""
    packets = {}
    
    try:
        # Prepare packets for each controller
        for column in CONFIG["columns"]:
            port = column["controller_port"]
            start_col, end_col = column["column_range"]
            col_data = full_matrix[:, start_col:end_col+1]
            panels = [col_data[i*7:(i+1)*7, :] for i in range(8)]
            panel_packets = []
            for i, panel_slice in enumerate(panels):
                # Use optimized matrix_to_panel function
                panel_data = matrix_to_panel(panel_slice)
                panel_packets.append(create_packet(column["panel_addresses"][i], panel_data))
            packets[port] = panel_packets

        # Send packets using persistent connections in parallel
        def send_to_controller_optimized(port, packets):
            success = serial_pool.send_packets(port, packets)
            if not success:
                print(f"Failed to send packets to {port}")

        threads = []
        for port, port_packets in packets.items():
            t = threading.Thread(target=send_to_controller_optimized, args=(port, port_packets))
            threads.append(t)
            t.start()
        
        # Wait for all transmissions to complete
        for t in threads:
            t.join()
        
        # Return full update info
        return {
            'total_panels': 16,
            'changed_panels': 16,
            'bandwidth_saved': False,
            'changed_panel_ids': list(range(16))
        }
            
    except Exception as e:
        print(f"Display update error: {str(e)}")
        return None

def display_time(text_manager, x=None, y=1, font_file='./fonts/6x8.otb', 
                font_height=8, font_width=6, inverted=False, split_positions=None, font_config=None):
    """Display current time on the matrix with optional split components."""
    
    # Use font_config only as fallback when using default font
    if font_config and font_file == './fonts/6x8.otb' and font_height == 8 and font_width == 6:
        font_file = font_config['file']
        font_height = font_config['height']
        font_width = font_config['width']
    
    current_time = datetime.now().strftime("%H:%M")
    text_manager.text_elements = [elem for elem in text_manager.text_elements 
                                if not elem.get('is_time', False)]

    if split_positions:
        # Split into components and add separately
        hours = current_time[:2]
        separator = current_time[2]
        minutes = current_time[3:]

        for part, pos in zip([hours, separator, minutes], split_positions):
            if pos is None:
                continue
            text_manager.add_text(
                text=part,
                position=tuple(pos),
                font_file=font_file,
                font_height=font_height,
                font_width=font_width,
                inverted=inverted,
                center=False,
                is_time=True
            )
    else:
        # Original centered behavior
        if x is None:
            # Use cached bitmap to get accurate width instead of approximation
            time_bitmap = font_cache.get_text_bitmap(current_time, font_file, font_height, font_width)
            text_width = time_bitmap.shape[1]
            x = (text_manager.matrix_size[0] - text_width) // 2
            
        text_manager.add_text(
            text=current_time,
            position=(x, y),
            font_file=font_file,
            font_height=font_height,
            font_width=font_width,
            inverted=inverted,
            center=False,
            is_time=True
        )

def display_temperature_ext(text_manager, temp_value, x=None, y=1, font_file='./fonts/6x8.otb', font_height=8, font_width=6, inverted=False, font_config=None):
    """Display external temperature on the matrix."""
    
    # Use font_config only as fallback when using default font
    if font_config and font_file == './fonts/6x8.otb' and font_height == 8 and font_width == 6:
        font_file = font_config['file']
        font_height = font_config['height']
        font_width = font_config['width']
    
    temp_str = str(temp_value)
    text_manager.text_elements = [elem for elem in text_manager.text_elements if not elem.get('is_ext_temp', False)]
    if x is None:
        # Use cached bitmap to get accurate width instead of approximation
        temp_bitmap = font_cache.get_text_bitmap(temp_str, font_file, font_height, font_width)
        text_width = temp_bitmap.shape[1]
        x = (text_manager.matrix_size[0] - text_width) // 2
    text_manager.add_text(
        text=temp_str,
        position=(x, y),
        font_file=font_file,
        font_height=font_height,
        font_width=font_width,
        inverted=inverted,
        center=False,
        is_ext_temp=True
    )

def display_temperature_int(text_manager, temp_value, x=None, y=1, font_file='./fonts/6x8.otb', font_height=8, font_width=6, inverted=False, font_config=None):
    """Display internal temperature on the matrix."""
    
    # Use font_config only as fallback when using default font
    if font_config and font_file == './fonts/6x8.otb' and font_height == 8 and font_width == 6:
        font_file = font_config['file']
        font_height = font_config['height']
        font_width = font_config['width']
    
    temp_str = f"{float(temp_value):.1f}"
    text_manager.text_elements = [elem for elem in text_manager.text_elements if not elem.get('is_int_temp', False)]
    if x is None:
        # Use cached bitmap to get accurate width instead of approximation
        temp_bitmap = font_cache.get_text_bitmap(temp_str, font_file, font_height, font_width)
        text_width = temp_bitmap.shape[1]
        x = (text_manager.matrix_size[0] - text_width) // 2
    text_manager.add_text(
        text=temp_str,
        position=(x, y),
        font_file=font_file,
        font_height=font_height,
        font_width=font_width,
        inverted=inverted,
        center=False,
        is_int_temp=True
    )

class TextManager:
    def __init__(self, matrix_size=(56, 56)):
        self.text_elements = []
        self.matrix_size = matrix_size
        self.scroll_speed = 0.06
        self.scroll_thread = None
        self.scroll_active = False

    def add_text(self, text, position, font_file, font_height=8, font_width=6,
                 scroll=False, inverted=False, scroll_mode='loop', center=False, font_config=None, **kwargs):
        
        # Use font_config only as fallback when using default font
        if font_config and font_file == './fonts/6x8.otb' and font_height == 8 and font_width == 6:
            font_file = font_config['file']
            font_height = font_config['height']
            font_width = font_config['width']
        
        # Use font cache for optimized text rendering
        text_data = font_cache.get_text_bitmap(text, font_file, font_height, font_width, inverted)
        
        text_width = text_data.shape[1]
        if center and text_width < (self.matrix_size[0] - 2):
            centered_x = (self.matrix_size[0] - text_width) // 2
            position = (centered_x, position[1])
        window_width = self.matrix_size[0] - position[0]
        self.text_elements.append({
            'data': text_data,
            'position': position,
            'scroll': scroll,
            'scroll_mode': scroll_mode,
            'offset': 0,
            'full_width': text_data.shape[1],
            'window_width': window_width,
            'max_offset': text_data.shape[1],
            **kwargs
        })

    def start_scroll(self):
        self.scroll_active = True
        self.scroll_thread = threading.Thread(target=self._auto_scroll)
        self.scroll_thread.daemon = True
        self.scroll_thread.start()

    def _auto_scroll(self):
        while self.scroll_active:
            time.sleep(self.scroll_speed)
            for element in self.text_elements:
                if element['scroll']:
                    if element['scroll_mode'] == 'loop':
                        element['offset'] = (element['offset'] + 1) % element['full_width']
                    elif element['scroll_mode'] == 'once' and element['offset'] < element['max_offset']:
                        element['offset'] += 1

class RainSimulator:
    def __init__(self, matrix_size=(56, 56), density=0.1, speed=1, drop_length=4, wind_angle=0):
        self.matrix_size = matrix_size
        self.density = density
        self.speed = speed
        self.drop_length = drop_length
        self.wind_angle = wind_angle
        self.drops = []
        self.reset_drops()

    def render(self):
        """Generate rain matrix overlay using optimized memory allocation and vectorized operations."""
        global buffer_pool, vector_ops
        
        if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
            rain_matrix = buffer_pool.get_rain_buffer()
        else:
            rain_matrix = np.zeros(self.matrix_size, dtype=bool)
        
        if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
            return vector_ops.rain_drops_vectorized(self.drops, rain_matrix, self.drop_length)
        else:
            # Original implementation
            rain_matrix.fill(False)
            for drop in self.drops:
                for i in range(self.drop_length):
                    y_pos = int(drop['y'] - i)
                    if 0 <= y_pos < self.matrix_size[1]:
                        rain_matrix[y_pos, drop['x']] = True
            return rain_matrix
        
    def reset_drops(self):
        self.drops = []
        for x in range(self.matrix_size[0]):
            if random.random() < self.density:
                self.drops.append({
                    'x': x,
                    'y': random.randint(-self.matrix_size[1], 0),
                    'speed': self.speed
                })

    def update(self):
        for drop in self.drops:
            drop['y'] += drop['speed']
        max_y = self.matrix_size[1] + self.drop_length
        self.drops = [drop for drop in self.drops if drop['y'] < max_y]
        for x in range(self.matrix_size[0]):
            if random.random() < self.density and not any(d['x'] == x for d in self.drops):
                self.drops.append({
                    'x': x,
                    'y': random.randint(-self.matrix_size[1], 0),
                    'speed': self.speed
                })

def create_composite_matrix_optimized(base_image, text_manager):
    """Create composite matrix using optimized memory allocation and vectorized operations."""
    global buffer_pool, vector_ops
    
    if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
        composite = buffer_pool.get_composite_buffer()
        if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
            vector_ops.copy_optimized(base_image, composite)
        else:
            np.copyto(composite, base_image)
    else:
        composite = np.copy(base_image)
    
    # Apply text overlays
    for element in text_manager.text_elements:
        x, y = element['position']
        text_data = element['data']
        window_width = element['window_width']
        
        if not element['scroll']:
            visible_width = min(text_data.shape[1], CONFIG['matrix_size'][0] - x)
            if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                vector_ops.text_overlay_vectorized(
                    composite, 
                    text_data[:, :visible_width], 
                    (x, y)
                )
            else:
                composite[y:y+text_data.shape[0], x:x+visible_width] = text_data[:, :visible_width]
        else:
            offset = element['offset']
            if element['scroll_mode'] == 'loop':
                indices = (np.arange(window_width) + offset) % element['full_width']
                text_slice = text_data[:, indices]
                if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                    vector_ops.text_overlay_vectorized(composite, text_slice, (x, y))
                else:
                    composite[y:y+text_data.shape[0], x:x+window_width] = text_slice
            elif element['scroll_mode'] == 'once':
                visible_start = offset
                visible_end = min(offset + window_width, element['full_width'])
                visible_width = visible_end - visible_start
                if visible_width > 0:
                    text_slice = text_data[:, visible_start:visible_end]
                    if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                        vector_ops.text_overlay_vectorized(composite, text_slice, (x, y))
                    else:
                        composite[y:y+text_data.shape[0], x:x+visible_width] = text_slice
    
    return composite

def apply_config(display_config, font_config=None):
    text_mgr = TextManager(matrix_size=CONFIG['matrix_size'])
    text_mgr.start_scroll()
    
    # Add static text elements if provided
    if display_config.get('texts'):
        for text_config in display_config['texts']:
            # Only pass font_config if no font settings in JSON
            use_font_config = font_config if not text_config.get('font_file') else None
            
            text_mgr.add_text(
                text=text_config['text'],
                position=tuple(text_config['position']),
                font_file=text_config.get('font_file', './fonts/6x8.otb'),
                font_height=text_config.get('font_height', 8),
                font_width=text_config.get('font_width', 6),
                scroll=text_config.get('scroll', False),
                inverted=text_config.get('inverted', False),
                scroll_mode=text_config.get('scroll_mode', 'loop'),
                center=text_config.get('center', False),
                font_config=use_font_config
            )
    
    # Time configuration handling
    time_config = display_config.get('time', [{}])
    split_positions = None
    if time_config and len(time_config) > 0:
        tc = time_config[0]
        time_x, time_y = tc.get('position', (None, 1))
        time_inverted = tc.get('inverted', False)
        # Use JSON config if specified, otherwise use global font config as fallback
        time_font_file = tc.get('font_file')
        time_font_height = tc.get('font_height')
        time_font_width = tc.get('font_width')
        
        # Apply global font config only if no font settings in JSON
        if not time_font_file and font_config:
            time_font_file = font_config['file']
            time_font_height = font_config['height']
            time_font_width = font_config['width']
        elif not time_font_file:
            # Final fallback to defaults
            time_font_file = './fonts/6x8.otb'
            time_font_height = 8
            time_font_width = 6
        split_positions = tc.get('split_positions')
    else:
        time_x, time_y, time_inverted = (None, 1, False)
        # Use global font config if provided, otherwise use defaults
        if font_config:
            time_font_file = font_config['file']
            time_font_height = font_config['height']
            time_font_width = font_config['width']
        else:
            time_font_file = './fonts/6x8.otb'
            time_font_height = 8
            time_font_width = 6
                
    # Internal temperature configuration
    if display_config.get('temp_int'):
        temp_int_config = display_config['temp_int'][0]
        temp_int_x, temp_int_y = tuple(temp_int_config.get('position', (None, 1)))
        temp_int_inverted = temp_int_config.get('inverted', False)
        
        # Extract font settings from JSON, use global font config as fallback
        temp_int_font_file = temp_int_config.get('font_file')
        temp_int_font_height = temp_int_config.get('font_height')
        temp_int_font_width = temp_int_config.get('font_width')
        
        if not temp_int_font_file and font_config:
            temp_int_font_file = font_config['file']
            temp_int_font_height = font_config['height']
            temp_int_font_width = font_config['width']
        elif not temp_int_font_file:
            temp_int_font_file = './fonts/6x8.otb'
            temp_int_font_height = 8
            temp_int_font_width = 6
    else:
        temp_int_x, temp_int_y, temp_int_inverted = (None, 1, False)
        # Use global font config if provided, otherwise use defaults
        if font_config:
            temp_int_font_file = font_config['file']
            temp_int_font_height = font_config['height']
            temp_int_font_width = font_config['width']
        else:
            temp_int_font_file = './fonts/6x8.otb'
            temp_int_font_height = 8
            temp_int_font_width = 6
        
    # External temperature configuration
    if display_config.get('temp_ext'):
        temp_ext_config = display_config['temp_ext'][0]
        temp_ext_x, temp_ext_y = tuple(temp_ext_config.get('position', (None, 1)))
        temp_ext_inverted = temp_ext_config.get('inverted', False)
        
        # Extract font settings from JSON, use global font config as fallback
        temp_ext_font_file = temp_ext_config.get('font_file')
        temp_ext_font_height = temp_ext_config.get('font_height')
        temp_ext_font_width = temp_ext_config.get('font_width')
        
        if not temp_ext_font_file and font_config:
            temp_ext_font_file = font_config['file']
            temp_ext_font_height = font_config['height']
            temp_ext_font_width = font_config['width']
        elif not temp_ext_font_file:
            temp_ext_font_file = './fonts/6x8.otb'
            temp_ext_font_height = 8
            temp_ext_font_width = 6
    else:
        temp_ext_x, temp_ext_y, temp_ext_inverted = (None, 1, False)
        # Use global font config if provided, otherwise use defaults
        if font_config:
            temp_ext_font_file = font_config['file']
            temp_ext_font_height = font_config['height']
            temp_ext_font_width = font_config['width']
        else:
            temp_ext_font_file = './fonts/6x8.otb'
            temp_ext_font_height = 8
            temp_ext_font_width = 6
        
    # Image configuration (background)
    original_image = None
    if 'image_path' in display_config:
        try:
            original_image = prepare_image(display_config['image_path']).astype(bool)
        except FileNotFoundError:
            print(f"Error: Image file '{display_config['image_path']}' not found")
            if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                original_image = buffer_pool.get_base_buffer()
            else:
                original_image = np.zeros(CONFIG['matrix_size'], dtype=bool)
    else:
        if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
            original_image = buffer_pool.get_base_buffer()
        else:
            original_image = np.zeros(CONFIG['matrix_size'], dtype=bool)
        
    # Get fade parameters
    fade_enabled = display_config.get('fade_image', False)
    fade_params = display_config.get('fade_params', {})
    
    # Get trail color parameter (default to white/True if not specified)
    trail_color = display_config.get('trail_color', True)
    
    # Get hand control parameter (default to disabled/False if not specified)
    hand_control_enabled = display_config.get('hand_control', False)
    
    return ( 
        text_mgr, 
        original_image,
        (time_x, time_y, time_inverted, time_font_file, time_font_height, time_font_width, split_positions), 
        (temp_int_x, temp_int_y, temp_int_inverted, temp_int_font_file, temp_int_font_height, temp_int_font_width), 
        (temp_ext_x, temp_ext_y, temp_ext_inverted, temp_ext_font_file, temp_ext_font_height, temp_ext_font_width),
        fade_enabled,
        fade_params,
        trail_color,
        hand_control_enabled
    )

def cleanup_and_exit():
    """Cleanup function called on program exit."""
    print("\nCleaning up...")
    
    # Cleanup camera
    if 'camera_manager' in globals() and camera_manager:
        camera_manager.stop_camera()
    
    # Cleanup gesture listener
    if 'gesture_listener' in globals() and gesture_listener:
        gesture_listener.stop_listener()
        
    # Cleanup serial connections
    serial_pool.close_all_connections()
    print("Serial connections closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='LED Matrix Display Controller with Differential Updates')
    parser.add_argument('--black', action='store_true', help='All off matrix')
    parser.add_argument('--white', action='store_true', help='All on matrix')
    parser.add_argument('--config', required=False, help='Path to configuration file (single config)')
    parser.add_argument('--config-dir', required=False, help='Path to configuration directory containing JSON files')
    parser.add_argument('--interval', type=int, default=1200, help='Display duration (in seconds) for each configuration')
    parser.add_argument('--rain-simulator', action='store_true', help='Enable rain simulation')
    parser.add_argument('--rain-density', type=float, default=0.1, help='Rain density (0-1)')
    parser.add_argument('--rain-speed', type=int, default=1, help='Rain fall speed (pixels per frame)')
    parser.add_argument('--rain-length', type=int, default=4, help='Rain drop length in pixels')
    parser.add_argument('--serial-health-check', type=int, default=300, help='Perform serial health check every N seconds (default: 300)')
    
    # Camera mode arguments
    parser.add_argument('--camera', action='store_true', help='Enable live CSI camera feed mode')
    parser.add_argument('--fps', type=int, default=10, help='Camera capture FPS (default: 10)')
    parser.add_argument('--show-time', action='store_true', help='Overlay current time on camera feed')
    parser.add_argument('--time-position', nargs=2, type=int, metavar=('X', 'Y'), 
                       help='Time position (default: centered top)')
    parser.add_argument('--debug-camera', action='store_true', help='Enable camera debug output')
    
    # Hand gesture mode arguments
    parser.add_argument('--hand-gesture', action='store_true', help='Enable hand gesture detection mode')
    parser.add_argument('--zmq-port', type=str, default='tcp://127.0.0.1:5555', help='ZMQ port to listen for hand gestures')
    parser.add_argument('--gesture-debounce', type=float, default=0.1, help='Debounce time for gesture state changes (seconds)')
    parser.add_argument('--debug-gesture', action='store_true', help='Enable gesture detection debug output')
    parser.add_argument('--debug-hand-control', action='store_true', help='Enable hand control positioning and state change messages')
    parser.add_argument('--font', choices=['4x6', '6x8'], default='6x8', help='Choose font size: 4x6 or 6x8 (default: 6x8)')
    
    args = parser.parse_args()


    # Handle camera mode
    if args.camera:
        ENABLE_CAMERA_MODE = True
        print(f"Camera mode ENABLED - CSI camera at {args.fps} FPS")
    
    # Handle hand gesture mode
    ENABLE_GESTURE_MODE = args.hand_gesture
    if ENABLE_GESTURE_MODE:
        print(f"Hand gesture mode ENABLED - listening on {args.zmq_port}")

    # Register cleanup function
    import atexit
    atexit.register(cleanup_and_exit)

    # Initialize all optimization systems
    buffer_pool = MatrixBufferPool(
        matrix_size=CONFIG['matrix_size'], 
        enable_pooling=ENABLE_BUFFER_POOLING
    )
    
    vector_ops = VectorizedOperations(
        enable_vectorization=ENABLE_VECTORIZED_OPERATIONS
    )
    
    diff_updater = DifferentialUpdater(
        matrix_size=CONFIG['matrix_size'],
        enable_differential=ENABLE_DIFFERENTIAL_UPDATES
    )
    
    # Initialize camera manager if camera mode is enabled
    camera_manager = None
    if ENABLE_CAMERA_MODE:
        camera_manager = CameraManager(matrix_size=CONFIG['matrix_size'], fps=args.fps)
    
    # Initialize gesture listener if hand gesture mode is enabled OR for config mode with hand control
    gesture_listener = None
    if ENABLE_GESTURE_MODE:
        gesture_listener = HandGestureListener(zmq_port=args.zmq_port, debounce_time=args.gesture_debounce, debug_enabled=args.debug_hand_control)
    elif not ENABLE_CAMERA_MODE:  # Also enable for config mode (but not camera mode)
        gesture_listener = HandGestureListener(zmq_port="tcp://127.0.0.1:5555", debounce_time=0.1, debug_enabled=args.debug_hand_control)

    # Handle special cases for fully off or on displays.
    if args.black:
        full_matrix = np.zeros(CONFIG['matrix_size'], dtype=bool)
        update_display_differential(full_matrix)
        cleanup_and_exit()
        exit(0)
    if args.white:
        full_matrix = np.ones(CONFIG['matrix_size'], dtype=bool)
        update_display_differential(full_matrix)
        cleanup_and_exit()
        exit(0)

    # Handle camera mode - bypass config file requirements
    if ENABLE_CAMERA_MODE and camera_manager:
        # Initialize text manager if time overlay is requested
        text_manager = None
        if args.show_time:
            text_manager = TextManager(matrix_size=CONFIG['matrix_size'])
            text_manager.start_scroll()

        # Start camera
        if not camera_manager.start_camera():
            print("Failed to initialize camera. Exiting.")
            cleanup_and_exit()
            exit(1)

        print(f"Starting live camera feed at {args.fps} FPS")
        print("Press Ctrl+C to stop")

        last_displayed_time = None
        last_debug_time = 0
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Get current camera frame
                camera_frame = camera_manager.get_current_frame()
                
                # Add time overlay if requested
                if args.show_time and text_manager:
                    current_time_str = datetime.now().strftime("%H:%M")
                    if current_time_str != last_displayed_time:
                        time_x = args.time_position[0] if args.time_position else None
                        time_y = args.time_position[1] if args.time_position else 1
                        display_time(text_manager, x=time_x, y=time_y, inverted=True)
                        last_displayed_time = current_time_str
                    
                    # Create composite with text overlay
                    composite_matrix = create_composite_matrix_optimized(camera_frame, text_manager)
                else:
                    composite_matrix = camera_frame
                
                # Update display using optimized differential system
                update_info = update_display_differential(composite_matrix)
                frame_count += 1
                
                # Debug output
                if args.debug_camera and time.time() - last_debug_time >= 5:
                    elapsed = time.time() - start_time
                    actual_fps = frame_count / elapsed if elapsed > 0 else 0
                    print(f"Camera Debug - Actual FPS: {actual_fps:.1f}, Frames: {frame_count}")
                    
                    
                    if camera_manager:
                        camera_stats = camera_manager.get_camera_stats()
                        print(f"Camera Stats - Processing FPS: {camera_stats['actual_fps']:.1f}, Success Rate: {camera_stats['processing_success_rate']:.1f}%")
                    
                    last_debug_time = time.time()
                
                # Control update rate (target 10 FPS display updates)
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping camera feed...")
        except Exception as e:
            print(f"Camera error: {e}")
        finally:
            # Cleanup
            camera_manager.stop_camera()
            if text_manager:
                text_manager.scroll_active = False
            print("Camera feed stopped")
            cleanup_and_exit()
            exit(0)
    
    # Handle hand gesture mode - bypass config file requirements
    if ENABLE_GESTURE_MODE and gesture_listener:
        # Start gesture listener
        if not gesture_listener.start_listener():
            print("Failed to initialize gesture listener. Exiting.")
            cleanup_and_exit()
            exit(1)

        print("Starting hand gesture detection mode")
        print("Show an open palm to display a cursor at your hand position")
        print("Move your hand to move the cursor around the display")
        print("Press Ctrl+C to stop")

        last_gesture_state = False
        last_hand_position = (0, 0)
        last_debug_time = 0
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Check current gesture state and hand position
                current_gesture_state = gesture_listener.is_hand_control_active()
                current_hand_position = gesture_listener.get_hand_position()
                
                # Update display if gesture state or hand position changed
                position_changed = current_hand_position != last_hand_position
                state_changed = current_gesture_state != last_gesture_state
                
                if state_changed or position_changed:
                    if current_gesture_state:
                        # Open palm detected - display cursor at hand position
                        full_matrix = generate_hand_cursor_matrix(
                            CONFIG['matrix_size'], 
                            current_hand_position[0], 
                            current_hand_position[1]
                        )
                        print(f"Displaying cursor at position ({current_hand_position[0]}, {current_hand_position[1]}) (Open Palm detected)")
                    else:
                        # No open palm - display black matrix
                        full_matrix = np.zeros(CONFIG['matrix_size'], dtype=bool)
                        print("Displaying black matrix (No Open Palm)")
                    
                    # Update display using optimized differential system
                    update_info = update_display_differential(full_matrix)
                    frame_count += 1
                    last_gesture_state = current_gesture_state
                    last_hand_position = current_hand_position
                
                # Debug output
                if args.debug_gesture and time.time() - last_debug_time >= 5:
                    elapsed = time.time() - start_time
                    gesture_stats = gesture_listener.get_stats()
                    print(f"Gesture Debug - Messages: {gesture_stats['messages_received']}, "
                          f"Open Palm: {gesture_stats['open_palm_count']}, "
                          f"State Changes: {gesture_stats['debounced_changes']}/{gesture_stats['state_changes']}, "
                          f"MPS: {gesture_stats['messages_per_second']:.1f}, "
                          f"Debounce Efficiency: {gesture_stats['debounce_efficiency']*100:.1f}%, "
                          f"Display Updates: {frame_count}")
                    last_debug_time = time.time()
                
                # Control update rate
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nStopping hand gesture detection...")
        except Exception as e:
            print(f"Hand gesture error: {e}")
        finally:
            # Cleanup
            gesture_listener.stop_listener()
            print("Hand gesture detection stopped")
            cleanup_and_exit()
            exit(0)

    # Determine which configuration files to load.
    if args.config_dir:
        config_files = load_all_configs(args.config_dir)
    elif args.config:
        config_files = [args.config]
    else:
        print("No configuration provided. Use --config or --config-dir")
        exit(1)

    last_temp_update = 0
    current_temp_int = None
    current_temp_ext = None
    last_health_check = 0
    
    optimization_status = []
    if ENABLE_MEMORY_PREALLOCATION: optimization_status.append("memory")
    if ENABLE_VECTORIZED_OPERATIONS: optimization_status.append("vectorization")
    if ENABLE_DIFFERENTIAL_UPDATES: optimization_status.append("differential")
    
    print(f"Starting flipdot display with optimizations: {', '.join(optimization_status) if optimization_status else 'none'}")
    
    # Start gesture listener for config mode if available
    if gesture_listener and not ENABLE_GESTURE_MODE:
        if gesture_listener.start_listener():
            print("Hand gesture integration enabled for config mode")
        else:
            print("Warning: Hand gesture integration failed to start")
            gesture_listener = None
    
    # Main loop - Standard config-based mode
    # Get font configuration based on user selection
    font_config = get_font_config(args.font)
    
    try:
        while True:
            for config_file in config_files:
                config_data = load_config(config_file)
                text_manager, original_image, time_params, temp_int_params, temp_ext_params, fade_enabled, fade_params, trail_color, hand_control_enabled = apply_config(config_data, font_config)

                if args.rain_simulator:
                    rain_sim = RainSimulator(
                        CONFIG['matrix_size'],
                        density=args.rain_density,
                        speed=args.rain_speed,
                        drop_length=args.rain_length
                    )
                
                # State tracking for optimization
                last_displayed_time = None
                last_displayed_temp_int = None
                last_displayed_temp_ext = None
                last_fade_second = None
                current_base_matrix = None
                needs_display_update = True
                last_hand_control_state = False
                    
                config_start = time.time()
                while (time.time() - config_start) < args.interval:
                    content_changed = False
                    
                    # Check hand gesture state if gesture listener is available AND hand control is enabled in config
                    hand_control_active = False
                    hand_position = (0, 0)
                    if gesture_listener and hand_control_enabled:
                        hand_control_active = gesture_listener.is_hand_control_active()
                        hand_position = gesture_listener.get_hand_position()
                    
                    # Check if hand control state changed
                    hand_state_changed = hand_control_active != last_hand_control_state
                    if hand_state_changed:
                        content_changed = True
                        last_hand_control_state = hand_control_active
                        # Reset display state when transitioning back from hand control
                        if not hand_control_active:
                            last_displayed_time = None
                            last_displayed_temp_int = None
                            last_displayed_temp_ext = None
                            needs_display_update = True
                            reset_image_trail()  # Clear the trail when returning to normal mode
                            if args.debug_hand_control:
                                print("Hand control deactivated - returning to normal display")
                    
                    # Temperature updates (skip if hand control is active)
                    if not hand_control_active and ('temp_int' in config_data or 'temp_ext' in config_data):
                        if time.time() - last_temp_update >= 300:
                            try:
                                if 'temp_int' in config_data:
                                    cmd_int = jeedom_obj.cmd.byHumanName('[Séjour][Netatmo - Salon][Température]')
                                    new_temp_int = round(float(cmd_int['currentValue']) + 0.6, 1)
                                    if new_temp_int != current_temp_int:
                                        current_temp_int = new_temp_int
                                        content_changed = True
                                if 'temp_ext' in config_data:
                                    cmd_ext = jeedom_obj.cmd.byHumanName('[Maison][Température jardin][Température]')
                                    new_temp_ext = float(cmd_ext['currentValue'])
                                    if new_temp_ext != current_temp_ext:
                                        current_temp_ext = new_temp_ext
                                        content_changed = True
                                last_temp_update = time.time()
                            except Exception as e:
                                print(f"Temperature update failed: {str(e)}")
                    
                    # Time updates (skip if hand control is active)
                    if not hand_control_active and time_params:
                        current_time_str = datetime.now().strftime("%H:%M")
                        if current_time_str != last_displayed_time:
                            display_time(text_manager,
                                       x=time_params[0],
                                       y=time_params[1],
                                       font_file=time_params[3],
                                       font_height=time_params[4],
                                       font_width=time_params[5],
                                       inverted=time_params[2],
                                       split_positions=time_params[6],
                                       font_config=font_config)
                            last_displayed_time = current_time_str
                            content_changed = True
                    
                    # Temperature display updates (skip if hand control is active)
                    if not hand_control_active and temp_int_params and current_temp_int is not None and current_temp_int != last_displayed_temp_int:
                        display_temperature_int(text_manager,
                                              current_temp_int,
                                              x=temp_int_params[0],
                                              y=temp_int_params[1],
                                              font_file=temp_int_params[3],
                                              font_height=temp_int_params[4],
                                              font_width=temp_int_params[5],
                                              inverted=temp_int_params[2])
                        last_displayed_temp_int = current_temp_int
                        content_changed = True
                        
                    if not hand_control_active and temp_ext_params and current_temp_ext is not None and current_temp_ext != last_displayed_temp_ext:
                        display_temperature_ext(text_manager,
                                              current_temp_ext,
                                              x=temp_ext_params[0],
                                              y=temp_ext_params[1],
                                              font_file=temp_ext_params[3],
                                              font_height=temp_ext_params[4],
                                              font_width=temp_ext_params[5],
                                              inverted=temp_ext_params[2])
                        last_displayed_temp_ext = current_temp_ext
                        content_changed = True
                    
                    # Fade processing
                    current_time = datetime.now()
                    seconds_since_hour = current_time.minute * 60 + current_time.second
                    
                    if fade_enabled:
                        if seconds_since_hour != last_fade_second:
                            fade_params_dict = config_data.get('fade_params', {})
                            current_base_matrix = fade_image_optimized(
                                original_image,
                                seconds_since_hour,
                                fade_in=fade_params_dict.get('fade_in', 900),
                                display=fade_params_dict.get('display', 1800),
                                fade_out=fade_params_dict.get('fade_out', 900)
                            )
                            last_fade_second = seconds_since_hour
                            content_changed = True
                    elif current_base_matrix is None:
                        if ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                            current_base_matrix = buffer_pool.get_base_buffer()
                            if ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                                vector_ops.copy_optimized(original_image, current_base_matrix)
                            else:
                                np.copyto(current_base_matrix, original_image)
                        else:
                            current_base_matrix = original_image.copy()
                        content_changed = True

                    # Rain simulation
                    if args.rain_simulator:
                        rain_sim.update()
                        rain_matrix = rain_sim.render()
                        if ENABLE_MEMORY_PREALLOCATION and buffer_pool and ENABLE_VECTORIZED_OPERATIONS and vector_ops:
                            or_buffer = buffer_pool.get_logical_or_buffer()
                            composite_base = vector_ops.logical_or_optimized(current_base_matrix, rain_matrix, or_buffer)
                        elif ENABLE_MEMORY_PREALLOCATION and buffer_pool:
                            composite_base = buffer_pool.get_temp_buffer()
                            np.logical_or(current_base_matrix, rain_matrix, out=composite_base)
                        else:
                            composite_base = np.logical_or(current_base_matrix, rain_matrix)
                        content_changed = True
                    else:
                        composite_base = current_base_matrix

                    # Display update
                    has_scrolling_text = any(elem.get('scroll', False) for elem in text_manager.text_elements)
                    
                    if content_changed or has_scrolling_text or needs_display_update or hand_control_active:
                        if hand_control_active and 'image_path' in config_data:
                            # Hand control mode: position image at hand coordinates, no text overlays
                            composite_matrix = position_image_at_coordinates(
                                original_image, 
                                hand_position[0], 
                                hand_position[1], 
                                CONFIG['matrix_size'],
                                trail_color=trail_color
                            )
                            if args.debug_hand_control:
                                print(f"Hand control: Positioning image at ({hand_position[0]}, {hand_position[1]})")
                        else:
                            # Normal mode: composite with text overlays
                            composite_matrix = create_composite_matrix_optimized(composite_base, text_manager)
                        
                        update_info = update_display_differential(composite_matrix)
                        needs_display_update = False


                    if time.time() - last_health_check >= args.serial_health_check:
                        healthy, total = serial_pool.health_check()
                        if total > 0:
                            print(f"Serial health check: {healthy}/{total} connections healthy")
                        last_health_check = time.time()

                    time.sleep(0.1)
                    
    except KeyboardInterrupt:
        print("\nReceived interrupt signal...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        cleanup_and_exit()

