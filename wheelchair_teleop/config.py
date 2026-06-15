"""
Configuration Management Module
Load and manage teleoperation settings
"""

import yaml
import logging
import os
import copy
from pathlib import Path

logger = logging.getLogger(__name__)


class Config:
    """Load and manage configuration."""
    
    # Default values
    DEFAULTS = {
        "wheelchair": {
            "can_interface": "can0",
            "device_slot": 1,
            "max_speed": 100,
        },
        "gateway": {
            "enabled": True,
            "interface": "can1",
        },
        "safety": {
            "acceleration_rate": 50.0,
            "inactivity_timeout": 5.0,
            "min_frame_interval_ms": 10.0,
        },
        "control": {
            "send_interval_ms": 10.0,
        },
        "logging": {
            "level": "INFO",
            "file": None,
        },
    }
    
    def __init__(self, config_file: str = None):
        """
        Initialize configuration.
        
        Args:
            config_file: Path to YAML config file (optional)
        """
        self.data = copy.deepcopy(self.DEFAULTS)
        
        # Load from file if provided
        if config_file and os.path.exists(config_file):
            self._load_yaml(config_file)
            logger.info(f"Configuration loaded from: {config_file}")
    
    def _load_yaml(self, filepath: str):
        """Load YAML configuration file."""
        try:
            with open(filepath, 'r') as f:
                user_config = yaml.safe_load(f) or {}
            
            # Merge user config into defaults
            self._merge_dicts(self.data, user_config)
        
        except Exception as e:
            logger.error(f"Failed to load config file: {e}")
    
    def _merge_dicts(self, target: dict, source: dict):
        """Recursively merge source dict into target dict."""
        for key, value in source.items():
            if key in target and isinstance(target[key], dict) and isinstance(value, dict):
                self._merge_dicts(target[key], value)
            else:
                target[key] = value
    
    def get(self, path: str, default=None):
        """
        Get configuration value by dot-notation path.
        
        Args:
            path: Dot-separated path (e.g., "wheelchair.can_interface")
            default: Default value if not found
        
        Returns:
            Configuration value
        """
        keys = path.split(".")
        current = self.data
        
        try:
            for key in keys:
                current = current[key]
            return current
        except (KeyError, TypeError):
            return default
    
    def save(self, filepath: str):
        """Save current configuration to YAML file."""
        try:
            with open(filepath, 'w') as f:
                yaml.dump(self.data, f, default_flow_style=False)
            logger.info(f"Configuration saved to: {filepath}")
        except Exception as e:
            logger.error(f"Failed to save configuration: {e}")
