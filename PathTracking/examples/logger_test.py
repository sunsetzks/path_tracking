"""
Test file to demonstrate logger configuration with clickable file paths in VSCode.
"""

import os
from loguru import logger
from PathTracking.utils.logger_config import setup_logger

def get_absolute_path(relative_path):
    """Convert relative path to absolute path."""
    return os.path.abspath(relative_path)

def test_function():
    """Test function to demonstrate logging."""
    # Get absolute path of this file
    current_file = get_absolute_path(__file__)
    
    logger.debug("This is a debug message")
    logger.info("This is an info message")
    logger.warning("This is a warning message")
    logger.error("This is an error message")
    
    # Test exception logging
    try:
        1/0
    except ZeroDivisionError:
        logger.exception("Caught a division by zero error")

if __name__ == "__main__":
    # Setup logger with VSCode-clickable format
    setup_logger()
    
    # Run test
    test_function() 