"""
Logger Configuration for PathTracking

This module configures the loguru logger with VSCode-clickable format and other settings.
"""

from loguru import logger
import sys
import os


def setup_logger():
    """
    Configure loguru logger with VSCode-clickable format and other settings.
    """
    # Remove default handler
    logger.remove()
    
    # Add new handler with VSCode-clickable format
    logger.add(
        sys.stderr,
        format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | "
        "<level>{level: <8}</level> | "
        "{file}:{line}:0 | "  # VSCode clickable format with column number
        "<level>{message}</level>",
        colorize=True,
        backtrace=True,
        diagnose=True,
        enqueue=True,
    )

    # Add file handler for persistent logging
    logger.add(
        "log/path_tracking.log",
        format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {file}:{line}:0 | {message}",
        rotation="10 MB",  # Rotate when file reaches 10MB
        retention="1 week",  # Keep logs for 1 week
        compression="zip",  # Compress rotated logs
        backtrace=True,
        diagnose=True,
        enqueue=True,
    ) 