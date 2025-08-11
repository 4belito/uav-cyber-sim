"""Helper module for setting up logging configuration for UAV Cyber Sim."""

import logging
import os


def setup_logging(filename: str, verbose: int = 1, console_output: bool = True):
    """
    Set up logging configuration with flexible output options.

    Args:
        filename: Name for the log file (without .log extension)
        verbose: Verbosity level (0=silent, 1=info, 2=debug, 3=trace/all)
        console_output: Whether to output to console in addition to file

    """
    # Use process ID to create unique log files for each process
    os.makedirs("log", exist_ok=True)

    # Remove existing handlers to avoid conflicts
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Create formatter
    file_formatter = logging.Formatter(
        "%(asctime)s - PID:%(process)d - %(name)s - %(levelname)s - %(message)s",
        datefmt="%H:%M:%S",
    )

    # File handler (always log everything to file)
    file_handler = logging.FileHandler(f"log/{filename}.log", mode="w")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(file_formatter)

    handlers: list[logging.Handler] = [file_handler]

    # Console handler (based on verbosity and console_output flag)
    if console_output:
        console_handler = logging.StreamHandler()

        # Map verbose levels to logging levels
        if verbose == 0:
            console_handler.setLevel(logging.CRITICAL + 1)  # Silent - suppress all
        elif verbose == 1:
            console_handler.setLevel(logging.INFO)  # Normal - INFO and above
        elif verbose == 2:
            console_handler.setLevel(logging.DEBUG)  # Debug - DEBUG and above
        else:  # verbose >= 3
            console_handler.setLevel(
                logging.DEBUG
            )  # Trace - everything (same as debug for now)

        console_formatter = logging.Formatter(
            f"%(asctime)s - {filename} - %(levelname)s - %(message)s",
            datefmt="%H:%M:%S",
        )
        console_handler.setFormatter(console_formatter)
        handlers.append(console_handler)

    # Configure logging
    logging.basicConfig(level=logging.DEBUG, handlers=handlers, force=True)
