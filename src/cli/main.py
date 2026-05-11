#!/usr/bin/env python3
"""
PyxArm CLI - Main entry point

Command-line interface for controlling xArm robots.

Usage:
    pyxarm api [--host HOST] [--port PORT]
    pyxarm web [--host HOST] [--port PORT]
    pyxarm --version
    pyxarm --help

Commands:
    api         Start the API server only
    web         Start the web interface (automatically starts API server)
    
Options:
    --host HOST     Host to bind the server to [default: 0.0.0.0]
    --port PORT     Port to bind the server to [default: 8000 for API, 6001 for web]
    --version       Show version information
    --help          Show this help message
"""

import argparse
import os
import sys
import signal
import threading
import time
import subprocess
from typing import List, Optional


class ServerManager:
    """Manages API and web servers with proper shutdown handling."""
    
    def __init__(self):
        self.api_process = None
        self.web_server = None
        self.shutdown_event = threading.Event()
        
    def start_api_server_process(self, host: str = "0.0.0.0", port: int = 8000):
        """Start API server in a separate process."""
        try:
            # Check for existing processes on the port and clean them up
            self.cleanup_existing_servers(port)
            
            # Start API server as a subprocess
            cmd = [
                sys.executable, "-c",
                f"""
import sys
sys.path.insert(0, '{os.getcwd()}')
from core.xarm_api_server import app
import uvicorn

if __name__ == '__main__':
    uvicorn.run(app, host='{host}', port={port}, log_level='info')
"""
            ]
            
            self.api_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                env=os.environ.copy()
            )
            return True
            
        except Exception as e:
            print(f"Failed to start API server process: {e}")
            return False
    
    def cleanup_existing_servers(self, port: int):
        """Clean up any existing processes using the specified port."""
        try:
            import psutil
            
            for proc in psutil.process_iter(['pid', 'name', 'connections']):
                try:
                    connections = proc.info['connections']
                    if connections:
                        for conn in connections:
                            if conn.laddr.port == port:
                                print(f"Cleaning up existing process on port {port} (PID: {proc.info['pid']})")
                                proc.terminate()
                                proc.wait(timeout=3)
                                break
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired):
                    pass
        except ImportError:
            # psutil not available, try simple approach
            self._simple_port_cleanup(port)
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
    
    def _simple_port_cleanup(self, port: int):
        """Simple port cleanup without psutil dependency."""
        try:
            if sys.platform != "win32":
                # On Unix-like systems, try to find and kill processes using the port
                result = subprocess.run(
                    ["lsof", "-ti", f":{port}"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0 and result.stdout.strip():
                    pids = result.stdout.strip().split('\n')
                    for pid in pids:
                        if pid.isdigit():
                            print(f"Cleaning up process PID {pid} on port {port}")
                            subprocess.run(["kill", "-TERM", pid], timeout=3)
        except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.CalledProcessError):
            # lsof not available or other error, skip cleanup
            pass
        except Exception as e:
            print(f"Warning: Error during simple cleanup: {e}")
    
    def shutdown_api_server(self):
        """Gracefully shutdown the API server."""
        if self.api_process:
            print("Shutting down API server...")
            try:
                self.api_process.terminate()
                try:
                    self.api_process.wait(timeout=5)
                    print("API server shut down gracefully")
                except subprocess.TimeoutExpired:
                    print("API server didn't respond to SIGTERM, forcing shutdown...")
                    self.api_process.kill()
                    self.api_process.wait()
                    print("API server forcefully shut down")
            except Exception as e:
                print(f"Error shutting down API server: {e}")
            finally:
                self.api_process = None
    
    def setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown."""
        def signal_handler(signum, frame):
            print(f"\nReceived signal {signum}, shutting down...")
            self.shutdown_event.set()
            self.shutdown_api_server()
            
            # Force exit after a timeout to prevent hanging
            import threading
            import time
            
            def force_exit():
                time.sleep(3)  # Give 3 seconds for cleanup
                print("Warning: Forcing exit due to timeout...")
                os._exit(0)
            
            threading.Thread(target=force_exit, daemon=True).start()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # On Windows, also handle CTRL_BREAK_EVENT
        if sys.platform == "win32":
            import atexit
            atexit.register(self.shutdown_api_server)


# Global server manager instance
server_manager = ServerManager()


def start_api_server(host: str = "0.0.0.0", port: int = 8000):
    """Start the API server only."""
    print(f"Starting PyxArm API Server...")
    print(f"Server: http://{host}:{port}")
    print(f"API Docs: http://{host}:{port}/docs")
    print("=" * 50)
    
    # Only setup signal handlers if this is the main thread
    import threading
    if threading.current_thread() is threading.main_thread():
        server_manager.setup_signal_handlers()
    
    try:
        # Import and run the API server
        from core.xarm_api_server import app
        import uvicorn
        
        uvicorn.run(
            app,
            host=host,
            port=port,
            log_level="info",
            access_log=True
        )
        
    except KeyboardInterrupt:
        print("\nAPI Server stopped by user")
    except ImportError as e:
        print(f"Error importing API server: {e}")
        print("Make sure you're in the correct directory and dependencies are installed.")
        sys.exit(1)
    except Exception as e:
        print(f"Error starting API server: {e}")
        sys.exit(1)


def start_web_server(host: str = "0.0.0.0", port: int = 6001):
    """Start the web interface with API server."""
    print(f"Starting PyxArm Web Interface...")
    print(f"Web UI: http://{host}:{port}")
    print(f"API Server: http://localhost:8000 (starting automatically)")
    print("=" * 50)
    
    import threading
    import time
    
    # Setup signal handlers
    server_manager.setup_signal_handlers()
    
    # Start API server in a separate thread (simpler approach)
    print("Starting API server...")
    api_thread = threading.Thread(target=start_api_server, args=("0.0.0.0", 8000), daemon=False)
    api_thread.start()
    
    # Wait a moment for API server to start
    time.sleep(3)
    print("API server started")
    
    try:
        # Import and run the web server
        from web.server import start_web_server as run_web_server
        run_web_server(port)
        
    except KeyboardInterrupt:
        print("\nWeb server stopped by user")
    except ImportError as e:
        print(f"Error importing web server: {e}")
        print("Make sure you're in the correct directory and web files exist.")
        sys.exit(1)
    except Exception as e:
        print(f"Error starting web server: {e}")
        sys.exit(1)
    finally:
        print("Shutting down servers...")
        # The signal handler will take care of cleanup


def show_version():
    """Show version information."""
    try:
        from cli import __version__
        print(f"PyxArm CLI v{__version__}")
        print("Comprehensive xArm robot control package")
    except ImportError:
        print("PyxArm CLI (version unknown)")


def show_help():
    """Show help information."""
    print(__doc__)


def main(args: Optional[List[str]] = None):
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        prog="pyxarm",
        description="PyxArm - xArm robot control CLI",
        add_help=False  # We'll handle help ourselves
    )
    
    # Add subcommands
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # API command
    api_parser = subparsers.add_parser("api", help="Start the API server only")
    api_parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host to bind the server to (default: 0.0.0.0)"
    )
    api_parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port to bind the server to (default: 8000)"
    )
    
    # Web command
    web_parser = subparsers.add_parser("web", help="Start the web interface (automatically starts API server)")
    web_parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host to bind the server to (default: 0.0.0.0)"
    )
    web_parser.add_argument(
        "--port",
        type=int,
        default=6001,
        help="Port to bind the server to (default: 6001)"
    )
    
    # Global options
    parser.add_argument("--version", action="store_true", help="Show version information")
    parser.add_argument("--help", action="store_true", help="Show help information")
    
    # Parse arguments
    if args is None:
        args = sys.argv[1:]
    
    parsed_args = parser.parse_args(args)
    
    # Handle global options
    if parsed_args.help or (not parsed_args.command and not parsed_args.version):
        show_help()
        return
    
    if parsed_args.version:
        show_version()
        return
    
    # Handle commands
    if parsed_args.command == "api":
        start_api_server(host=parsed_args.host, port=parsed_args.port)
    elif parsed_args.command == "web":
        start_web_server(host=parsed_args.host, port=parsed_args.port)
    else:
        print(f"Unknown command: {parsed_args.command}")
        print("Use 'pyxarm --help' for available commands")
        sys.exit(1)


if __name__ == "__main__":
    main() 