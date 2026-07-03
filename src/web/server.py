#!/usr/bin/env python3
"""
Simple HTTP server for serving the xArm web interface.
This server only serves static files and proxies API requests to the xArm API server.
"""

import http.server
import socketserver
import os
import sys
from urllib.parse import urlparse
import urllib.request
import json

class XArmWebHandler(http.server.SimpleHTTPRequestHandler):
    """Custom handler that serves static files and proxies API requests."""
    
    def __init__(self, *args, **kwargs):
        # Set the directory to serve files from
        super().__init__(*args, directory=os.path.dirname(__file__), **kwargs)
    
    def do_GET(self):
        """Handle GET requests."""
        parsed_path = urlparse(self.path)
        
        # Proxy API requests to the API server
        api_paths = [
            '/api', '/status', '/locations', '/track',
            '/connect', '/disconnect', '/move', '/clear', '/gripper', '/ws',
            '/graph', '/control', '/auth',
        ]
        if any(parsed_path.path.startswith(path) for path in api_paths):
            self.proxy_to_api_server()
        # Serve index.html for root path
        elif parsed_path.path == '/':
            self.path = '/index.html'
            super().do_GET()
        else:
            super().do_GET()
    
    def do_POST(self):
        """Handle POST requests by proxying to API server."""
        # All POST requests should go to the API server
        self.proxy_to_api_server()
    
    def proxy_to_api_server(self):
        """Proxy requests to the xArm API server."""
        api_url = f"http://localhost:8000{self.path}"
        
        try:
            # Prepare the request
            headers = {}
            if hasattr(self, 'headers'):
                for key, value in self.headers.items():
                    if key.lower() not in ['host', 'connection']:
                        headers[key] = value
            
            # Get request body for POST requests
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = None
            if content_length > 0:
                post_data = self.rfile.read(content_length)
            
            # Make the request to API server
            req = urllib.request.Request(api_url, data=post_data, headers=headers)
            req.get_method = lambda: self.command
            
            with urllib.request.urlopen(req) as response:
                # Send response status
                self.send_response(response.getcode())
                
                # Send response headers
                for key, value in response.headers.items():
                    if key.lower() != 'connection':
                        self.send_header(key, value)
                self.end_headers()
                
                # Send response body
                self.wfile.write(response.read())
                
        except urllib.error.HTTPError as e:
            self.send_response(e.code)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            error_response = {
                'error': f'API request failed: {e.reason}',
                'status_code': e.code
            }
            self.wfile.write(json.dumps(error_response).encode())
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            error_response = {
                'error': f'Proxy error: {str(e)}',
                'status_code': 500
            }
            self.wfile.write(json.dumps(error_response).encode())

def start_web_server(port=6001):
    """Start the web server."""
    handler = XArmWebHandler
    
    # Create the server with better socket handling
    try:
        httpd = socketserver.TCPServer(("", port), handler)
        # Allow address reuse to prevent "Address already in use" errors
        httpd.allow_reuse_address = True
        httpd.socket.setsockopt(socketserver.socket.SOL_SOCKET, socketserver.socket.SO_REUSEADDR, 1)
        
        print(f"xArm Web Interface running at http://localhost:{port}")
        print("Proxying API requests to http://localhost:8000")
        print("Press Ctrl+C to stop the server")
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nWeb server stopping...")
            httpd.shutdown()
            print("Web server stopped gracefully")
        except Exception as e:
            print(f"Web server error: {e}")
            httpd.shutdown()
            raise
        finally:
            httpd.server_close()
            
    except OSError as e:
        if e.errno == 48:  # Address already in use
            print(f"Error starting web server: Port {port} is already in use")
            print(f"Try using a different port: pyxarm web --port {port + 1}")
        else:
            print(f"Error starting web server: {e}")
        raise

if __name__ == "__main__":
    port = 6001
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print("Invalid port number. Using default port 6001.")
    
    start_web_server(port) 