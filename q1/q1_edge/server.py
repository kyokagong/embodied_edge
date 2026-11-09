"""Local operator API: same-origin JSON commands, token, and bounded bodies."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import secrets
from urllib.parse import urlsplit

logger = logging.getLogger(__name__)

WEB = Path(__file__).with_name('web')


def create_server(controller, host='127.0.0.1', port=8766, allowed_hosts=None):
    token = secrets.token_urlsafe(32)
    hosts = {'localhost', '127.0.0.1', '::1'} | set(allowed_hosts or [])
    if host not in ('0.0.0.0','::'):
        hosts.add(host)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        timeout = 3
        disable_nagle_algorithm = True

        def log_message(self, *_):
            pass

        def _send(self, status, payload, content_type='application/json; charset=utf-8'):
            if not isinstance(payload, bytes):
                payload = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            if status >= 400:
                self.close_connection = True
                self.send_header('Connection','close')
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'same-origin')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'")
            try:
                self.end_headers()
                self.wfile.write(payload)
            except (BrokenPipeError,ConnectionResetError):
                self.close_connection=True
                logger.debug('http_response_abandoned path=%r',self.path)

        def _host_ok(self):
            try:
                value = self.headers.get('Host', '')
                return bool(value) and urlsplit('http://' + value).hostname in hosts
            except ValueError:
                return False

        def do_GET(self):
            if not self._host_ok():
                self._send(403, {'ok':False,'error':'Host not allowed'})
                return
            path = urlsplit(self.path).path
            if path == '/api/state':
                self._send(200, controller.state())
                return
            files = {'/':('index.html','text/html; charset=utf-8'), '/app.js':('app.js','text/javascript; charset=utf-8'), '/style.css':('style.css','text/css; charset=utf-8')}
            if path not in files:
                self._send(404, {'ok':False,'error':'Not found'})
                return
            name, mime = files[path]
            file = WEB / name
            if not file.is_file():
                self._send(503, {'ok':False,'error':'UI assets unavailable'})
                return
            payload = file.read_bytes()
            if name == 'index.html':
                payload = payload.replace(b'__Q1_API_TOKEN__', token.encode())
            self._send(200, payload, mime)

        def do_POST(self):
            host_header = self.headers.get('Host', '')
            origin = self.headers.get('Origin')
            if not self._host_ok() or (origin and origin not in ('http://' + host_header, 'https://' + host_header)):
                self._send(403, {'ok':False,'error':'Origin or Host not allowed'})
                return
            supplied = self.headers.get('X-Q1-Token', '')
            if not secrets.compare_digest(supplied, token):
                self._send(403, {'ok':False,'error':'Missing or invalid operator token'})
                return
            if self.headers.get('Content-Type','').split(';')[0].strip().lower() != 'application/json':
                self._send(415, {'ok':False,'error':'JSON required'})
                return
            try:
                size = int(self.headers.get('Content-Length','0'))
            except ValueError:
                size = -1
            if not 0 < size <= 32768:
                self._send(413 if size > 32768 else 400, {'ok':False,'error':'Invalid request length'})
                return
            self.connection.settimeout(3)
            try:
                data = json.loads(self.rfile.read(size), parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Non-finite JSON')))
                if not isinstance(data, dict):
                    raise ValueError('Request must be an object')
                path = urlsplit(self.path).path
                compact = path == '/api/input' and self.headers.get('X-Q1-Compact') == '1'
                if path == '/api/action':
                    if data.get('action') not in ('stop','disable') and (
                        not isinstance(data.get('client_id'), str) or not 1 <= len(data['client_id']) <= 80
                    ):
                        raise ValueError('Action requires a valid client_id')
                    result = controller.action(data.get('action'), client_id=data.get('client_id'))
                elif path == '/api/input':
                    result = controller.input(data,compact=True) if compact else controller.input(data)
                elif path == '/api/pico':
                    if data.get('source','simulator') != 'simulator':
                        raise ValueError('Native Pico input must come from the local XRobo reader')
                    frame = data.get('frame', data)
                    result = controller.pico_frame(frame, source='simulator')
                else:
                    self._send(404, {'ok':False,'error':'Not found'})
                    return
                self._send(200, {'ok':True,'ack':result} if compact else {'ok':True,'state':result})
            except (ValueError, TypeError, KeyError, UnicodeError, TimeoutError, RuntimeError) as error:
                logger.warning('api_rejected path=%r error=%s', self.path, error)
                self._send(400, {'ok':False,'error':str(error),'state':controller.state()})

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True
        request_queue_size = 32

    server = Server((host, port), Handler)
    server.api_token = token
    return server
