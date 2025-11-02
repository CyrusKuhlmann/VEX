import socket
import logging

logger = logging.getLogger(__name__)


class Server:
    def __init__(self, port=65432):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.listen()
        self._conn = None

    def start(self, on_connect, process_message):
        self._conn, addr = self._sock.accept()
        logger.info(f"Connected by {addr}")
        on_connect()
        while self._conn:
            try:
                data = self._conn.recv(1024)
                if not data:
                    break

                msg = data.decode().strip()
                logger.info(f"Received message: {msg}")
                response = process_message(msg)
                logger.info(f"Sending response: {response}")

                self._conn.sendall(response.encode())

            except socket.error as e:
                logger.error(f"Socket error: {e}")
                break

    def stop(self):
        """Clean shutdown of the server"""
        if self._conn:
            self._conn.close()
        self._sock.close()
        self._conn = None
