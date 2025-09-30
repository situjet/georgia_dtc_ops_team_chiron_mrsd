#!/usr/bin/env python3
import socket
import select
import threading
import time
import os
import sys

try:
    import rclpy
    from rclpy.node import Node
    ROS_AVAILABLE = True
except Exception:
    ROS_AVAILABLE = False
import subprocess


class TcpProxy:
    """
    Minimal TCP proxy suitable for forwarding RTSP over TCP between operator and camera.

    - Listens on listen_host:listen_port for clients (operator player)
    - For each client, connects to target_host:target_port (camera RTSP server)
    - Bi-directionally pipes bytes until either side closes
    - Supports multiple simultaneous clients
    - Retries upstream connection on failure with a small backoff
    """

    def __init__(self, listen_host: str, listen_port: int, target_host: str, target_port: int, logger=None):
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.target_host = target_host
        self.target_port = target_port
        self.logger = logger or self._print
        self._stop_event = threading.Event()
        self._server_sock = None

    def _print(self, *args, **kwargs):
        print(*args, **kwargs, flush=True)

    def start(self):
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        return t

    def stop(self):
        self._stop_event.set()
        try:
            if self._server_sock:
                self._server_sock.close()
        except Exception:
            pass

    def _run(self):
        self.logger(f"RTSP TCP proxy listening on {self.listen_host}:{self.listen_port} -> {self.target_host}:{self.target_port}")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.listen_host, self.listen_port))
            server.listen(5)
            server.settimeout(1.0)
            self._server_sock = server

            while not self._stop_event.is_set():
                try:
                    client_sock, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self.logger(f"Client connected: {addr}")
                threading.Thread(target=self._handle_client, args=(client_sock, addr), daemon=True).start()

    def _handle_client(self, client_sock: socket.socket, addr):
        upstream = None
        try:
            # Connect to target (camera)
            upstream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            upstream.settimeout(5.0)
            upstream.connect((self.target_host, self.target_port))
            upstream.settimeout(None)
            self.logger(f"Connected upstream to {self.target_host}:{self.target_port} for {addr}")

            client_sock.setblocking(False)
            upstream.setblocking(False)

            sockets = [client_sock, upstream]
            while not self._stop_event.is_set():
                r, _, e = select.select(sockets, [], sockets, 1.0)
                if e:
                    break
                for s in r:
                    try:
                        data = s.recv(8192)
                        if not data:
                            raise ConnectionError("socket closed")
                        if s is client_sock:
                            upstream.sendall(data)
                        else:
                            client_sock.sendall(data)
                    except (BlockingIOError, InterruptedError):
                        continue
                    except Exception:
                        # break out to cleanup
                        r = []
                        e = [s]
                        break
                if e:
                    break
        except Exception as ex:
            self.logger(f"Proxy error for {addr}: {ex}")
        finally:
            try:
                client_sock.close()
            except Exception:
                pass
            try:
                if upstream:
                    upstream.close()
            except Exception:
                pass
            self.logger(f"Client disconnected: {addr}")


def run_standalone(listen_host: str, listen_port: int, target_host: str, target_port: int):
    proxy = TcpProxy(listen_host, listen_port, target_host, target_port)
    t = proxy.start()
    try:
        while t.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        proxy.stop()


class RtspForwarderNode(Node):
    def __init__(self):
        super().__init__('rtsp_forwarder')
        self.declare_parameter('listen_host', '10.3.1.124')
        self.declare_parameter('listen_port', 8556)
        self.declare_parameter('target_host', '0.0.0.0')
        self.declare_parameter('target_port', 8556)
        self.declare_parameter('use_iptables', True)

        listen_host = self.get_parameter('listen_host').get_parameter_value().string_value
        listen_port = int(self.get_parameter('listen_port').get_parameter_value().integer_value)
        target_host = self.get_parameter('target_host').get_parameter_value().string_value
        target_port = int(self.get_parameter('target_port').get_parameter_value().integer_value)
        use_iptables = bool(self.get_parameter('use_iptables').get_parameter_value().bool_value)

        self._proxy = None
        self._iptables_rules = []
        if use_iptables:
            ok = self._setup_iptables(listen_port, target_host, target_port)
            if ok:
                self.get_logger().info(
                    f"RTSP iptables DNAT active: :{listen_port} -> {target_host}:{target_port} (TCP). Use RTSP-over-TCP in client.")
            else:
                self.get_logger().warn("Failed to set iptables rules; falling back to userspace TCP proxy.")
                self._start_proxy(listen_host, listen_port, target_host, target_port)
        else:
            self._start_proxy(listen_host, listen_port, target_host, target_port)

    def _start_proxy(self, listen_host, listen_port, target_host, target_port):
        self._proxy = TcpProxy(listen_host, listen_port, target_host, target_port, logger=self._log)
        self._thread = self._proxy.start()
        self.get_logger().info(f"RTSP proxy active: {listen_host}:{listen_port} -> {target_host}:{target_port}. Clients must use RTSP-over-TCP.")

    def _iptables_cmd(self, args):
        try:
            subprocess.run(['iptables'] + args, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return True
        except Exception as e:
            self.get_logger().warn(f"iptables {' '.join(args)} failed: {e}")
            return False

    def _setup_iptables(self, listen_port: int, target_host: str, target_port: int) -> bool:
        # Ensure IP forwarding is enabled
        try:
            with open('/proc/sys/net/ipv4/ip_forward', 'r') as f:
                val = f.read().strip()
            if val != '1':
                self.get_logger().info('Enabling net.ipv4.ip_forward=1')
                subprocess.run(['sysctl', 'net.ipv4.ip_forward=1'], check=False)
        except Exception as e:
            self.get_logger().warn(f"Could not verify/enable ip_forward: {e}")

        # Only forward TCP RTSP control/data interleaved; UDP RTP ports are not forwarded.
        prerouting = ['-t', 'nat', '-A', 'PREROUTING', '-p', 'tcp', '--dport', str(listen_port), '-j', 'DNAT', '--to-destination', f'{target_host}:{target_port}']
        postrouting = ['-t', 'nat', '-A', 'POSTROUTING', '-p', 'tcp', '-d', target_host, '--dport', str(target_port), '-j', 'MASQUERADE']
        forward_allow = ['-A', 'FORWARD', '-p', 'tcp', '-d', target_host, '--dport', str(target_port), '-j', 'ACCEPT']
        forward_established = ['-A', 'FORWARD', '-m', 'state', '--state', 'RELATED,ESTABLISHED', '-j', 'ACCEPT']
        # Avoid duplicates using -C to check
        exists_preroute = self._iptables_cmd(['-t', 'nat', '-C', 'PREROUTING', '-p', 'tcp', '--dport', str(listen_port), '-j', 'DNAT', '--to-destination', f'{target_host}:{target_port}'])
        if not exists_preroute:
            if not self._iptables_cmd(prerouting):
                return False
            self._iptables_rules.append(['-t', 'nat', '-D', 'PREROUTING', '-p', 'tcp', '--dport', str(listen_port), '-j', 'DNAT', '--to-destination', f'{target_host}:{target_port}'])
        else:
            self.get_logger().info("PREROUTING DNAT rule already present")
        exists_postroute = self._iptables_cmd(['-t', 'nat', '-C', 'POSTROUTING', '-p', 'tcp', '-d', target_host, '--dport', str(target_port), '-j', 'MASQUERADE'])
        if not exists_postroute:
            if not self._iptables_cmd(postrouting):
                # cleanup previous
                self._cleanup_iptables()
                return False
            self._iptables_rules.append(['-t', 'nat', '-D', 'POSTROUTING', '-p', 'tcp', '-d', target_host, '--dport', str(target_port), '-j', 'MASQUERADE'])
        else:
            self.get_logger().info("POSTROUTING MASQUERADE rule already present")
        # Filter FORWARD rules
        exists_forward_allow = self._iptables_cmd(['-C', 'FORWARD', '-p', 'tcp', '-d', target_host, '--dport', str(target_port), '-j', 'ACCEPT'])
        if not exists_forward_allow:
            if self._iptables_cmd(forward_allow):
                self._iptables_rules.append(['-D', 'FORWARD', '-p', 'tcp', '-d', target_host, '--dport', str(target_port), '-j', 'ACCEPT'])
        exists_forward_est = self._iptables_cmd(['-C', 'FORWARD', '-m', 'state', '--state', 'RELATED,ESTABLISHED', '-j', 'ACCEPT'])
        if not exists_forward_est:
            if self._iptables_cmd(forward_established):
                self._iptables_rules.append(['-D', 'FORWARD', '-m', 'state', '--state', 'RELATED,ESTABLISHED', '-j', 'ACCEPT'])
        return True

    def _cleanup_iptables(self):
        # Remove rules in reverse order
        while self._iptables_rules:
            rule = self._iptables_rules.pop()
            self._iptables_cmd(rule)

    def _log(self, msg: str):
        self.get_logger().info(msg)

    def destroy_node(self):
        try:
            if self._proxy:
                self._proxy.stop()
        except Exception:
            pass
        try:
            self._cleanup_iptables()
        except Exception:
            pass
        return super().destroy_node()


def main(argv=None):
    if ROS_AVAILABLE:
        rclpy.init(args=argv)
        node = RtspForwarderNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Standalone mode via env vars for quick testing
        listen_host = os.environ.get('LISTEN_HOST', '0.0.0.0')
        listen_port = int(os.environ.get('LISTEN_PORT', '8556'))
        target_host = os.environ.get('TARGET_HOST', '10.3.1.124')
        target_port = int(os.environ.get('TARGET_PORT', '8556'))
        run_standalone(listen_host, listen_port, target_host, target_port)


if __name__ == '__main__':
    main()
