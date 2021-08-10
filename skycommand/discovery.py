import re
import subprocess
import threading
import time
from multiprocessing import Pool

import requests

# Default IP range to scan for Skyserve instances.
DEFAULT_IP_RANGE = '192.168.1.*'
# Default port on which Skyserve listens, if not overridden.
DEFAULT_SKYSERVE_PORT = 5000


def discover_drones(ip_range=DEFAULT_IP_RANGE, skyserve_port=DEFAULT_SKYSERVE_PORT, parallel=False):
    """
    Discover all the drones (Skyserve instances) in the network.

    :param ip_range: String representing the IP range to scan.
    :param skyserve_port: Expected Skyserve port.
    :param parallel: Whether to execute the discovery in parallel.
    :return: A list of IP addresses of all discovered drones on the network.
    """
    network_devices = [
        (ip, skyserve_port)
        for ip in _nmap_discover_devices(ip_range)
    ]

    if parallel:
        pool = Pool()
        skyserve_instances = filter(None, pool.map(_is_skyserve, network_devices))
        pool.close()
        pool.join()
        return skyserve_instances

    return filter(None, map(_is_skyserve, network_devices))


def _nmap_discover_devices(ip_range):
    """
    Discover all IP addresses on the network using nmap.

    :param ip_range: String representing the IP range to scan.
    :return: A list of IP addresses discovered on the network.
    """
    def map_host_ip(line):
        ip = re.match('.*\s(\d+\.\d+\.\d+\.\d+).*', line)
        if ip:
            return ip.groups()[0]

    nmap_lines = subprocess.check_output(['nmap', '-n', '-sn', ip_range]).split('\n')
    return filter(None, map(map_host_ip, nmap_lines))


def _is_skyserve(ip_port):
    """
    Check if a device on the network is a Skyserve server.

    :param ip_port: Pair of (IP, expected Skyserve port) to check.
    :return: The device's IP address if it is a Skyserve server; None otherwise.
    """
    ip, skyserve_port = ip_port
    url = 'http://{ip}:{skyserve_port}/discover'.format(ip=ip, skyserve_port=DEFAULT_SKYSERVE_PORT)

    try:
        resp = requests.get(url, timeout=1.5).json()
        if resp.get('success'):
            return ip
    except:  # noqa: E722
        return


class DiscoveryDaemon(object):
    """
    Abstraction for periodically discovering drones in the network.
    """

    def __init__(
        self,
        ip_range=DEFAULT_IP_RANGE,
        skyserve_port=DEFAULT_SKYSERVE_PORT,
        refresh_interval=2000,
    ):
        """
        Create a discovery daemon instance.

        :param ip_range: String representing the IP range to scan.
        :param skyserve_port: Expected Skyserve port.
        :param refresh_interval: The number of milliseconds between each refresh.
        """
        self.ip_range = ip_range
        self.skyserve_port = skyserve_port
        self.refresh_interval = refresh_interval

        self.devices = []
        self.terminate_flag = False

    def start(self):
        """
        Start the daemon.
        """
        thread = threading.Thread(target=self._refresh_discovery, args=())
        thread.daemon = True
        thread.start()

    def stop(self):
        """
        Stop the daemon.
        """
        self.terminate_flag = True

    def discovered_drones(self):
        """
        Get a list of all currently known drones.

        :return: A list of all drones in the network discovered so far.
        """
        return self.devices

    def _refresh_discovery(self):
        """
        Search the network for Skyserve instances.
        """
        if self.terminate_flag:
            return

        self.devices = discover_drones(self.ip_range, self.skyserve_port)
        time.sleep(self.refresh_interval / 1000)
        self._refresh_discovery()
