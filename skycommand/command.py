import requests


def broadcast(drone_ips, mission_id, endpoint, data={}, skyserve_port=5000):
    """
    Make a request to the Skyserve instance on multiple drones.

    :param drone_ips: IP of the drones.
    :param mission_id: Registered mission ID.
    :param endpoint: Skymission-registered endpoint for the logic callback.
    :param data: JSON data to send as input.
    :param skyserve_port: Skyserve server port.
    :return: List of response objects from making the request to each drone.
    """
    return [
        send(drone_ip, mission_id, endpoint, data, skyserve_port)
        for drone_ip in drone_ips
    ]


def send(drone_ip, mission_id, endpoint, data={}, skyserve_port=5000):
    """
    Make a request to the Skyserve instance on a single drone.

    :param drone_ip: IP of the drone.
    :param mission_id: Registered mission ID.
    :param endpoint: Skymission-registered endpoint for the logic callback.
    :param data: JSON data to send as input.
    :param skyserve_port: Skyserve server port.
    :return: Response object from making the request.
    """
    return requests.post(
        url='http://{drone_ip}:{skyserve_port}/mission/{mission_id}{endpoint}'.format(
            drone_ip=drone_ip,
            mission_id=mission_id,
            skyserve_port=skyserve_port,
            endpoint=endpoint,
        ),
        json=data,
        headers={
            'X-Skynet-Source': 'skycommand',
        }
    )
