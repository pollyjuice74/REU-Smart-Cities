from integration.mock_car_interface.gps_input import get_latest_gps

def build_cam_message(mock=True):
    gps_data = get_latest_gps(mock)
    return {
        "type": "CAM",
        "source": "vehicle-001",
        "payload": gps_data
    }
