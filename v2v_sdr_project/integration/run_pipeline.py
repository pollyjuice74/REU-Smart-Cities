from integration.message_builder.builder import build_cam_message
from integration.message_router.router import route_to_file

def main():
    cam_msg = build_cam_message(mock=False)
    route_to_file(cam_msg)

if __name__ == "__main__":
    main()
