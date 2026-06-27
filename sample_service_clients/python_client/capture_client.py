#!/usr/bin/env python3
import os
import sys

import rclpy
from rclpy.node import Node
from pgwaam_msgs.srv import CaptureDslrImage


class CaptureClient(Node):
    def __init__(self) -> None:
        super().__init__("pgwaam_dslr_capture_python_client")
        self.service_name = os.environ.get(
            "SERVICE_NAME",
            "/pgwaam/id2_rpi4/Canon_EOS_6D/capture",
        )
        self.client = self.create_client(CaptureDslrImage, self.service_name)

    def call(self) -> CaptureDslrImage.Response:
        if not self.client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f"service unavailable: {self.service_name}")

        request = CaptureDslrImage.Request()

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError("service call timed out without a response")
        return future.result()


def main() -> int:
    rclpy.init()
    node = CaptureClient()
    try:
        response = node.call()
        print(f"PYTHON_CLIENT_ACK ack_msg={response.ack_msg}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
