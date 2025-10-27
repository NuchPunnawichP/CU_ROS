#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_text_encoder.srv import TextEncode
import base64, codecs, hashlib, urllib.parse

def encode_text(text, scheme):
    s = scheme.strip().lower()
    if s == "base64":
        return base64.b64encode(text.encode()).decode()
    if s == "rot13":
        return codecs.encode(text, "rot_13")
    if s == "sha256":
        return hashlib.sha256(text.encode()).hexdigest()
    if s == "url":
        return urllib.parse.quote(text, safe="")
    if s.startswith("xor:"):
        key = int(s.split(":")[1])
        bs = bytes([b ^ (key & 0xFF) for b in text.encode()])
        return base64.b64encode(bs).decode()
    raise ValueError("Unknown scheme")

class TextEncoderService(Node):
    def __init__(self):
        super().__init__("text_encode_service")
        self.srv = self.create_service(TextEncode, "text_encode", self.callback)

    def callback(self, request, response):
        self.get_logger().info(f"Request: text='{request.text}' scheme='{request.scheme}'")
        response.stamp = self.get_clock().now().to_msg()
        try:
            response.encoded = encode_text(request.text, request.scheme)
        except Exception as e:
            response.encoded = f"ERROR: {e}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TextEncoderService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
