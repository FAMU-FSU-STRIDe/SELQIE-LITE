# ws2812b_ros/cli.py
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32MultiArray

def pack_rgb(r, g, b):
    return ((int(r) & 0xFF) << 16) | ((int(g) & 0xFF) << 8) | (int(b) & 0xFF)

def parse_hex(s):
    s = s.strip()
    if s.startswith("#"):
        s = s[1:]
    if s.lower().startswith("0x"):
        s = s[2:]
    if len(s) != 6:
        raise ValueError("HEX must be exactly 6 hex digits (RRGGBB)")
    return int(s, 16)

def build_parser():
    p = argparse.ArgumentParser(
        prog="ws2812b-set",
        description="Publish a one-shot color command to /led_colors for WS2812B driver."
    )
    p.add_argument("--hex", help="Hex color in RRGGBB (e.g., FF0000 or #00FF00).")
    p.add_argument("--off", action="store_true", help="Turn LEDs off.")
    p.add_argument("--white", type=int, nargs="?", const=255, metavar="LEVEL",
                   help="Set to white (0-255). Default 255.")
    p.add_argument("rgb", nargs="*", type=int,
                   help="R G B (0-255 0-255 0-255)")

    p.add_argument("--num-leds", type=int, default=2, help="Total LEDs in the chain.")
    p.add_argument("--index", type=int, default=None,
                   help="If set, only that LED index is changed (0-based). Others set to 0.")
    p.add_argument("--topic", default="led_colors", help="Topic name (default: led_colors).")
    p.add_argument("--repeat", type=int, default=1,
                   help="Publish this many times. Default 1.")
    p.add_argument("--rate", type=float, default=20.0,
                   help="Repeat rate in Hz if --repeat > 1. Default 20 Hz.")
    return p

def resolve_color(args):
    picks = 0
    color = None
    if args.off:
        picks += 1
        color = 0
    if args.white is not None:
        picks += 1
        w = max(0, min(255, args.white))
        color = pack_rgb(w, w, w)
    if args.hex:
        picks += 1
        color = parse_hex(args.hex)
    if len(args.rgb) == 3:
        picks += 1
        r, g, b = [max(0, min(255, v)) for v in args.rgb]
        color = pack_rgb(r, g, b)

    if picks == 0:
        raise SystemExit("Provide one of: --hex RRGGBB | --off | --white [LEVEL] | R G B")
    if picks > 1:
        raise SystemExit("Choose exactly ONE of: --hex | --off | --white | R G B")
    return color

def main():
    parser = build_parser()
    args = parser.parse_args()

    color = resolve_color(args)
    n = max(1, args.num_leds)

    # Build payload
    if args.index is None:
        data = [color] * n
    else:
        if not (0 <= args.index < n):
            raise SystemExit(f"--index {args.index} out of range for --num-leds {n}")
        data = [0] * n
        data[args.index] = color

    rclpy.init()
    node = Node("ws2812b_set_cli")
    pub = node.create_publisher(UInt32MultiArray, args.topic, 10)

    msg = UInt32MultiArray()
    msg.data = data

    # publish N times at given rate
    period = 1.0 / max(1e-3, args.rate)
    for _ in range(max(1, args.repeat)):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        node.get_logger().info(f"Published to '{args.topic}': {data}")
        node.get_clock().sleep_for(rclpy.duration.Duration(seconds=period))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

