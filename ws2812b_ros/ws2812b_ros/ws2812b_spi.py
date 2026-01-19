# Minimal WS2812B driver using Linux spidev (no Blinka).
# Encodes each WS bit into 3 SPI bits at ~2.4 MHz:
#   WS '1' -> 110  (T_high ~0.83us)
#   WS '0' -> 100  (T_high ~0.42us)
# Stores bytes in GRB wire order per WS2812B datasheet.

import spidev

ORDER_MAP = {
    "GRB": (1, 0, 2),  # (index of R,G,B to send as G,R,B)
    "RGB": (0, 1, 2),
    "BRG": (2, 0, 1),
    "GBR": (1, 2, 0),
    "RBG": (0, 2, 1),
    "BGR": (2, 1, 0),
}

class WS2812B_SPI:
    def __init__(self, n_leds=2, spi_bus=0, spi_dev=0, spi_hz=2_400_000,
                 brightness=1.0, pixel_order="GRB"):
        self.n = int(n_leds)
        self.brightness = float(brightness)
        self.order = ORDER_MAP.get(str(pixel_order).upper(), ORDER_MAP["GRB"])

        self.spi = spidev.SpiDev()
        self.spi.open(int(spi_bus), int(spi_dev))
        self.spi.max_speed_hz = int(spi_hz)
        self.spi.mode = 0

        # LED buffer stored as (G,R,B) tuples ready to transmit
        self.buf = [(0, 0, 0)] * self.n

        # Precompute LUT mapping 8 bits -> 24 encoded bits -> 3 bytes
        self._lut = [self._encode_byte(b) for b in range(256)]

    @staticmethod
    def _encode_byte(b):
        # Build 24 bits (MSB first), packing 8 groups of 3 bits
        out_bits = []
        for i in range(8):
            bit = (b >> (7 - i)) & 1
            out_bits += [1, 1, 0] if bit else [1, 0, 0]
        # Pack into 3 bytes MSB-first
        out = []
        val = 0
        for i, bit in enumerate(out_bits):
            val = (val << 1) | bit
            if (i % 8) == 7:
                out.append(val)
                val = 0
        return bytes(out)  # length 3

    def set_rgb(self, idx, r, g, b):
        if not (0 <= idx < self.n):
            return
        # brightness scaling
        if self.brightness < 1.0:
            r = int(r * self.brightness)
            g = int(g * self.brightness)
            b = int(b * self.brightness)
        # remap from (R,G,B) to desired pixel order, then store as GRB wire order
        channels = [r & 0xFF, g & 0xFF, b & 0xFF]
        iR, iG, iB = self.order
        r_out = channels[iR]
        g_out = channels[iG]
        b_out = channels[iB]
        # WS2812 wants bytes in GRB order on the wire
        self.buf[idx] = (g_out, r_out, b_out)

    def fill(self, r, g, b):
        for i in range(self.n):
            self.set_rgb(i, r, g, b)

    def show(self):
        encoded = bytearray()
        for (g, r, b) in self.buf:
            encoded += self._lut[g] + self._lut[r] + self._lut[b]
        # Latch/reset: >= 80 µs low. At 2.4 MHz ~ 30+ zero bytes is sufficient; send 40.
        encoded += b"\x00" * 40
        self.spi.xfer2(encoded)

    def close(self):
        try:
            self.spi.close()
        except Exception:
            pass

