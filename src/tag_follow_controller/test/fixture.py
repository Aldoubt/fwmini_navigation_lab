"""Test-only official tag36h11 ID 0 raster; not a measured robot camera."""
# Source: AprilRobotics/apriltag-imgs/tag36h11/tag36_11_00000.png
# Source Git blob: be526dd7282c962d26747cd1fca50e2116a942ee
# 10x10 includes a one-cell white margin; active black edge spans eight cells.
TAG = [
    "1111111111",
    "1000000001",
    "1011010101",
    "1001110101",
    "1001100001",
    "1010100001",
    "1001011001",
    "1000010001",
    "1000000001",
    "1111111111",
]


def image_bytes(show_tag=True, offset=60):
    width, height, cell = 640, 480, 10
    image = bytearray([255]) * (width * height)
    if show_tag:
        left = width // 2 - 50 + offset
        top = height // 2 - 50
        for row, line in enumerate(TAG):
            for col, bit in enumerate(line):
                value = 255 if bit == '1' else 0
                for dy in range(cell):
                    start = (top + row * cell + dy) * width + left + col * cell
                    image[start:start + cell] = bytes([value]) * cell
    return bytes(image)
