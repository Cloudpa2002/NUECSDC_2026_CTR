#!/usr/bin/env python3
import serial
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

FRAME_HEADER = b"\x3c\x02"
FRAME_LEN = 17


def parse_frame(frame: bytes):
    # 校验和：B17 = B1 + ... + B16 的低 8 位
    checksum = sum(frame[:16]) & 0xFF
    if checksum != frame[16]:
        return None

    pm25 = frame[8] * 256 + frame[9]

    temp_int_raw = frame[12]
    temp_decimal = frame[13] / 10.0

    # B13 bit7 = 1 表示负温度
    if temp_int_raw & 0x80:
        temperature = -((temp_int_raw & 0x7F) + temp_decimal)
    else:
        temperature = temp_int_raw + temp_decimal

    humidity = frame[14] + frame[15] / 10.0

    return temperature, humidity, pm25


def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    buffer = bytearray()

    print(f"Reading M702 sensor from {PORT}, baudrate={BAUDRATE}")

    while True:
        data = ser.read(64)
        if data:
            buffer.extend(data)

        while len(buffer) >= FRAME_LEN:
            idx = buffer.find(FRAME_HEADER)

            if idx < 0:
                buffer.clear()
                break

            if idx > 0:
                del buffer[:idx]

            if len(buffer) < FRAME_LEN:
                break

            frame = bytes(buffer[:FRAME_LEN])
            del buffer[:FRAME_LEN]

            result = parse_frame(frame)
            if result is None:
                print("Invalid checksum, skip frame")
                continue

            temperature, humidity, pm25 = result

            print(
                f"Temperature: {temperature:.1f} °C | "
                f"Humidity: {humidity:.1f} %RH | "
                f"PM2.5: {pm25} μg/m³"
            )

        time.sleep(0.05)


if __name__ == "__main__":
    main()

