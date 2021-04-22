import serial
import libscrc
from enum import Enum
import time

def build_frame(dest_addr, data):
    out_buffer = bytearray([dest_addr, len(data)]);
    out_buffer.extend(data)
    crc = libscrc.modbus(out_buffer)
    out_buffer.extend(crc.to_bytes(2, byteorder='little'))

    # print('data: {}, crc: {:04x}'.format(out_buffer.hex(), crc))
    return out_buffer

class FrameState(Enum):
    ADDRESS = 1
    LENGTH = 2
    PAYLOAD = 3
    CHECKSUM = 4

def read_frame(ser):
    # Serial
    state = FrameState.ADDRESS
    address = 0
    length = 0
    payload = bytearray()
    checksum = bytearray()
    while (True):
        val = ser.read(1)
        if len(val) == 0:
            return
    
        if state == FrameState.ADDRESS:
            address = val[0]
            state = FrameState.LENGTH
        elif state == FrameState.LENGTH:
            length = val[0]
            if length == 0:
                state = FrameState.CHECKSUM
            else:
                state = FrameState.PAYLOAD
        elif state == FrameState.PAYLOAD:
            payload.extend(val)
            if length == len(payload):
                state = FrameState.CHECKSUM
        elif state == FrameState.CHECKSUM:
            checksum.extend(val)
            if 2 == len(checksum):
                break

    print("RX Frame (from: {}, length: {}, payload: {}, checksum: {})".format(address, length, payload.hex(), checksum.hex()))
    return (address, payload)

dest_address = 1
#uuid = b'\x06\x70\xff\x34\x37\x35\x55\x41\x57\x24\x42\x46'

with serial.Serial('/dev/tty.SLAB_USBtoUART', 115200, timeout=2) as ser:
    # Read Version
    #ser.write(b'\x01\x01\x03\x61\x91')
    out = build_frame(dest_address, bytearray([0x03]))
    print("out: {}".format(out.hex()))
    ser.write(out)
    (_, payload) = read_frame(ser)
    print('version: {}'.format(payload.hex()))

    
    # Read UUID
    ser.write(build_frame(dest_address, bytearray([0x01])))
    (_, payload) = read_frame(ser)
    print('uuid response: {}'.format(payload.hex()))
    uuid = payload[2:]


    # Initialize Feeder with Bad ID
    ser.write(build_frame(dest_address, bytearray([0x02, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c])))
    (_, payload) = read_frame(ser)
    print('bad feeder response: {}'.format(payload.hex()))

    # Initialize Feeder With All Zeros
    ser.write(build_frame(dest_address, bytearray([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])))
    (_, payload) = read_frame(ser)
    print('zero uuid feeder response: {}'.format(payload.hex()))

    # Initialize Feeder With All UUID
    out = bytearray([0x02])
    out.extend(uuid)
    ser.write(build_frame(dest_address, out))
    (_, payload) = read_frame(ser)
    print('good uuid feeder response: {}'.format(payload.hex()))

    # Unicast Feeder Get Feeder Address
    out = bytearray([0xC0])
    out.extend(uuid)
    ser.write(build_frame(dest_address, out))
    (_, payload) = read_frame(ser)
    print('get feeder address unicast: {}'.format(payload.hex()))

    # Broadcast Feeder Get Feeder Address
    ser.write(build_frame(0xff, out))
    (_, payload) = read_frame(ser)
    print('get feeder address broadcast: {}'.format(payload.hex()))

    for i in range(0, 5):
        # Move Forward 4.0mm 
        out = bytearray([0x04, 0x28])
        ser.write(build_frame(dest_address, out))
        (_, payload) = read_frame(ser)
        print('move forward 40mm: {}'.format(payload.hex()))
        time.sleep(1)


    for i in range(0, 5):
        # Move Backward 4.0mm 
        out = bytearray([0x05, 0x28])
        ser.write(build_frame(dest_address, out))
        (_, payload) = read_frame(ser)
        print('move backwards 40mm: {}'.format(payload.hex()))
        time.sleep(1)

    '''
    # Move Backward 8.0mm
    out = bytearray([0x05, 0x50])
    ser.write(build_frame(dest_address, out))
    (_, payload) = read_frame(ser)
    print('move backward 80mm: {}'.format(payload.hex()))

    # Move Backward 1.0mm
    out = bytearray([0x05, 0x0a])
    ser.write(build_frame(dest_address, out))
    (_, payload) = read_frame(ser)
    print('move backward 10mm: {}'.format(payload.hex()))
    '''
