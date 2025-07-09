from constants import TOTAL_BYTES, MSG_LEN, AAPL_LOCATE, NON_AAPL_LOCATE

# test_generator.py
"""
Unit test to verify Python packet constants match Verilog TB golden bytes.
"""

from constants import TOTAL_BYTES, MSG_LEN, AAPL_LOCATE, NON_AAPL_LOCATE

# Golden R-message bytes (first MSG_LEN bytes) as in the Verilog TB
GOLDEN_R_MESSAGE = bytes([
    0x01,                      # valid flag
    ord('R'),                 # msg_type 'R'
    AAPL_LOCATE & 0xFF,       # stock_locate low byte
    (AAPL_LOCATE >> 8) & 0xFF,# stock_locate high byte
    0x08,                     # payload length = 8
    0x00,                     # length high byte
    ord('A'), ord('A'), ord('P'), ord('L'),
    ord(' '), ord(' '), ord(' '), ord(' ')
])

def make_packet_prefix():
    """
    Construct a packet of TOTAL_BYTES bytes where the
    first MSG_LEN bytes equal the GOLDEN_R_MESSAGE,
    and the remainder is zero-padded.
    """
    return GOLDEN_R_MESSAGE + b'\x00' * (TOTAL_BYTES - MSG_LEN)

def test_r_message():
    packet = make_packet_prefix()
    assert packet[:MSG_LEN] == GOLDEN_R_MESSAGE, (
        f"Mismatch in first {MSG_LEN} bytes: "
        f"{packet[:MSG_LEN]} != {GOLDEN_R_MESSAGE}"
    )
    print(" R-message matches golden bytes (MSG_LEN, TOTAL_BYTES correct).")

if __name__ == "__main__":
    test_r_message()
# test_generator.py
"""
Expanded unit tests for the Python MoldUDP64 generator,
verifying byte-exact behavior and error-injection paths.
"""

import struct
import random
from constants import TOTAL_BYTES, MSG_LEN, AAPL_LOCATE, NON_AAPL_LOCATE
from moldudp64generator import make_raw_packet

# --- Golden References ---
# First MSG_LEN bytes of an 'R' message (AAPL)
GOLDEN_R_MESSAGE = bytes([
    0x01,                      # valid flag
    ord('R'),                  # msg_type 'R'
    AAPL_LOCATE & 0xFF,        # stock_locate low byte
    (AAPL_LOCATE >> 8) & 0xFF, # stock_locate high byte
    0x08,                      # payload length = 8
    0x00,                      # length high byte
    ord('A'), ord('A'), ord('P'), ord('L'),
    ord(' '), ord(' '), ord(' '), ord(' ')
])

def test_r_message_prefix():
    packet = make_raw_packet(
        session_id=0,
        msg_type='R',
        stock_locate=AAPL_LOCATE,
        price=0,
        shares=0
    )
    assert packet[:MSG_LEN] == GOLDEN_R_MESSAGE, \
        f"Prefix mismatch: {packet[:MSG_LEN]} != {GOLDEN_R_MESSAGE}"
    print("R-message prefix matches golden reference.")

def test_aapl_payload_and_length():
    price = 123456
    shares = 789
    packet = make_raw_packet(
        session_id=0x0102030405060708,
        msg_type='A',
        stock_locate=AAPL_LOCATE,
        price=price,
        shares=shares
    )
    # Check length field
    payload_size = 1 + 2 + 4 + 4
    length_field = int.from_bytes(packet[8:12], 'big')
    assert length_field == payload_size, f"Length field {length_field} != {payload_size}"
    # Unpack payload and verify values
    offset = 8 + 4
    msg_type = chr(packet[offset])
    stock_locate = int.from_bytes(packet[offset+1:offset+3], 'little')
    price_unpacked = int.from_bytes(packet[offset+3:offset+7], 'little')
    shares_unpacked = int.from_bytes(packet[offset+7:offset+11], 'little')
    assert msg_type == 'A'
    assert stock_locate == AAPL_LOCATE
    assert price_unpacked == price
    assert shares_unpacked == shares
    print("AAPL payload and length field correct.")

def test_non_aapl_payload():
    packet = make_raw_packet(
        session_id=0,
        msg_type='A',
        stock_locate=NON_AAPL_LOCATE,
        price=1,
        shares=2
    )
    # Ensure stock_locate bytes match
    loc = int.from_bytes(packet[9:11], 'little')
    assert loc == NON_AAPL_LOCATE, f"Stock locate mismatch: {loc} != {NON_AAPL_LOCATE}"
    print("NON-AAPL stock locate correct.")

def test_malformed_length():
    packet = make_raw_packet(
        session_id=0,
        msg_type='X',
        stock_locate=AAPL_LOCATE,
        price=0,
        shares=0,
        malformed=True
    )
    length_field = int.from_bytes(packet[8:12], 'big')
    assert length_field == TOTAL_BYTES + 1, \
        f"Malformed length not injected: {length_field}"
    print("Malformed-length flag behavior correct.")

def test_corrupt_header():
    packet_clean = make_raw_packet(
        session_id=0,
        msg_type='X',
        stock_locate=AAPL_LOCATE,
        price=0,
        shares=0
    )
    packet_corrupt = make_raw_packet(
        session_id=0,
        msg_type='X',
        stock_locate=AAPL_LOCATE,
        price=0,
        shares=0,
        corrupt=True
    )
    # Only first payload byte should differ
    assert packet_clean[12] != packet_corrupt[12], \
        "Header corrupt did not flip first byte"
    assert packet_clean[13:] == packet_corrupt[13:], \
        "Corrupt injection affected beyond first byte"
    print("Corrupt-header flag behavior correct.")

if __name__ == "__main__":
    # Run all tests
    test_r_message_prefix()
    test_aapl_payload_and_length()
    test_non_aapl_payload()
    test_malformed_length()
    test_corrupt_header()
    print("All tests passed!")