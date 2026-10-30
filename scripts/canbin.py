"""Read Tab5 .canbin captures (CANBIN01 and CANBIN02).

Layout mirrors components/can_logger/can_logger.h:

    32-byte header: magic, version, rec_size (24), bitrate (bus 0), epoch_us,
                    v2: bus-1 nominal bitrate, bus-1 data bitrate
    24-byte records: ts_us, id, dlc, flags, bus, cont, data[8]

CANBIN02 adds bus 1 (the FD channel). A frame with more than 8 data bytes is a
head record (first 8 bytes, `cont` = continuation records) followed by `cont`
raw 24-byte records with the rest of the payload.
"""
import struct
import sys
from dataclasses import dataclass

HDR = struct.Struct("<8sHHIQII")   # magic, ver, rec_size, bitrate, epoch_us, flags, rsvd
REC = struct.Struct("<QIBBBB8s")    # ts_us, id, dlc, flags, bus, cont, data

META = 1 << 29
EXT = 1 << 31
RTR = 1 << 30
F_GAP, F_FDF, F_BRS, F_ESI = 1, 2, 4, 8
EV = {1: "CAPTURE_START", 2: "FLASH_START", 3: "FLASH_END", 4: "OVERRUN_GAP", 5: "SEG_ROTATE"}


@dataclass
class Header:
    version: int
    bitrate: int          # bus 0
    epoch_us: int
    fd_bitrate: int       # bus 1 nominal (v2), else 0
    fd_data_bitrate: int  # bus 1 data phase (v2), 0 = classic


@dataclass
class Frame:
    ts_us: int
    id: int          # 29-bit id, without flags
    ext: bool
    rtr: bool
    meta: bool       # meta/event record: dlc is the event code
    dlc: int         # data bytes (RTR: requested length)
    flags: int       # F_* bits
    bus: int
    data: bytes

    @property
    def fd(self):
        return bool(self.flags & F_FDF)

    @property
    def gap(self):
        return bool(self.flags & F_GAP)


def read(path):
    """Yield the Header, then every Frame in file order."""
    with open(path, "rb") as f:
        raw = f.read(HDR.size)
        if len(raw) < HDR.size:
            sys.exit(f"{path}: only {len(raw)} bytes, no header (file not synced yet?)")
        magic, ver, rec_size, bitrate, epoch_us, fl, rs = HDR.unpack(raw)
        if magic not in (b"CANBIN01", b"CANBIN02"):
            sys.exit(f"{path}: not a .canbin capture (magic={magic!r})")
        if rec_size != REC.size:
            sys.exit(f"{path}: rec_size {rec_size} != {REC.size}")
        v2 = magic == b"CANBIN02"
        yield Header(ver, bitrate, epoch_us, fl if v2 else 0, rs if v2 else 0)
        while True:
            raw = f.read(REC.size)
            if len(raw) < REC.size:
                break
            ts_us, idf, dlc, flags, bus, cont, data = REC.unpack(raw)
            if not v2:
                cont = 0
            payload = data
            if cont:
                more = f.read(REC.size * cont)
                if len(more) < REC.size * cont:
                    break  # file ends inside a frame (capture still being written)
                payload = data + more
            yield Frame(ts_us, idf & 0x1FFFFFFF, bool(idf & EXT), bool(idf & RTR), bool(idf & META),
                        dlc, flags, bus, b"" if idf & (RTR | META) else payload[:dlc])


def candump_body(fr):
    """'ID#DATA', 'ID#R', or CAN FD 'ID##<flags>DATA' (flags: 1 = BRS, 2 = ESI)."""
    idstr = f"{fr.id:08X}" if fr.ext else f"{fr.id:03X}"
    if fr.rtr:
        return f"{idstr}#R"
    if fr.fd:
        fl = (1 if fr.flags & F_BRS else 0) | (2 if fr.flags & F_ESI else 0)
        return f"{idstr}##{fl:X}{fr.data.hex().upper()}"
    return f"{idstr}#{fr.data.hex().upper()}"
