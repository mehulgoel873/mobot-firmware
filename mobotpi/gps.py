from dataclasses import dataclass
from time import sleep

from serial import Serial
from pyubx2 import (
    UBXReader,
    UBXMessage,
    SET_LAYER_RAM,
    TXN_NONE,
    UBX_PROTOCOL,
)


@dataclass
class GPSFix:
    lat: float = float("inf")
    lon: float = float("inf")
    h_acc_m: float = float("inf")   # 1-sigma, meters
    fix_type: int = 0               # 0=no fix, 3=3D
    carr_soln: int = 0              # 0=none, 1=float RTK, 2=fixed RTK

    def setup(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.stream = Serial(port, baudrate=baudrate, timeout=3)

        # Parse only UBX packets
        self.ubr = UBXReader(self.stream, protfilter=UBX_PROTOCOL)

        # Enable periodic NAV-PVT output on USB
        cfg = UBXMessage.config_set(
            SET_LAYER_RAM,
            TXN_NONE,
            [
                ("CFG_MSGOUT_UBX_NAV_PVT_USB", 1),
            ],
        )
        self.stream.write(cfg.serialize())

    def read_gps(self):
        _, parsed = self.ubr.read()

        if parsed is None:
            return

        # Ignore ACK-ACK, ACK-NAK, MON-*, etc.
        if getattr(parsed, "identity", None) != "NAV-PVT":
            return

        self.lat = parsed.lat
        self.lon = parsed.lon
        self.h_acc_m = parsed.hAcc / 1000.0   # mm -> m
        self.fix_type = parsed.fixType
        self.carr_soln = parsed.carrSoln

        print(
            f"NAV-PVT lat={self.lat:.8f}, lon={self.lon:.8f}, "
            f"hAcc={self.h_acc_m:.3f} m, fixType={self.fix_type}, carrSoln={self.carr_soln}"
        )


if __name__ == "__main__":
    gps = GPSFix()
    gps.setup()

    while True:
        gps.read_gps()
        print(f"Lat: {gps.lat:.8f}, Lon: {gps.lon:.8f}")
        print(f"Accuracy: {gps.h_acc_m:.3f} m (1σ), Fix: {gps.fix_type}, RTK: {gps.carr_soln}")
        sleep(0.1)