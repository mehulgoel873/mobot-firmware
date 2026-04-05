from dataclasses import dataclass

from serial import Serial
from pyubx2 import UBXMessage, UBXReader, UBX_PROTOCOL

from time import sleep


@dataclass
class GPSFix:
    lat: float = float('inf')          # degrees
    lon: float = float('inf')          # degrees
    h_acc_m: float = float('inf')      # horizontal accuracy, 1-sigma (not CEP50; CEP50 ≈ 1.18 * h_acc_m)
    fix_type: int  = 0                 # 0=no fix, 3=3D
    carr_soln: int = 0   # 0=none, 1=float RTK, 2=fixed RTK


    # Setup the GPS
    def setup(self, port: str = '/dev/ttyACM0', baudrate: int = 38400):
        self.stream = Serial(port, baudrate, timeout=3)
        self.ubr = UBXReader(self.stream, protfilter=UBX_PROTOCOL)

    # Update state with new messages
    def read_gps(self):
        for _, parsed in self.ubr:
            if parsed is None or parsed.identity != 'NAV-PVT':
                continue
            
            self.lat = parsed.lat
            self.lon = parsed.lon
            self.h_acc_m=parsed.hAcc / 1e3
            self.fix_type=parsed.fixType
            self.carr_soln=parsed.carrSoln
    


if __name__ == "__main__":
    gps = GPSFix()

    gps.setup()

    while True:
        gps.read_gps()
        print(f"Lat: {gps.lat:.8f}, Lon: {gps.lon:.8f}")
        print(f"Accuracy: {gps.h_acc_m:.3f} m (1σ), Fix: {gps.fix_type}, RTK: {gps.carr_soln}")

        sleep(0.1)
