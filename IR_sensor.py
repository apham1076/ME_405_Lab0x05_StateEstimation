from pyb import Pin, ADC, Timer
import array

class IR_sensor_single:
    '''Reads from a single ADC pin and calibrates for black and white'''
    def __init__(self,
                 adc: ADC,
                 tim: Timer,
                 samples=100):
        
        # Initialization
        self.adc = adc
        self.tim = tim
        self.samples = samples
        self.black = 0
        self.white = 0

    def calibrate(self, color: str):
        # Create buffer to store values
        buf = array.array('H', [0] * self.samples)
        self.adc.read_timed(self.buf, self.tim)
        avg = sum(buf) / len(buf)
        if color == 'b':
            self.black = avg
        else:
            self.white = avg
        return avg
    

    def read(self):
        r = self.adc.read()
        b = self.black
        w = self.white

        # Normalize values
        denom = b - w
        if denom == 0:
            n = 0
        else:
            n = r - w / denom

        # Saturation
        if n > 1:
            n = 1.0
        elif n < 0:
            n = 0.0

        return n


class IRArray:
    """Reads from an array of ADC pins and computes line centroid."""

    def __init__(self, adcs, tim, sensor_index=None, samples=100):
        self.adcs = list(adcs)
        self.tim = tim
        self.samples = samples
        self.num = len(self.adcs)
        self.black = [0] * self.num
        self.white = [0] * self.num
        # Default indices (1..N) if not given
        self.sensor_index = sensor_index if sensor_index else list(range(1, self.num + 1))
        self.norm = [0.0] * self.num

    def calibrate(self, color: str):
        import array
        bufs = [array.array('H', [0] * self.samples) for _ in range(self.num)]
        from pyb import ADC
        ADC.read_timed_multi(tuple(self.adcs), tuple(bufs), self.tim)
        avgs = [sum(b) / len(b) for b in bufs]
        target = self.black if color == 'b' else self.white
        for i, val in enumerate(avgs):
            target[i] = val

    def read(self):
        raw = [adc.read() for adc in self.adcs]
        self.norm = []
        for i, r in enumerate(raw):
            b, w = self.black[i], self.white[i]
            denom = b - w
            n = (r - w) / denom if denom != 0 else 0.0
            n = min(max(n, 0.0), 1.0)
            self.norm.append(n)
        return self.norm

    def get_centroid(self):
        self.read()
        weighted_sum = sum(idx * val for idx, val in zip(self.sensor_index, self.norm))
        total = sum(self.norm)
        return weighted_sum / total if total != 0 else 0.0

        