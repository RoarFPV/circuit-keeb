
import math
import time


"""Hardware"""

# Pins
class PinStateChange:
  def __init__(self, id=(-1,-1), ts=0, dt=0, pins=None, last=0, raw=0, next=0, delta=0):
    self.set(id, ts, dt, pins, last, raw, next, delta)

  def set(self, id, ts, dt, pins, last, raw, next, delta):
    self.id = tuple(id)
    self.ts = ts
    self.dt = dt
    self.pins = pins
    self.last = last
    self.next = next
    self.delta = delta
    self.raw = raw
    return self

  def __repr__(self):
    return f"{self.id},{self.pins},{self.delta}"

class Pin:
    def __init__(self, id, input):
        self.id = id
        self.isInput = input
        self.value = 0

    def sample(self):
      pass

    def set(self, value):
        pass

# Pin Groups
class PinGroup:
    def __init__(self, name, inputPins, outputPins) -> None:
        self.inputPins = inputPins
        self.outputPins = outputPins
        self.name = name
        self.samples = {}

    def update(self, ts, boardid, hwid, watcher) -> int:
        changes = 0
        for ii, inp in enumerate(self.inputPins):
            id=(boardid, hwid,ii)
            sample = inp.sample()
            last = self.samples[id] if id in self.samples else (ts,0)

            if sample != last[1]:
              self.samples[id] = (ts, sample)
              # print('change', sample)

              watcher.addChange(id, sample, sample, last[1], ts)

        return changes

class GroupWatcher:
  def __init__(self, hardware:list, boardId, pinChangeBufferCount=16):
    self.pinChangePool = [ PinStateChange() for i in range(pinChangeBufferCount)]
    self.hardware = hardware
    self.changes = 0
    self.boardId = boardId

  def addChange(self, id, raw, value, last, ts):
    c = self.pinChangePool[self.changes]    
    self.changes += 1
    c.id = id
    c.next = value
    c.last = last
    c.ts = ts
    c.raw = raw
        
  def update(self) -> slice:
      ns = time.monotonic_ns()
      ms =  ns /1e6
      
      # update all hardware objects
      for h, hw in enumerate(self.hardware):
        self.changes += hw.update(ms, self.boardId, h, self)
        
      return self.pinChangePool[:self.changes]
    
  def clearChanges(self):
    self.changes = 0

# 
# Filter
# 
class Filter:
  def __init__(self) -> None:
      pass

  def apply(self, value, sample):
    pass

class FilterLowPass(Filter):
  def alpha(cutoffHz, sampleRateHz):
    rc = 1.0 / (cutoffHz * 2.0 * math.pi)
    dt = 1.0 / sampleRateHz
    return dt / (rc + dt)

  def __init__(self, cutoffHz, sampleRateHz) -> None:
      self.cutoffHz = cutoffHz
      self.sampleRateHz = sampleRateHz
      self.alphaValue = FilterLowPass.alpha(cutoffHz, sampleRateHz)

  def apply(self, value, sample):
    return value - self.alphaValue * (value - sample)


class AnalogVoltageGroup(PinGroup):
    def __init__(self, name, inputPins, outputPins, 
                 calibrationSamples=100, 
                 sampleFilter=None) -> None:
        super().__init__(name, inputPins, outputPins)
        #todo: rate and deadzone should be on button side

        self.sampleFilter = sampleFilter
        self.filtered = [ -10000 ] * len(inputPins)
        
        self.calibrate = calibrationSamples
        self.offset = [ 0.0 ] * len(inputPins)

    def update(self, ts, boardid, hwid, watcher) -> int:
        changes = 0
        for ii, inp in enumerate(self.inputPins):
            id=(boardid,int(hwid),ii)
            
            sample = ((inp.sample())-0.5)*2
            if self.filtered == -10000:
              self.filtered = sample

            self.filtered[ii] = self.sampleFilter.apply(self.filtered[ii], sample) if self.sampleFilter else sample

            if self.calibrate > 0:
              self.offset[ii] = self.filtered[ii]
              self.calibrate -= 1
              if self.calibrate == 0:
                print(f"{self.name}.offset: {self.offset}")
              continue

            # self.filtered -= self.offset

            last = self.samples[id] if id in self.samples else (ts,self.filtered[ii])
            self.samples[id] = (ts, self.filtered[ii])

            delta = last[1] - self.filtered[ii]

            # Always sample analog values
            watcher.addChange(
                id=id,
                ts=ts,
                raw=sample,
                last=last[1],
                value = self.filtered[ii])
            changes += 1
              
        return changes

class PinMatrix(PinGroup):
    def __init__(self, name, inPins, outPins) -> None:
        super().__init__(name, inPins, outPins)
        self.samples = {}

    # TODO: move multiplexing logic to pio program
    def update(self, ts, boardid, hwid, watcher) -> int:
        changes = 0
        for oi, outpin in enumerate(self.outputPins):
          outpin.set(1)

          for ii, inp in enumerate(self.inputPins):
              id=(boardid, hwid,oi,ii)
              sample = inp.sample()
              last = self.samples[id] if id in self.samples else (ts,False)

              if sample != last[1]:
                self.samples[id] = (ts, sample)

                watcher.addChange(
                    id=id,
                    ts=ts,
                    last=last[1],
                    raw=sample,
                    value = sample)
                changes += 1

          outpin.set(0)
        return changes






