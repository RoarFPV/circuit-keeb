
import keeb.hardware as hw
import math
import time

"""Actions"""

class Control:
  ACTIVE=True
  INACTIVE=False

  def handlePinChange(self, change:hw.PinStateChange):
    return False

  def update(self, ts):
    return False

class AxisControl:
  def __init__(self, hwid, feedForward=0, rate=1,  position=0, rateFunc=None, sampleFilter:Filter=None ) -> None:
    self.hwid = hwid
    self.feedForward = feedForward
    self.position = position
    self.rateFunc = rateFunc if rateFunc else lambda v: v
    self.sampleFilter = sampleFilter
    self.filtered = position
    self.rate = rate

  def update(self, change: hw.PinStateChange):
    if self.hwid != change.id:
       return False
    
    self.filtered = self.sampleFilter.apply(
      self.filtered, 
      change.next + change.raw * self.feedForward
      )
    
    self.position = self.rateFunc(self.rate, self.filtered)
    return True

class JoystickControl:
  def __init__( self, inputs:list, deadZone=0.0, onPosition=None) -> None:
      self.inputs = inputs
      self.deadZone = deadZone
      self.onPosition = onPosition

  def handlePinChange(self, change:hw.PinStateChange):
    handled = False
    for axis in self.inputs:
      if axis:
        handled |= axis.update(change)

    return handled

  def update(self, ts):
    sum = 0
    for axis in self.inputs:
      if axis:
        sum += axis.position * axis.position
        
    length = math.sqrt(sum)
  
    if( length < self.deadZone):
      return
    
    if self.onPosition:
      self.onPosition(self.inputs)

    return False

  
class TapHold(Control):
  RELEASED = 0
  PRESSED = 1
  HELD = 2

  def __init__(self, press=None, release=None, hold=None, holdMs=-1, repeat=False) -> None:
    self.lastChange = None
    self.holdMs = holdMs
    self.state = TapHold.RELEASED
    self.timeHeld = 0
    self.onPress = press
    self.onRelease = release
    self.onHold = hold if hold else lambda dt: press
    self.repeat = repeat

  def handlePinChange(self, change:hw.PinStateChange):
    self.lastChange = change.ts
    
    if self.state == TapHold.RELEASED:
      if change.next > 0:    
        self.state = TapHold.PRESSED
        if self.holdMs >= 0:
          return Control.ACTIVE
        
        if self.onPress:
          self.onPress()
          
        return Control.INACTIVE
        

    if self.state == TapHold.PRESSED or self.state == TapHold.HELD:
      if change.next <= 0:
        isheld = self.state == TapHold.HELD
        if not isheld and self.onPress:
          self.onPress()

        # released
        self.state = TapHold.RELEASED
        if self.onRelease:
          # print("release", isheld)
          self.onRelease(isheld)
        return Control.INACTIVE

    return Control.INACTIVE

  def update(self, ts):
    if self.state == TapHold.RELEASED:
      return Control.INACTIVE

    if self.state == TapHold.PRESSED or self.state == TapHold.HELD:
      dt = ts - self.lastChange
      if dt > self.holdMs and (self.state != TapHold.HELD or self.repeat):
        self.state = TapHold.HELD
        if self.onHold: 
          self.onHold(dt)
        
        # if not self.repeat:
        #   return Control.INACTIVE
        
        if self.repeat:
          self.lastChange = ts

    return Control.ACTIVE




"""



"""