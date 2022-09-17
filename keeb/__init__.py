import keeb.controls as ctrl
import keeb.hardware as hw
import time

class Keeb:
  def __init__(self, keymap:dict=None, defaultLayer=0):
    self.defaultLayer = defaultLayer
    self.layer = defaultLayer
    self.layerStack = [defaultLayer]
    self.activeSet = {}
    self.keymap = keymap
    

  def pushLayer(self, l):
    if l == self.layerStack[-1]:
      return

    self.layerStack.append(l)
    print(self.layerStack)                                                                                          

  def popLayer(self):
    if len(self.layerStack) > 1:
      self.layerStack.pop()
    else:
      self.layerStack.append(0)
    
    print(self.layerStack)

  def updateMapping(self, ms, changes:slice):
    if self.keymap == None:
      return
      
    for c in changes:
      if c.id in self.activeSet:
        self.activeSet[c.id].handlePinChange(c)
        continue
      
      for l in reversed(self.layerStack):
        layer = self.keymap[l]
        if c.id in layer:
          button = layer[c.id]
          # print('button:', button)
          if button.handlePinChange(c):
            self.activeSet[c.id] = button
          break
        
          
  def update(self, changes:slice):
    ns = time.monotonic_ns()
    ms =  ns /1e6
    
    # process all hardware changes
    self.updateMapping(ms, changes)
        
    # update all active buttons
    for k in self.activeSet:
      if not self.activeSet[k].update(ms):
        del self.activeSet[k]
