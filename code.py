import board
import usb_hid
import busio
import struct, time
import msgpack
from io import BytesIO

from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.keycode import Keycode as kc

from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_hid.mouse import Mouse

import digitalio
import analogio

import keeb
import keeb.hardware as hw
import keeb.controls as ctrls

import config


uart = busio.UART(board.D24, board.D25, baudrate=400000,receiver_buffer_size=128)

class DigitalPin(hw.Pin):
    def __init__(self, id, input):
        super().__init__(id, input)
        self.dio = digitalio.DigitalInOut(id)
        self.dio.direction = digitalio.Direction.OUTPUT
        if input:
            self.dio.direction = digitalio.Direction.INPUT
            self.dio.pull = digitalio.Pull.UP
            self.value = self.dio.value

    def sample(self):
        return not self.dio.value

    def set(self, value):
        if not self.isInput:
          self.dio.value = not value

class AnalogPin(hw.Pin):
    def __init__(self, id, input):
        super().__init__(id, input)
        self.aio = analogio.AnalogIn(id) if input else analogio.AnalogOut(id)

    def sample(self):
        return (self.aio.value) / 65536.0

    def set(self, value):
        if not self.isInput:
          self.aio.value = int((value * 65536.0))
          
          
          
analogFilter = hw.FilterLowPass(cutoffHz=100, sampleRateHz=2000)

hardware = [

  hw.PinMatrix(
    "keyboard",
    outPins=[
       DigitalPin(board.SCL, input=False),
      DigitalPin(board.SDA, input=False),
      DigitalPin(board.SCK, input=False),
      DigitalPin(board.MOSI, input=False)

      
    ],
    inPins=[
      # DigitalPin(board.D24, input=True),
      # DigitalPin(board.D25, input=True),
      DigitalPin(board.D5, input=True),
      DigitalPin(board.D6, input=True),
      DigitalPin(board.D9, input=True),
      DigitalPin(board.D10, input=True),                 
      DigitalPin(board.D11, input=True),
      DigitalPin(board.D12, input=True)
    ]
  ), 
  # hw.AnalogVoltageGroup(
  #   "joystick.x",
  #   inputPins=[AnalogPin(board.A0, input=True)],
  #   outputPins=None,
  #   sampleFilter=analogFilter
  # ),

  # hw.AnalogVoltageGroup(
  #   "joystick.y",
  #   inputPins=[AnalogPin(board.A1, input=True)],
  #   outputPins=None,
  #   sampleFilter=analogFilter
  # ),
  
  hw.PinGroup(
    "encoder.press",
    inputPins=[
      DigitalPin(board.D4, input=True),
      DigitalPin(board.D13, input=True)],
    outputPins=None    
  ),
  
  # AnalogVoltageHardware(
  #   "joystick1.x",
  #   inputPins=[AnalogPin(board.A2, input=True)],
  #   outputPins=None,
  #   sampleFilter=analogFilter
  # ),

  # AnalogVoltageHardware(
  #   "joystick1.y",
  #   inputPins=[AnalogPin(board.A3, input=True)],
  #   outputPins=None,
  #   sampleFilter=analogFilter
  # ),

]
watcher = hw.GroupWatcher(hardware=hardware, boardId=config.keeb_id)
k = keeb.Keeb()

if config.hid_enabled:
  keyboard = Keyboard(usb_hid.devices)
  keyboard_layout = KeyboardLayoutUS(keyboard)  # We're in the US :)
  mouse = Mouse(usb_hid.devices)
  joyRate = 50

  def key(p:kc):
    return keeb.controls.TapHold(
      press=lambda: keyboard.press(p), 
      release=lambda held: keyboard.release(p), 
      repeat=True)

  def mod(p:kc, h:kc, holdMs=300):
    return keeb.controls.TapHold(
      press=lambda: keyboard.press(p), 
      hold=lambda dtpp: keyboard.press(h), 
      release=lambda held: keyboard.release(h) if held else keyboard.release(p),
      holdMs=holdMs)

  def khc(p:kc, holdMs=300):
    return mod(p, kc.CONTROL, holdMs)
  
  def khs(p:kc, holdMs=300):
    return mod(p,kc.SHIFT,holdMs)

  def kha(p:kc, holdMs=300):
    return mod(p,kc.ALT, holdMs)

  def enthome(holdMs=300):
    return mod(kc.ENTER, kc.HOME, holdMs )



  navLayer = keeb.controls.TapHold(
    hold=lambda dt: k.pushLayer(3),
    release=lambda h: k.popLayer(),
    holdMs=200)

  symbolLayer = keeb.controls.TapHold(
    hold=lambda dt: k.pushLayer(2),
    release=lambda h: k.popLayer(),
    holdMs=200)

  def lk(key:kc, layer):
    return keeb.controls.TapHold(
      hold=lambda dt: k.pushLayer(layer),
      release=lambda held: k.popLayer() if held else keyboard.send(key),
      holdMs=200
    )

  def calcRate_xsqr(rate, value):
      a = value * value * rate
      return a if value >= 0 else -a

  mouseAxisFilter = hw.FilterLowPass(cutoffHz=20, sampleRateHz=1000)
  mouseWheelFilter = hw.FilterLowPass(cutoffHz=100, sampleRateHz=1000)
  
  # Board Half Ids
  r = 'r'
  l = 'l' 

  hw_keyboard = 0
  hw_joystick_x = (l, 1,0)
  hw_joystick_y = (r, 2,0)

  mouseAxis = keeb.controls.JoystickControl( deadZone=0.1, inputs=[
    keeb.controls.AxisControl(hw_joystick_y, rate=-joyRate, rateFunc=calcRate_xsqr, sampleFilter=mouseAxisFilter),
    keeb.controls.AxisControl(hw_joystick_x, rate=-joyRate, rateFunc=calcRate_xsqr, sampleFilter=mouseAxisFilter)],
    onPosition=lambda axies: mouse.move(x=int(axies[0].position), y=int(axies[1].position), wheel=0)
  )

  mouseScroll = keeb.controls.JoystickControl(deadZone=0.08,
    inputs = [
      keeb.controls.AxisControl(
        hw_joystick_x, rate=2, rateFunc=calcRate_xsqr, sampleFilter=mouseWheelFilter)],
    onPosition=lambda axies: mouse.move(w=axies[0].position, x=0, y=0)
  )

  k_qu_layer_1 = lk(kc.SEMICOLON, 2 )
  k_bs_ent = mod(p=kc.BACKSPACE, h=kc.ENTER )

  

  k.keymap = [
    
    {
    'name' : 'workman',
    #####################################################
    # * Left Half
    (l,0,2,5):key(kc.SHIFT),  (l,0,1,5):key(kc.TAB), (l,0,0,5):key(kc.ESCAPE), 
    (l,0,2,4):key(kc.Z), (l,0,1,4):lk(kc.A, 2 ),   (l,0,0,4):key(kc.Q),
    (l,0,2,3):key(kc.X), (l,0,1,3):kha(kc.S),     (l,0,0,3):key(kc.D),
    (l,0,2,2):key(kc.M), (l,0,1,2):khc(kc.H),     (l,0,0,2):key(kc.R),
    (l,0,2,1):key(kc.C), (l,0,1,1):khs(kc.T),     (l,0,0,1):key(kc.W),
    (l,0,2,0):key(kc.V), (l,0,1,0):key(kc.G),     (l,0,0,0):key(kc.B),

        
    (l,0,3,2):symbolLayer,                                         
    (l,0,3,1):navLayer,

    #Thumb           
    (l,0,3,4):key(kc.SPACE),
    (l,0,3,5):key(kc.BACKSPACE), 
    (l,0,3,0):lk(kc.END, 3), 

    #################################################                                                                
                            
    # * Right Half          
    #                                            
    (r,0,0,0):key(kc.J),  (r,0,0,1):key(kc.F),  (r,0,0,2):key(kc.U),     (r,0,0,3):key(kc.P),       (r,0,0,4):key(kc.SEMICOLON),              (r,0,0,5):key(kc.BACKSPACE),
    (r,0,1,0):key(kc.Y),  (r,0,1,1):khs(kc.N),  (r,0,1,2):khc(kc.E),     (r,0,1,3):key(kc.O),       (r,0,1,4):lk(kc.I, 2),           (r,0,1,5):key(kc.QUOTE),
    (r,0,2,0):key(kc.K),  (r,0,2,1):key(kc.L),  (r,0,2,2):key(kc.COMMA), (r,0,2,3):key(kc.PERIOD),  (r,0,2,4):key(kc.FORWARD_SLASH),  (r,0,2,5):key(kc.SHIFT),
                             

    #Thumb           
    (r,0,3,4):key(kc.SPACE),
    (r,0,3,5):key(kc.ENTER),
    (r,0,3,1):key(kc.BACKSPACE),
                       
    (r,0,3,0):lk(kc.END, 3), 
    
    hw_joystick_x:mouseAxis,
    hw_joystick_y:mouseAxis
  },
  {
    'name' : 'qwerty',
    #####################################################
    # * Left Half
    (l,0,2,5):k_bs_ent,  (l,0,1,5):key(kc.TAB), (l,0,0,5):key(kc.ESCAPE), 
    (l,0,2,4):key(kc.Z), (l,0,1,4):lk(kc.A, 2 ),   (l,0,0,4):key(kc.Q),
    (l,0,2,3):key(kc.X), (l,0,1,3):kha(kc.S),     (l,0,0,3):key(kc.W),
    (l,0,2,2):key(kc.C), (l,0,1,2):khc(kc.D),     (l,0,0,2):key(kc.E),
    (l,0,2,1):key(kc.V), (l,0,1,1):khs(kc.F),     (l,0,0,1):key(kc.R),
    (l,0,2,0):key(kc.B), (l,0,1,0):key(kc.G),     (l,0,0,0):key(kc.T),
        
        
    (l,0,3,0):mod(kc.SPACE, h=kc.GUI, holdMs=200), (l,0,3,2):navLayer,                                         
    (l,0,3,1):mod(kc.TAB,   h=kc.GUI, holdMs=200),

    #Thumb           
    (l,0,3,4):key(kc.SPACE),
    (l,0,3,5):key(kc.BACKSPACE), 

    #################################################                                                                
                            
    # * Right Half          
    #                                            
    (r,0,0,0):key(kc.Y),  (r,0,0,1):key(kc.U),  (r,0,0,2):key(kc.I),     (r,0,0,3):key(kc.O),       (r,0,0,4):key(kc.P),              (r,0,0,5):key(kc.BACKSPACE),
    (r,0,1,0):key(kc.H),  (r,0,1,1):khs(kc.J),  (r,0,1,2):khc(kc.K),     (r,0,1,3):key(kc.L),       (r,0,1,4):k_qu_layer_1,           (r,0,1,5):key(kc.QUOTE),
    (r,0,2,0):key(kc.N),  (r,0,2,1):key(kc.M),  (r,0,2,2):key(kc.COMMA), (r,0,2,3):key(kc.PERIOD),  (r,0,2,4):key(kc.FORWARD_SLASH),  (r,0,2,5):key(kc.SHIFT),
                             

    #Thumb           
    (r,0,3,4):key(kc.SPACE),
    (r,0,3,5):key(kc.ENTER),
    (r,0,3,1):key(kc.BACKSPACE),
                       
    (r,0,3,0):
    
    
    keeb.controls.TapHold(
      press=lambda:mouse.press(Mouse.LEFT_BUTTON), 
      release=lambda held:mouse.release(Mouse.LEFT_BUTTON)), 
    # mod(kc.SPACE, h=kc.GUI, holdMs=200),                                                         
    hw_joystick_x:mouseAxis,
    hw_joystick_y:mouseAxis
  },  
  {
    'name' : 'Number Symbols',
    # Left (local)
    # (l,0,3,1):navLayer,
  
    (l,0,0,4):key(kc.ONE),     (l,0,0,3):key(kc.TWO),   (l,0,0,2):key(kc.THREE),    (l,0,0,1):key(kc.FOUR),    (l,0,0,0):key(kc.FIVE),
    
    # Right
    (r,0,0,0):key(kc.SIX),        (r,0,0,1):key(kc.SEVEN),     (r,0,0,2):key(kc.EIGHT),      (r,0,0,3):key(kc.NINE),        (r,0,0,4):key(kc.ZERO), (r,0,0,5):key(kc.MINUS),
    (r,0,1,0):key(kc.HOME),       (r,0,1,1):key(kc.LEFT_ARROW), (r,0,1,2):key(kc.UP_ARROW),  (r,0,1,3):key(kc.DOWN_ARROW), (r,0,1,4):key(kc.RIGHT_ARROW), (r,0,1,5):key(kc.END),
    
    # (r,0,2,1):key(kc.), (r,0,2,2):key(kc.COMMA),
    hw_joystick_x:mouseScroll
    
  },
   {
    'name' : 'Navigation',
    # Left (local)
    # (l,0,3,1):navLayer,
  
    
    # Right
    (r,0,1,0):key(kc.HOME),       (r,0,1,1):key(kc.LEFT_ARROW), (r,0,1,2):key(kc.UP_ARROW),  (r,0,1,3):key(kc.DOWN_ARROW), (r,0,1,4):key(kc.RIGHT_ARROW), (r,0,1,5):key(kc.END),
    
    # (r,0,2,1):key(kc.), (r,0,2,2):key(kc.COMMA),
    hw_joystick_x:mouseScroll
    
  },
  {
    'name' : 'game',
    #####################################################
    # * Left Half
    (l,0,2,5):key(kc.SHIFT),  (l,0,1,5):key(kc.TAB), (l,0,0,5):key(kc.ESCAPE), 
    (l,0,2,4):key(kc.Z), (l,0,1,4):key(kc.A ),   (l,0,0,4):key(kc.Q),
    (l,0,2,3):key(kc.X), (l,0,1,3):key(kc.S),     (l,0,0,3):key(kc.D),
    (l,0,2,2):key(kc.M), (l,0,1,2):key(kc.H),     (l,0,0,2):key(kc.R),
    (l,0,2,1):key(kc.C), (l,0,1,1):key(kc.T),     (l,0,0,1):key(kc.W),
    (l,0,2,0):key(kc.V), (l,0,1,0):key(kc.G),     (l,0,0,0):key(kc.B),

        
    (l,0,3,2):navLayer,                                         
    (l,0,3,1):key(kc.HOME),

    #Thumb           
    (l,0,3,4):key(kc.SPACE),
    (l,0,3,5):key(kc.BACKSPACE), 
    (r,0,3,0):lk(kc.END, 2), 

    #################################################                                                                
                            
    # * Right Half          
    #                                            
    (r,0,0,0):key(kc.J),  (r,0,0,1):key(kc.F),  (r,0,0,2):key(kc.U),     (r,0,0,3):key(kc.P),       (r,0,0,4):key(kc.SEMICOLON),              (r,0,0,5):key(kc.BACKSPACE),
    (r,0,1,0):key(kc.Y),  (r,0,1,1):khs(kc.N),  (r,0,1,2):khc(kc.E),     (r,0,1,3):key(kc.O),       (r,0,1,4):lk(kc.I, 2),           (r,0,1,5):key(kc.QUOTE),
    (r,0,2,0):key(kc.K),  (r,0,2,1):key(kc.L),  (r,0,2,2):key(kc.COMMA), (r,0,2,3):key(kc.PERIOD),  (r,0,2,4):key(kc.FORWARD_SLASH),  (r,0,2,5):key(kc.SHIFT),
                             

    #Thumb           
    (r,0,3,4):key(kc.SPACE),
    (r,0,3,5):key(kc.ENTER),
    (r,0,3,1):key(kc.BACKSPACE),
                       
    (r,0,3,0):lk(kc.END, 2), 
    
    hw_joystick_x:mouseAxis,
    hw_joystick_y:mouseAxis
  },
  ]
  
def sendPinChange(change):
    
    b = BytesIO()
    msgpack.pack({
      'i':change.id,
      'n':change.next,
      'l':change.last,
      'r':change.raw
      
      }, b)
    b.write(b'\x0A')
    
    b.seek(0)
    # print('send:', len(b.getvalue()), b.getvalue())
    # print( msgpack.unpack(b, use_list=False))
    b.seek(0)
    
    uart.write(b.getvalue())





rc = 1
inbuf = BytesIO()



def update_remote():
  if not config.hid_enabled:
    return
  
  ns = time.monotonic_ns()
  ms =  ns /1e6
      
  if uart.in_waiting == 0:
    return
  
  try:
    r = uart.readline()
    # print("recv:", len(r),r)
    c = msgpack.unpack(BytesIO(r), use_list=False)
    print("recv", c)
  except Exception as e:
    print('error:', e)
    return
  
  try:
    if isinstance(c, dict):
      watcher.addChange(id=c['i'],
                  ts=ms,
                  last=c['l'],
                  raw=c['r'],
                  value = c['n'])
            
  except KeyError as ke:
    print('Keyerror:',ke)

while True:
  update_remote()
  
  changes = watcher.update()
  
  # if len(changes):
  #    print(changes)
  
  if config.hid_enabled:
    k.update(changes)
  else:
    for c in changes:
      sendPinChange(c)
      
  watcher.clearChanges()