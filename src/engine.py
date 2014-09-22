import serial, time, sys


class YASNAC():  # a classs to handle the yasnac
  def __init__(self):
    self.com = serial.Serial(port='/dev/ttyS0', baudrate=4800,parity=serial.PARITY_EVEN,timeout=0)
    #initialize the class a connection to the serial port
    time.sleep(1)# wait for the port to be ready (this is an arbitrary period)
    sys.stdout.write('opened serial port.\n')
    sys.stdout.flush()

  def rx(self, cue):
    packet = ''
    while self.com.inWaiting():
      r = self.com.read()
      packet += r # append the character
      # sys.stdout.write(r)
      # sys.stdout.flush()
    # sys.stdout.write('\n')
    # sys.stdout.flush()
    return packet
      
  def  tx(self, response): # automatic or custom reply
    if response is None: 
      response = "\x02\x03\x00ACK\x2e\xff"
    self.com.write(response)
  
  #rx() and tx() are the two most basic methods of this class
  #handshake is an example of combining rx() and tx() 
  
  def handshake(self): # press flesh with the yasnac
    if(self.rx("ENQ")  == True):  #default value for handshake
      self.tx(None) #default value for handshake 
    else:
      print("handshake failed, no inquiry heard")    
    return(self.rx(None))  #ready for the next step

  def list_files(self, filenames): #filenames as a single string in ASCII separated by FOUR (4) spaces
    self.tx("0273004C5354{0}1FE0".format(filenames.encode("hex")))
    
    if self.rx("ACK") == True:
      self.tx("020300454F4623".decode("hex"))
      if self.rx("EOT") == True:
        print("filenames sent... proceeding")
        
# some procedural style stuff to get you started:
moto = YASNAC()  #instantiate the class
while True:
  packet = moto.rx(None)
  if packet != '':
    print packet
  enqpos = packet.find('ENQ')
  if enqpos >= 0:
    print enqpos
    time.sleep(0.005)
    moto.tx(None)
"""if "LST" in moto.handshake():
  moto.list_files()
else:
  print ("no list requested")
  exit(0)
"""
