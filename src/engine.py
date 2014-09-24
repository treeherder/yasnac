import serial, time, sys


class YASNAC():  # a classs to handle the yasnac
  def __init__(self):
    self.com = serial.Serial(port='/dev/ttyS0', baudrate=4800,parity=serial.PARITY_EVEN,timeout=0.5)
    #initialize the class a connection to the serial port
    time.sleep(1)# wait for the port to be ready (this is an arbitrary period)
    sys.stdout.write('opened serial port.\n')
    sys.stdout.flush()

  def rx(self, cue):
    packet = ''
    while self.com.inWaiting():
      r = self.com.read()
      packet += r # append the character
      time.sleep(0.005)
    if len(packet) > 7:
      length = ord(packet[1]) + ord(packet[2]) * 256
      checksum = 65536 - sum([ord(c) for c in packet[1:length+3]]) # length and message
      # print '\''+str(packet[1:length+3])+'\''
      if ord(packet[0]) != 2:
        print 'packet[0] != 2'
      if len(packet) != length + 5:
        print str(len(packet))+'len(packet) != length + 5'+str(length+5)
      if ord(packet[length+3]) != checksum % 256:
        print str(ord(packet[length+3]))+' ord(packet[length+3]) != checksum % 256'
      if ord(packet[length+4]) != checksum / 256:
        print str(ord(packet[length+4]))+' ord(packet[length+4]) != checksum / 256'
      return packet[3:length+3]
    return '' # empty string
      
  def  tx(self, response): # automatic or custom reply
    if response is None: 
      response = "ACK"
    length = len(response)
    response = chr(length % 256)+chr(length / 256)+response
    checksum = 65536 - sum([ord(c) for c in response])
    packet = '\x02'+response+chr(checksum % 256)+chr(checksum / 256)
    self.com.write(packet)
  
  #rx() and tx() are the two most basic methods of this class
  #handshake is an example of combining rx() and tx() 
  
  def handshake(self): # press flesh with the yasnac
    if(self.rx("ENQ")  == True):  #default value for handshake
      self.tx(None) #default value for handshake 
    else:
      print("handshake failed, no inquiry heard")    
    return(self.rx(None))  #ready for the next step

  def list_files(self, filenames): #filenames as a single string in ASCII separated by FOUR (4) spaces
    # self.tx("0273004C5354{0}1FE0".format(filenames.encode("hex")).decode("hex")) # "\x02\x73\x00LST0009123\x2e4C5354{0}1FE0"
    response = "\x02\x13\x00LST{0}\x00\xfc".format(filenames)
    checksum = sum([ord(c) for c in response])
    print response+" "+str(checksum) # checksum is 1278, any change results in NAK
    self.tx(response) #.format(filenames.encode("hex")).decode("hex")) # "\x02\x73\x00LST0009123\x2e4C5354{0}1FE0"
    return
        
# some procedural style stuff to get you started:
moto = YASNAC()  #instantiate the class
while True:
  packet = moto.rx(None)
  if packet != '':
    print packet
  if "ENQ" in packet:
    print "replying to ENQ"
    time.sleep(0.005)
    moto.tx(None)
  elif "LST" in packet:
    print "replying to LST"
    time.sleep(0.005)
    moto.tx("LST0001123.JBI     ")
  elif "ACK" in packet:
    print "replying to ACK"
    time.sleep(0.005)
    moto.tx("EOF")
