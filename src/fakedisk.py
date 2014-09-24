import serial, time, os

class YASNAC():  # a classs to handle the yasnac
  def __init__(self):
    self.com = serial.Serial(port='/dev/ttyS0', baudrate=4800,parity=serial.PARITY_EVEN,timeout=0.5)
    print('opened serial port.')

  def rx(self):
    packet = ''
    while self.com.inWaiting():
      r = self.com.read()
      packet += r # append the character
      time.sleep(0.005)
      if len(packet) > 7:
        if len(packet) == ord(packet[1]) + ord(packet[2]) * 256 + 5:
          break
    if len(packet) > 7:
      length = ord(packet[1]) + ord(packet[2]) * 256
      checksum = 65536 - sum([ord(c) for c in packet[1:length+3]]) # length and message
      # print '\''+str(packet[1:length+3])+'\''
      errors = ''
      if ord(packet[0]) != 2:
        errors += 'packet[0] != 2  '
      if len(packet) != (length + 5):
        errors += str(len(packet))+' len(packet) != (length + 5) '+str(length+5)+'  '
      if ord(packet[length+3]) != checksum % 256:
        errors += str(ord(packet[length+3]))+' ord(packet[length+3]) != checksum % 256 '+str(checksum % 256)+'  '
      if ord(packet[length+4]) != checksum / 256:
        errors += str(ord(packet[length+4]))+' ord(packet[length+4]) != checksum / 256 '+str(checksum / 256)
      if len(errors) > 0:
        debugmsg = ''
        for d in packet:
          if (ord(d) < 32) or (ord(d) > 126):
            debugmsg += '\\x' + d.encode('hex')
          else:
            debugmsg += d
        return '\'' + debugmsg + '\'  ' + errors
      return packet[3:length+3]
    return packet # should be empty string
      
  def  tx(self, response): # automatic or custom reply
    length = len(response)
    response = chr(length % 256)+chr(length / 256)+response
    checksum = 65536 - sum([ord(c) for c in response])
    packet = '\x02'+response+chr(checksum % 256)+chr(checksum / 256)
    self.com.write(packet)
  
moto = YASNAC()  #instantiate the class
filepath = ''
filename = 'BABY.JBI'
while True:
  packet = moto.rx()
  if packet != '':
    print packet
  if "ENQ" in packet:
    print "replying with ACK"
    time.sleep(0.005)
    moto.tx('ACK')
  elif "LST" in packet:
    print "replying with LST0001"+filename
    time.sleep(0.005)
    moto.tx("LST0001"+filename.ljust(12))
  elif "ACK" in packet:
    print "replying with EOF"
    time.sleep(0.005)
    moto.tx("EOF")
  elif "DSZ" in packet:
    print "replying with DSZ00729088"
    time.sleep(0.005)
    moto.tx("DSZ00729088") # tell it how much free space on our "disk"
  elif "FRD" in packet:
    jobFileLength = os.path.getsize(filepath+filename)
    jobFileLengthText = str(jobFileLength).rjust(8,'0')
    print "replying with FSZ"+jobFileLengthText
    time.sleep(0.005)
    moto.tx("FSZ"+jobFileLengthText) # tell it how big a file to expect
    packet = ''
    while packet == '':
      packet = moto.rx()
    print packet
    if 'ACK' not in packet:
      print 'expected ACK but received '+packet
    jobFile = open(filepath+filename,'r')
    while jobFileLength > 0:
      chunkSize = jobFileLength
      if chunkSize > 256:
        chunkSize = 256
      chunk = jobFile.read(chunkSize)
      print 'sending '+str(len(chunk))+' bytes'
      time.sleep(0.005)
      moto.tx("FRD"+chunk)
      jobFileLength -= chunkSize
      packet = ''
      while packet == '':
        packet = moto.rx()
      if 'ACK' not in packet:
        print 'expected ACK but received '+packet
    print "sending EOF"
    time.sleep(0.005)
    moto.tx('EOF')
