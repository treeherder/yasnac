import serial, time, sys

yasnac = serial.Serial(port='/dev/ttyS0', baudrate=4800,parity=serial.PARITY_EVEN,timeout=0)
disk = serial.Serial(port='/dev/ttyUSB0', baudrate=4800,parity=serial.PARITY_EVEN,timeout=0)

logfile = open('mitm.log','w')
sys.stdout.write('opened mitm.log\n')
sys.stdout.flush()

time.sleep(1)# wait for the ports to be ready (this is an arbitrary period)
sys.stdout.write('opened serial ports.\n')
sys.stdout.flush()
direction = ''
while True:
  try:
    if yasnac.inWaiting():
      if direction != 'y':
        logfile.write('yasnac: ')
      direction = 'y'
      d = yasnac.read()
      disk.write(d)
      if (ord(d) < 32) or (ord(d) > 126):
        d = '\\x' + d.encode('hex')
      sys.stdout.write(d)
      sys.stdout.flush()
      logfile.write(d)
      logfile.flush()
    if disk.inWaiting():
      if direction != 'd':
        logfile.write('disk: ')
      direction = 'd'
      d = disk.read()
      yasnac.write(d)
      if (ord(d) < 32) or (ord(d) > 126):
        d = '\\x' + d.encode('hex')
      sys.stdout.write(d)
      sys.stdout.flush()
      logfile.write(d)
      logfile.flush()
  except KeyboardInterrupt:
    print "Bye"
    logfile.close()
    sys.exit()
