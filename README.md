# motodisk

floppy disk drive emulation for YASNAC ERC motoman controller
also a serial man-in-the-middle program used for watching disk operations

src/fakedisk.py pretends to be a floppy disk drive, which the YASNAC ERC can read from and save files to.

presently, fakedisk.py is hard-coded to present a single filename to the YASNAC available to load.
saving from the yasnac with any filename is fine. 

DATA:
----
data have been saved from the YASNAC to various .DAT files which are in the dat/ subdirectory

JOBS:
----
robot jobs have been saved from the YASNAC to various .JBI files which are in the jobs/ subdirectory
=======
A server that provides a serial emulation of the YASNAC FC1 floppy disk drive.


## Usage

	usage: motodisk.py [-h] [-d] [-o] [file [file ...]]
	
	MotoDisk: a software emulator for the YASNAC FC1 floppy disk drive
	
	positional arguments:
	  file             optional: if you want only certain file(s) to be available
	                   to the robot, list those files on the command line. For
	                   example this allows you to send just a single file instead
	                   of all job (.JBI) files in the current working directory
	
	optional arguments:
	  -h, --help       show this help message and exit
	  -d, --debug      enable debugging output
	  -o, --overwrite  enable existing files to be overwritten by the program


## Todo

- add some tests
- disk: create a new exception specifically for the cancel message. IOError is the wrong answer for such a large diverse block 
