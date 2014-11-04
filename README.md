# YASNAC

This repository is a collection of libraries and programs for working with a YASNAC **ERC-series** industrial robot. The project is currently divided into "disk" and "remote" components.

- The disk code emulates an ERC-series disk drive and allows files to be loaded and stored to a PC instead of an ancient single density floppy disk.

- The remote code is for interfacing with the ERC-series robot directly as a connected PC. In addition to job transfer, this enables remote operation of utility functions and state queries. 

---

## motocommand

A program for issuing system control commands to an ERC-series robot. For example: `motocommand "SVON 1"` can be used to power up the robot's servo motors.

### Usage

	usage: motocommand [-h] [-d] command [command ...]
	
	Connects to a YASNAC ERC and sends system control command(s)
	Make sure you quote each command, for example: 
	
	motocommand "SVON 1" "JWAIT 10" "START TESTJOB"
	
	Available System Control Functions:
	-----------------------------------
	CANCEL - Cancels the error status
	CYCLE n - Selects a motion cycle mode, n is between 1 and 3
	DELETE j - Deletes the specified job, j is a job name or * 
	HLOCK n - Turns operator's panel interlock on/off, n is 1 or 0
	HOLD n - Puts a hold on the robot manipulator, n is 1 or 0
	JSEQ s,n - Set job and line number; s is job name, n is line no.
	JWAIT n - Wait n seconds, return status, n is between -1 and 32767
	MDSP s - Displays a message on the console, s is up to 28 chars.
	RESET - Resets robot alarms
	SETMJ j - Makes the specified job the master job, j is a job name
	START j - Starts robot operation. j is a an OPTIONAL job name
	SVON n - Turns servo power on/off. n is 1 or 0
	
	Available Status Read Functions:
	--------------------------------
	RALARM - Lists error and alarm codes
	RJDIR j - Lists all jobs or subjobs of j, j is * or a job name
	RJSEQ - Lists the current job name, line number and step number
	RPOS - Lists the current robot position in rectangular coordinates
	RPOSJ - Lists the current robot position in joint coordinates
	RSTATS - Lists the status of several robot conditions
	
	positional arguments:
	  command      A command to send to the ERC controller
	
	optional arguments:
	  -h, --help   show this help message and exit
	  -d, --debug  Enable transaction debugging output

---

## motodisk

A server that provides a serial emulation of the YASNAC FC1 floppy disk drive.


### Usage

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


### Todo

- add some tests
- disk: create a new exception specifically for the cancel message. IOError is the wrong answer for such a large diverse block 