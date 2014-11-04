# YASNAC

This repository is a collection of libraries and programs for working with a YASNAC **ERC-series** industrial robot. The project is currently divided into "disk" and "remote" components.

- The disk code emulates an ERC-series disk drive and allows files to be loaded and stored to a PC instead of an ancient single density floppy disk.

- The remote code is for interfacing with the ERC-series robot directly as a connected PC. In addition to job transfer, this enables remote operation of utility functions and state queries. 

---

## motocommand

A program for issuing system control commands to an ERC-series robot. For example: `motocommand "SVON 1"` can be used to power up the robot's servo motors.

### Usage

	usage: motocommand [-h] [-d] command [command ...]
	
	Connect to a YASNAC ERC and send system control command(s)
	
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