yasnac
======

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
