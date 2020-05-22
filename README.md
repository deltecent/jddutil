# JADE Double D Disk Controller Utility

The JADE Double D ("DD") Disk Controller for the S-100 is unique in that it has its own on-board Z80 CPU and 2K of RAM. In order for the controller to be useful to the host operating system, a Disk Controller Module (DCM) must first be loaded onto the DD. This process is usually handled by a boot loader in host ROM that loads a small boot module onto the DD. The boot module then loads the DCM from disk into the DD.

`JDDUTIL` can be used to assist with troubleshooting a JADE Double D Disk Controller. The utility provides the following functionality:

* Initialize the DD and load DCM without a boot floppy
* Format a SSSD 8" floppy
* Show the boot tracks (0-1) of a floppy
* Show the DCM command buffer
* Show the DCM internal buffers, including the sector buffer
* Show the DD  status register (43H)
* Execute a DCM command

`JDDUTIL` can be run under CP/M or stand-alone by loading a HEX file through a monitor.

```
JADE "DOUBLE D" UTILITY, VER 1.0
DELTEC ENTERPRISES LLC 2020

CMD>?

B     - SHOW BOOT TRACKS
C     - SHOW COMMAND BUFFER
D     - SHOW DCM BUFFERS
E n   - EXECUTE DCM COMMAND n
F     - FORMAT SSSD FLOPPY
I     - INITIALIZE DOUBLE D
M n   - REQUEST MEMORY BANK n
S     - DISPLAY DD STATUS REGISTER

X     - EXIT TO CP/M
```

To assemble the source code, you must use the `TDL Z80 CP/M DISK ASSEMBLER VERSION 2.21` which is available in this repository. The TDL assembler will not run on an 8080. It must be run on a Z80.
