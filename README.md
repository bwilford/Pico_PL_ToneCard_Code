# Pico_PL_ToneCard_Code
PL Tone and DCS code detector for HAM radio repeater use

To build this on a R-Pi see the document "Getting started
with Raspberry Pi Pico" chapter 8. This code must be built
on a system that has the Pico-SDK installed

To build this code, go into build directory:

	mkdir build
	cd build
	cmake ..
	make

Once the code is built. Plug in the Pico holding down
the BOOTSEL button. Once the Pico comes up as an
attached storage device:

	cp pl_tones.uf2 /media/wilford/RPI-RP2

and the code will automatically start running on the
Pico. To attach the USB serial console use minicom:

	minicom -b 115200 -o -D /dev/ttyACM0

If minicom won't connect, check that the user is in the
grep dialout in /etc/group

NOTE: Cntrl-A x to exit minicom

Aligns with 6a6ac3d7ce7891626fe03427d7e523ca2ea26fa6
of the private repo - Nov 24, 2025

