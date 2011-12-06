'/* joyrumble ( joystick number (1-8), strong motor intensity (0-100), weak motor intensity (0-100), duration in miliseconds ) */
'/* Notice the joystick number starts from 1. */
'/* 1=/dev/input/js0 , 2=/dev/input/js1 , and so on. */

screen 14

dim as any ptr libjoyrumble
libjoyrumble=dylibload(exepath+"/libjoyrumble.so")

Dim joyrumble As Function CDECL (byval joynumber as integer, byval strong as integer, byval weak as integer, byval duration as integer) As Integer
joyrumble=DyLibSymbol(libjoyrumble,"joyrumble")

print "Function pointer :",joyrumble

joyrumble(1,50,50,4000)

for i as integer=1 to 100
	print "Rumble is happening in another thread! ";
	sleep 40,1
next i

print

print "Press any key to end."

sleep

end
