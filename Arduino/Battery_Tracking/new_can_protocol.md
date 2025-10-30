0x135 rio -> ESP32

xx 				 xx 		xx 		  xx xx          xx 	    xx xx
voltage * 10 (e.g., 139 = 13.9V) State override use roboRIO as PD roborio energy ESP reboot unused


0x131 ESP32 -> rio

Battery Serial (first 8 bytes) (all 0 if tag not present)

0x132 ESP32 -> rio

First use Datetime + note + cycle

xx xx xx xx xx xx xx xx
yy mm dd hh mm cycle state


0x133 ESP32 -> rio

ESP32 state state

xx    xx     xx         xx xx           xx xx       xx
State PDType Readerlock auth fail count Write count unused