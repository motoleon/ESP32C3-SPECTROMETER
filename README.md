# ESP32 C3 SPECTROMETER
IN PROCESS...

With a simple ESP32 C3, its tiny 0.42-inch OLED screen (it's super mini), and an IMMP441 microphone (or a Chinese copy), I built a spectrograph (frequency spectrum analyzer) with n bands from 20 Hz to 20 kHz. 
It seems incredible that this C3 can calculate FFTs at 44 kHz... but it can, and in real time. 

It's not emulated, it's not an approximation; the bands you configure are the bands it actually responds to in terms of frequency. 
It truly analyzes the spectrum. 

All of this is done using ESP Home and and a little library I had developed in C++ to make it as powerful as possible.

It currently offers four functions: VU meter, "n" band spectrograph, sound level meter (in decibels) and dominant frequency analyzer (shows the frequency with the most intensity).

Visit https://www.youtube.com/watch?v=dPXrfUtZW50 to see the first version (only VU meter and "n" band spectrograph). You can coment there and write me.

IN PROCESS...
