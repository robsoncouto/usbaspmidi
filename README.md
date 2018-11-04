# usbaspmidi
Code for making the usbasp behave as a usb midi adapter, both midi in and out.

This is based on code from morecat_lab
http://morecatlab.akiba.coocan.jp/morecat_lab/MOCO-e.html

This is meant to be a gimmick. I just needed a converter to test a keyboard, so I hacked away morecat_labs code to work with the usbasp in some minutes. 

You will need additional circuitry to condition the midi signal. 
I use a 6n137 opt-isolator for midi in.

A write-up about this project is available at https://dragaosemchama.com/en/2018/10/usbasp-as-a-midi-adapter/
