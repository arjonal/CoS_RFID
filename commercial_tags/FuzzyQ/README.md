
Implementation of FuzzyQ anti-collision protocol [1] in a Softwared Defined Radio RFID reader [2,3], and tested with Alien RIFD tags.


	References:
------------------------
[1] L. Arjona, H. Landaluce, A. Perallos and E. Onieva, "Fast fuzzy anti-collision protocol for the RFID standard EPC Gen-2," in Electronics Letters, vol. 52, no. 8, pp. 663-665, 4 14 2016.

[2] Laura Arjona, Hugo Landaluce, Asier Perallos, and Aaron Parks. 
"Survey and Analysis of RFID DFSA Anti-Collision Protocols and their Physical Implementation Capabilities". 
RFID Technology: Design Principles, Applications and Controversies. Nova Publishers. Book chapter, accepted for publication.

[3] N. Kargas, F. Mavromatis and A. Bletsas, "Fully-Coherent Reader with Commodity SDR for Gen2 FM0 and Computational RFID", IEEE Wireless Communications Letters (WCL), Vol. 4, No. 6, pp. 617-620, Dec. 2015. 

[4] EPC Radio-Frequency Identity Protocols Generation-2 UHF RIFD. Specification for RFID Air Interface,” Version 2.0.1, 2015. Available on-line: http://www.gs1.org/epcrfid/epc-rfid-uhf-air-interface-protocol/2-0-1

	Default Parameters:
------------------------
- Initial frame size: 4 slots
- E_th: 0.03 (see [2])


	Hardware
------------------------
  - 1 USRP FLEX900  
  - 1 SBX daughterboard 
  - 2 circular polarized patch antennas  
  - 1 Gigabit etherent Switch 
  - 1 personal computer
  - ALN-9640 RFID tags


  	Software installation
 ------------------------
  - install log4cpp (http://log4cpp.sourceforge.net/)
  - install UHD driver + GNU Radio using wget http://www.sbrac.org/files/build-gnuradio && chmod a+x ./build-gnuradio && ./build-  gnuradio
  - cd FuzzyQ/gr-rfid/
  - mkdir build
  - cd build/
  - cmake ../ (logging should be enabled)
  - sudo make install
  - sudo ldconfig
  
 	Real Time execution
 ------------------------
  - cd FuzzyQ/gr-rfid/apps/
  - sudo GR_SCHEDULER=STS nice -n -20 python ./reader.py myfreq
  (myfreq corresponds to the reader frequency. Valid range: 902-920MHz)
  - output signals are saved in SlotCounter/gr-rfid/misc/data. They can be plotted with Matlab and Octave using the files in gr-rfid/misc/code
  
	Contact:
 ------------------------
  Laura Arjona (e-mail: arjonal@uw.edu   laura.arjona@deusto.es)  

