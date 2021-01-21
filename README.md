# CoS_RFID
Configurable SDR-RFID reader to identify WISP-5 and commercial tags. 
This reader can also read data from the wisp accelerometer sensor
Custom anti-collision protocols implementation.

----------------------------------
        Commercial Tags
----------------------------------
Tested with ALN-9640 RFID tags

Reader Configuration
 - FM0 line coding 
 - 40kHz Backscatter Link Frequency (BLF)
 
Hardware:
  - 1 USRP N210
  - 1 SBX daughterboard 
  - 2 circular polarized patch antennas  
  - 1 Gigabit ethernet Switch 
  - 1 personal computer
  - ALN-9640 RFID tags

Protocols Implemented
  - Slot Counter
  - FuzzyQ
  - DFSA-ideal
  
Contact:
Laura Arjona: arjonal@uw.edu, laura.arjona@deusto.es
SDR-RFID reader adapted from https://github.com/nikosl21/Gen2-UHF-RFID-Reader.
A special thanks to Nikos kargas for his discussions.


----------------------------------
        WISP Tags
----------------------------------
