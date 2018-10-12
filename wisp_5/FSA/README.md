# Gen2 UHF RFID Reader for WISP 5.1 sensor 

This is a Gen2 UHF RFID Reader. It is able to identify tags WISP 5.1 Tags with FM0 line coding and 160kHz tag data rate (BLF), and extract their EPC message. 
The value of Tari is 12.5us. It is also capable of reading data from their acceleromeer sensor.

The reader implements a typica FSA anti-collision protocol.
The reader sets a fixed frame size for the whole inventory round.

It has been implemented with an USRP N210 and was tested with the daughterboard SBX.  
Developed by Laura Arjona in the context of her PhD, University of Deusto, Department of Engineering.   

The project was initially based on the RFID Gen2 Reader developped by Nikos kargas https://github.com/nkargas/Gen2-UHF-RFID-Reader
 The reader borrows elements from the software developed by Nikos


## Installation

- install log4cpp (http://log4cpp.sourceforge.net/)
- install UHD driver + GNU Radio using 
- Install swig: $ sudo apt-get install swig
- In order to compile and build the project:
- cd Gen2-UHF-RFID-Reader/gr-rfid/  
	- mkdir build  
	- cd build/  
	- cmake ../ (logging should be enabled)  
	- sudo make install  
	- sudo ldconfig  

## Configuration

- Set USRP N210 address in apps/reader.py (default: 192.168.10.2)
- Set frequency in apps/reader.py (default: 915MHz)
- Set tx amplitude in apps/reader.py 
- Set rx gain in apps/reader.py 

## How to run

- Real time execution:  
cd Gen2-UHF-RFID-Reader/gr-rfid/apps/    
sudo GR_SCHEDULER=STS nice -n -20 python ./reader.py  915e6  

- Offline:  
    Change DEBUG variable in apps/reader.py to TRUE.
    The reader works with offline traces without using a USRP.  

    #### Output files 
    
    /misc/data/source  
    /misc/data/matched_filter  
    /misc/data/gate 
    /misc/data/decoder  
    /misc/data/reader
  
    
## Logging

- Configuration file : /home/username/.gnuradio/config.conf  
    Edit the above file and add the following lines  

    [LOG]  
    debug_file = /PathToLogFile/Filename  
    debug_level = info  
    
    Logging may cause latency issues if it is enabled during real time execution. However, it can be used with offline traces.
    
## Hardware:

  - 1x USRP N210
  - 1x SBX daughterboard  
  - 2x circular polarized antennas 
  - 1x Ethernet switch to conect the USRP with the laptop 
	

## Tested on:
  $lsb_release -a: Ubuntu 16.04.3 LTS
  $gnuradio-companion --version : GNU Radio Companion 3.7.9

## Contact:
  Laura Arjona  (e-mail:  laura.arjona@deusto.es ; arjonal@uw.edu)  



