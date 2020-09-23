# cgc
Very simple Command Ground Control 


### Commands
| cmd | options | example |
| --- | ------- | ------- |
| connect | [connection=serial:/dev/ttyACM0] [baudrate=57600] | connect <br> connect serial:/dev/ttyUSB0 <br> connect udp:127.0.0.1:14540 57600 |
| show | [filename=mission.txt] | show <br> show other-mission.txt |
| send | [filename=mission.txt] | send <br> send other-mission.txt |
| help | [cmd] | help <br> help connect <br> help cgc |
