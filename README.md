# rcs
Welcome to the  Rocket Control System!
This code was made for Auburn Rocketry Team and is a heavy rework of a drone control system.
		rc_pilot was the original lighweight flight code for controlling a multirotor developed by 
		Dr. Stawson, Dr. Gaskell and others - https://github.com/StrawsonDesign/rc_pilot
		
It is critical to learn and understand the flightcode before using it. Make sure you know
what you are doing and don't hold me liable for any equipment damage and follow the law!

THIS CODE IS NOT FOR PUBLIC USE. USE AT YOUR OWN RISK. THIS CAN AND WILL GET YOU IN JAIL IF MISUSED.
			CHECK WITH FAA AND LOCAL REGULATIONS

#Installation:
Made to run on Beaglebone Blue linux board with the debian image from the official website:
https://beagleboard.org/getting-started			- getting started page
https://beagleboard.org/latest-images 			- install image from here
http://strawsondesign.com/docs/librobotcontrol/ - robot control library (start with manual)
Depends on libjson-c-dev & libjson-c3 sudo apt install libjson-c-dev libjson-c3
  	and also libroboticscape >v0.4.0 (which is installed by default)
To install, you need to copy the entire RC_Pilot directory to BBB and cd into /RC_Pilot/
	compile the code by typing "make" (make sure you are doing is as root, type "sudo su")
	you can also check robot control library functionality by running rc_test_* 
		scripts (type rc_test_ and hit "Tab" key to see all options)



For questions contact me, Yevhenii Kovryzhenko - yzk0058@auburn.edu
