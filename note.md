##### 1.GCS与mavros共用

https://github.com/mavlink/mavros/issues/624

##### 2.Difficulty to OverrideRCIn for APM

https://github.com/mavlink/mavros/issues/629



##### 3.多机mavros，终端

roslaunch iq_sim apm.launch fcu_url:=/dev/ttyUSB0:57600 mavros_ns:=/drone1 tgt_system:=1

roslaunch iq_sim apm.launch fcu_url:=/dev/ttyUSB1:57600 mavros_ns:=/drone2 tgt_system:=2

