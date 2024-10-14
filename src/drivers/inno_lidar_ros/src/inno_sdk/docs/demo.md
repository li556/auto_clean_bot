# demo
## A. What is demo
```shell
Demo is used to teach you how to quickly form an application to get point_cloud.
```

## B. Introduction of parameter
```shell
# you can use ./demo -h or ./demo to get all information
--lidar-ip 172.168.1.10                                       # lidar IP
--lidar-udp-port 8010                                         # lidar udp port
--use-pcap                                                    # get data from pcap file, not from lidar
--input-filename input.pcap                                   # the name of pcap file
```

## C. How to use demo
```shell
# record frames from live LIDAR via UDP.
./demo --lidar-ip 172.168.1.10 --lidar-udp-port 8010

# record frames from pcap file.
./demo --input-filename input.pcap --use-pcap
```

## D. Location
If you want to get detial of demo, please press [here](../apps/example/demo.cpp)