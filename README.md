一、作用：
1.向4g模组发送AT指令，拨号上网，给wwan0口获取到ip。
2.每隔10min检查联网状态（ping www.baidu.com）


二、编译：
make clean 
./make.sh

三、使用：
1.将编译出的文件modem4g放到设备上，执行
root@ctfo:~# ./modem4g 

check_Dial_status Failed OR Dial status is down
udhcpc (v1.24.1) started
Sending discover...
Sending select for 10.198.155.248...
Lease of 10.198.155.248 obtained, lease time 7200
/etc/udhcpc.d/50default: Adding DNS 202.99.69.69
/etc/udhcpc.d/50default: Adding DNS 202.99.69.68
PING www.baidu.com (110.242.68.4): 56 data bytes
64 bytes from 110.242.68.4: seq=0 ttl=53 time=23.993 ms
64 bytes from 110.242.68.4: seq=1 ttl=53 time=24.969 ms
64 bytes from 110.242.68.4: seq=2 ttl=53 time=22.768 ms
64 bytes from 110.242.68.4: seq=3 ttl=53 time=24.436 ms

2.如需要输出详细的log信息
root@ctfo:~# ./modem4g -d 4

log level:4 (0:off 1:error 2:warn 3:info 4:debug)
Opening tty device /dev/ttyUSB2
AT> ATE0Q0V1
AT< OK
AT> AT+CMEE=1
AT< OK
getSIMStatus
AT> AT+CPIN?
AT< +CPIN: READY
AT< OK
...略

四、待解决问题：
1.需要放入开机启动脚本吗？
2.不能支持SIM卡热插拔#run modem SIM7600ce-t
