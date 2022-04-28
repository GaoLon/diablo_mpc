# DIABLO Onboard SDK二次开发工具包使用说明
## 简介
DIABLO OSDK是DIABLO双足轮式机器人系统配套的程序包，运行在机器人的机载小型电脑上。该程序包使用户可以在机器人上运行自定义程序，以验证用户自己开发的程序算法，实现用户所需的功能和应用，例如新型控制算法实践，自主智能导航和规划等功能。
DIABLO OSDK主要具有以下特点
* 兼容性优秀：软件设计上对环境依赖少，兼容多种硬件和用户使用的第三方程序包。
* 控制界面简单：用户可以通过我们提供的API，简单直观的向和机器人交互，收发指令。
* 安全性好：系统配套了多种安全保障逻辑，可以在用户开发过程中各种特殊工况下，尽可能保证安全。  

本开发指南请结合配套源码及其文档查看

## 模块介绍
### 机器人接口层
```class Vehicle```:
用户程序和机器人交互的主要界面, 包含OSDK其他模块的交互接口

### 通信协议层
```class Telemetry```:负责将用户给机器人发送和接收的指令编码，解码。提供用户和机器人的通信协议
### 硬件应用层
```class HAL```：负责操控机载电脑的硬件设备，和机器人底层控制板和关节电机通信
### 常用软件包接口层
```class Bridge```:抽象基类提供与常用第三方程序如ROS的自动和标准化数据交互模块

### 自定义回调
```class CallBack_Handler```:单例class，控制所有的回调进程
```class Callback_Manager```:管理单个回调进程
```class Broadcaster```:事件发出类，传递参数给监听类
```class Listener```:事件监听类，接收参数并调用用户定义的回调函数

## 功能介绍
* Telemetry: 
```
class Telemetry;
```
处理机器人向用户自动推送的各类数据，用户可以设置机器人自动推送数据的类型和更新频率。
* Movement Ctrl：
```
class Virtual_RC;
class Movement_Ctrl;
```
用户给机器人自带的运动控制器发送运动指令的方式，用户可以直接发送运动速度等指令，或使用遥控器虚拟程序发送虚拟的遥控器通道数据。
### 更改ROS_IP
若需要依赖```ros```，则需要更改机载树莓派```ROS_IP```  
terminal内输入

```
sudo nano .bashrc
```
将最后一行```export ROS_IP=```改为机载树莓派IP。  
修改之后登出并重新连接树莓派（仅ssh连接树莓派需要）

### 虚拟遥控器案例运行
1. 所需文件  
    通过说明书下载文件到控制端，文件内```./script```文件夹为虚拟遥控器控制脚本，依赖```rospy, pygame, pyautogui```运行

2. 树莓派 ONLY  
    如通过机载树莓派运行控制脚本，遇到```pygame，pyautogui```安装出错等问题，则需在PC上运行脚本，通过```ros```多机之间通信以进行虚拟遥控器控制，详见后文。

3. 开启SDK案例  
    在SDK运行端(机载树莓派/PC)进入```./catkin_ws```文件夹，terminal内输入:
    
    ```
    catkin_make
    ```
    编译完成后，再输入:
    ```
    roscore &
    ```
    启动ros。  
    最后输入:
    ```
    rosrun diablo_sdk virtual_rc_example
    ```
    运行SDK案例  
    在控制端，运行```./script```文件夹内```./script/teleop.py```脚本  
    (若是多机通信，则启动```ros```后需设置完成```ros```远程通信，再运行SDK案例和控制脚本)  
    
4. 

5. 连接说明  
    为保证安全，请不要在机器人站立的时候进行虚拟遥控器连接(左扳机键拨至最下)，连接之前请设置遥控器为急停模式(右扳机键拨至最下)  
    确认安全后，可开始连接虚拟遥控器。开始虚拟遥控器控制前，需解除遥控器急停模式(右扳机键拨离最下档位)  
    当terminal显示```SDK Virtual RC In Control```后，表示虚拟遥控器成功连接。此时机器人不再接收遥控器指令，直至退出虚拟遥控器  

6. 操控说明  
    运行teleop.py脚本且成功连接后，在弹出窗口内通过键盘控制  
    ```w```: 控制机器人向前移动  
    ```s```: 控制机器人向右移动  
    ```a```: 控制机器人向左转向  
    ```d```: 控制机器人向右转向  
    ```z```: 切换机器人至站立形态  
    ```x```: 切换机器人至匍匐形态  
    ```q```: 控制机器人向左倾斜(站立模式)  
    ```e```: 控制机器人向右倾斜(站立模式)  
    ```c```: 切换机器人至运动模式(更快的移动速度，站立形态下允许跳跃)  
    ```v```: 切换机器人至展示模式(平稳的移动速度，禁止跳跃)  
    ```r```: 运动模式站立形态下控制机器人跳跃  
    ```esc```: 退出虚拟遥控器控制，自动断开并切换回遥控器控制(为安全起见，请不要在虚拟遥控器操控时切换遥控器模式，断开虚拟遥控器前请切换机器人为匍匐形态且遥控器设置机器人为匍匐形态)  
    (PS. 切换机器人为站立模式时，若负载过大会触发电机保护程序，将阻止机器人站起，若严重过载将触发电池保护程序，电池将断开供电。为安全起见，请勿超过最大负载切换机器人形态。)
### ROS远程通信
1. 在树莓派和PC安装```chrony```和```ssh server```
    ```
    sudo apt-get install chrony
    sudo apt-get install openssh-server
    ```
    
2. 确认树莓派和PC上ssh服务器是否启动
    ```
    ps -e|grep ssh
    ```
    若只有```ssh-agent```，则服务器未启动，需通过
    ```
    /etc/init.d/ssh start
    ```
    启动。  
    若出现```sshd```，则ssh服务器成功启动  
    
3. 树莓派，PC互ping确认传输正常
    树莓派
    
    ```
    ping PC_ip
    ```
    PC
    ```
    ping Raspi_ip
    ```
    
4. 树莓派配置ROS_MASTER_URI
    树莓派启动```ros(roscore &)```之后
    
    ```
    export ROS_MASTER_URI=http://<Raspi_ip>:11311
    ```
    
5. PC配置ROS_MASTER_URI
    PC端请勿开启```ros```
    
    ```
    export ROS_MASTER_URI=http://<Raspi_ip>:11311
    ```
    (ip均不带'<>'符号')
    操作完成后即可进行```ros```远程通信  
    不推荐使用虚拟机进行远程通信，若为虚拟机安装Ubuntu，则需将虚拟机网络适配器设置为桥接模式。  
    由于虚拟机导致的一切问题，请自行解决，恕不负责。
## 开发指南
1. 系统环境依赖 ```cmake, boost```

2. 第三方软件 ```wiringPi(树莓派), serial(PC)```

3. serial库安装(NON-Raspberry Pi ONLY)
    1. Terminal进入```serial```文件夹 (./serial)
    2. 修改安装目录  
    打开```Makefile```文件(./Makefile),如果第三行```CMAKE_FLAGS := -DCMAKE_INSTALL_PREFIX=/tmp/usr/local```  
    删掉```/tmp, (CMAKE_FLAGS := -DCMAKE_INSTALL_PREFIX=/usr/local)```
    3. Build  
        Terminal 输入
        ```
        make
        ```
    4. Build and Run test  
        Terminal 输入
        
        ```
        make test
        ```
    5. 安装  
        第四步无异常后，Terminal输入        
        ```
        sudo make install
        ```
4. 主要使用步骤：
    1. 初始化机载电脑的硬件设备  
        树莓派
        ```c++
        DIABLO::OSDK::HAL_Pi Hal;//Initialize HAL driver
        Hal.init();
        ```
        PC
        ```c++
        DIABLO::OSDK::HAL_Serial Hal;//Initialize HAL driver
        Hal.initSerial("/dev/ttyUSB0", 460800);
        ```
    2. 初始化SDK，设置用户所需的反馈数据
        ```c++
        DIABLO::OSDK::Vehicle vehicle(&Hal);    //Initialize Onboard SDK
        if(vehicle.init()) return -1;
        vehicle.telemetry->configDataFreq(DIABLO::OSDK::TOPIC_QUATERNION, OSDK_PUSH_DATA_100Hz); // Set receiving topic
        vehicle.telemetry->configUpdate();
        ```
    3. 接管机器人的运动控制，开始发送指令
        ```c++
        if(!pVirtualRC->in_control()) //check control status
        {
            pVirtualRC->obtain_control();
            return;
        }
        ```
5. 切换开发平台  
    若需要切换开发平台，在```./diablo-onboard-sdk/CMakeLists.txt```中更改```WITH_SERIAL (PC)```和```WITH_PI (树莓派)```的值  
    ```1```为选中平台，二者只可设置其一为```1```。PC端则需安装serial库，详见第三条。
    
6. 不依赖```ros```  
    ```./example```文件夹中所有案例均依赖```ros```，若脱离```ros```开发，打开SDK根目录下```CMakeLists.txt```文件，将其中```WITH_ROS```设置为```0```即可。  
    不依赖```ros```编译，则需terminal进入SDK根目录下的```build```文件夹（若文件夹不存在，则需新建```mkdir build```）  
    进入后terminal内输入
    
    ```
    cmake ..
    make
    sudo make install
    ```
    以安装到本地，可执行文件会生成在build文件夹内，若只需要可执行文件，则不需输入```sudo make install```
## 其他函数
1. 自动标定  
	```Telemetry::Calibration```函数，将使机器人自动标定调整参数。开始标定前，请确保电池电量充足且机器人处于站立模式，自动标定时请留出足够空间，标定过程中请勿触碰机身，以防失控伤人以及参数错误。标定完成后需重启机器人。  
	出现以下状况，请调用此函数重新标定机器人  
	1. 站立姿态无法保持平衡  
	2. 站立姿态未推动遥控器，机器人自行前后移动(请先排除遥感漂移)
	若自动标定无法解决，且排除电机过热/过载因素，请联系售后。
2. 自动更新单片机固件  
	```diablo-iap-host```文件夹内为下位机自动更新固件模块，运行```/diablo-iap-host/build```内的```./diablo-iap```可执行文件即可自动更新下位机固件。更新固件时单片机LED将为白色闪烁，更新成功后为绿色闪烁。更新成功后请重启机器人。



