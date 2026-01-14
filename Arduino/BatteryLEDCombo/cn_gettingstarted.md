# 快速使用指南

## 刷机

1. 使用Arduino IDE 或IM Assistant对开发板刷机。
2. 使用IM Assistant为开发板设置CAN编码。

## 接线

1. RC522读卡模块应使用8p杜邦线直接和RFID接口对插
2. LED灯带Di连接开发板IO16接口，供电根据灯带本身电压，从PDP或VRM取电12V。从VRM取电5V。在使用读卡器的情况下不建议使用开发板5V输出避免电磁干扰。
3. 灯带的电源应由继电器控制。应将灯带电源线经过开发板继电器NO和COM控制，确保在继电器吸合的情况下灯带供电。此设计为了避免灯带在机器上电时频闪。
4. CAN接入机器人CAN网络。请注意检查终端电阻开关的位置，根据实际情况设置。
5. 开发板电源直接接入PDP。保险丝应使用5A以下。尽量不要将开发板和CANCoder，Pigeon连接在一个保险丝下，建议使用单独保险丝避免开发板抽风影响机器控制。
6. 建议打印外壳，保护好未使用GPIO，避免铁屑进入造成短路。
7. 将USB线或接口位置留好便于后续刷机。

## 快速测试

此步骤不需要在roboRIO上部署驱动代码，但需要roboRIO连接开发板且roboRIO有任意机器代码。
1. 上电后灯带将进入自检模式，继电器会自动吸合，开发板继电器黄灯亮。
2. 观察灯带颜色。灯带的前255个像素会渐入白色，然后1Hz红绿蓝三色。之后会1Hz闪烁前3个像素白色。
3. 按动IO0按钮，可以切换灯带图案。
4. 按动SW按钮可以开关灯带，继电器动作。开关为本地开关，roboRIO无法控制。此为调试关灯功能。如通过按钮关灯后需要重新开灯，需要使用按钮开灯。否则唯一从roboRIO远程开灯方法为使用API重启开发板。不建议这样操作。
5. 观察开发板上RGB三色灯。如果roboRIO已经启动完成，三色灯应该闪黄灯。
6. 初始化一个标签，具体操作请看英文文档。
7. 将标签放在RC522读卡器上，此时三色灯应该快速闪烁蓝灯并切换为黄灯或绿灯，代表写卡成功。此时使用Driver Station将机器人Enable 五秒后Disable。三色灯会再次快速闪烁蓝灯并切换为黄灯或绿灯。
8. 使用手机APP读卡，应有一条机器人使用记录

## 部署驱动

可以直接部署演示程序 [example project](https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/roboRIO/batteryReaderNewLEDCombo)，完整测试开发板。此章节为讲解如何集成roboRIO驱动
1. 将演示程序中·batteryCAN.java·和·addressableLEDCAN.java·拷贝到subsystems
2. 导入对应class
```java
import frc.robot.subsystems.addressableLEDCAN;
import frc.robot.subsystems.batteryCAN;
```
3. 在初始化代码中初始化对应设备。两个class使用的ID一样
```java
battery = new batteryCAN(ID); // your ESP32 device number
leds = new addressableLEDCAN(ID); // Match LED ESP32 device number
```
4. 获取电池数据并记录在Akit中。
```java
Battery battery = BatteryManager.getActiveBattery(); // example source

// --------------------
// 序列号
// --------------------
String serial = battery.getSerial();

// --------------------
// 循环数据
// --------------------
int cycleCount = battery.getCycleCount();
String firstUseDateTime = battery.getFirstUseDateTime();

// --------------------
// 状态标记
// --------------------
int note = battery.getNote();

String noteLabel = switch (note) {
    case 0 -> "Normal";
    case 1 -> "Practice Only";
    case 2 -> "Scrap";
    case 3 -> "Other";
    default -> "Unknown";
};
```
5. 操作灯带模式
```java
leds.setTotalPixel(totalPixels);
leds.sendGeneralCommand(mode, r, g, b, brightness, onOff, param0, param1);
```
6. 获取硬件状态；所有硬件状态有关API在batteryCAN Class中。
```java
battery.getIsESPOnline();//ESP32在线状态
battery.isReaderDetected();//读卡器连接状态
battery.getPDType();//PDP/PDH侦测状态，涉及能源监测调试
battery.getESPState();//ESP32程序运行状态，用于固件调试
battery.getWriteCount()//写卡次数计数
battery.getWriteFailCount()//写卡失败计数
battery.requestReboot();//使用此API让ESP32重启
battery.setOverrideState(overrideState);//调试用API，用于覆盖开发板程序状态。仅为调试用！
```
8. 电量使用监测。如果连接了支持CAN的PDP或PDH，开发板将自动计算使用电量。如果使用了不支持CAN的配电板，则可以自行计算电量后通过API写入开发板。
```java
battery.setEnergyKJ(energyKJ);//设置能量
battery.setUseRIOEnergy(isUsingRIOEnergy);//如果isUsingRIOEnergy为True则使用用户发送的能量。如果为True且energyKJ则会写入0
battery.setEnergyKJAndSend(energyKJ);//设置能量并将isUsingRIOEnergy设置为True
```
9. 如果LED灯带需要使用非内置灯效，需通过`leds.sendGeneralCommand`将模式设置为255，之后可以使用下列API直接对灯带进行像素级设置。slot为0-7通道。最多支持8个通道同时写入，加快效率。需要注意此API将占用大量CAN带宽十分不推荐使用！
```java
leds.sendPixelWrite(index, pr, pg, pb, 0, pbrig, slot);
```