package org.firstinspires.ftc.teamcode.util;
/*

I/RobotCore: ========= Device Information ===================================================
    Type                                               Name                           Connection
    REV Touch Sensor                                   touchsensor                    USB (embedded); module 173; digital channel 0
I/RobotCore: REV Expansion Hub IMU                              imu                            USB (embedded); module 173; bus 0; addr7=0x00
V/Robocol: sending CMD_PLAY_SOUND(44), attempt: 0
I/RobotCore: Ingenic Semiconductor CO., LTD. HD Web Camera      Webcam 1                       USB (Ucamera001)
I/RobotCore: Servo                                              servo1                         USB (embedded); module 173; port 0
I/RobotCore: Rev Color Sensor v3                                color                          USB (embedded); module 173; bus 1; addr7=0x52
    Expansion Hub DC Motor Controller                  Control Hub                    USB (embedded); module 173
I/RobotCore: Expansion Hub Servo Controller                     Control Hub                    USB (embedded); module 173
    Expansion Hub Voltage Sensor                       Control Hub                    USB (embedded); module 173
I/RobotCore: Expansion Hub Analog Input Controller              Control Hub                    USB (embedded); module 173
    Expansion Hub Digital Channel Controller           Control Hub                    USB (embedded); module 173
I/RobotCore: Expansion Hub (HW: 20, Maj: 1, Min: 8, Eng: 2)     Control Hub                    USB (embedded); module 173
I/RobotCore: MR Optical Distance Sensor                         front                          USB (embedded); module 173; analog port 0
    Motor                                              left_front                     USB (embedded); module 173; port 0
    REV Robotics USB Expansion Hub Portal              Control Hub Portal             USB (embedded)
V/LynxUsb: 0x0942574a on 0x01b5ef9f: releasing delegate to [(embedded)]
I/FtcEventLoop: ======= INIT FINISH =======
 */
public class Names_ProgrammingBoard {
    public static final String imu = "imu";

    public static final String sensor_Color = "color";
    public static final String sensor_Rings = "touchsensor";

    public static final String servo_GrabberClaw = "servo1";
    public static final String servo_DeliverySlide = "servo1";
    public static final String servo_BlinkinLED =  "servo1";

    public static final String servo_InchwormLifter = "servo1";

    public static final String servo_Magazine = "servo1";
    public static final String servo_Trigger = "servo1";

    public static final String motor_LeftFront = "left_front";
    public static final String motor_RightFront = "left_front";
    public static final String motor_LeftRear = "left_front";
    public static final String motor_RightRear = "left_front";
    public static final String motor_Intake = "left_front";
    public static final String motor_Shooter1 = "left_front";
    public static final String motor_Shooter2 = "left_front";

    public static final String range_Front = "front";
    public static final String range_Rear = "front";
    public static final String range_Left = "front";
    public static final String range_Right = "front";

    public static final String camera_Default = "Webcam 1";
}
