package org.firstinspires.ftc.teamcode.TeleopTest;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGBTest {
    private static final double RGB_RED=0.277;

    private static final double RGB_ORANGE=0.333;

    private static final double RGB_YELLOW=0.388;
    private static final double RGB_GREEN=0.500;
    private static final double RGB_BLUE=0.611;

    private Servo rgb;
    public RGBTest(HardwareMap hardwareMap){
        rgb=hardwareMap.get(Servo.class,"rgb");

    }

    public void setRed(){
        rgb.setPosition(RGB_RED);
    }
    public void setOrange(){
        rgb.setPosition(RGB_ORANGE);
    }
    public void setYellow(){
        rgb.setPosition(RGB_YELLOW);
    }
    public void setGreen(){
        rgb.setPosition(RGB_GREEN);
    }
    public void setBlue(){
        rgb.setPosition(RGB_BLUE);
    }
    public void setOff(){
        rgb.setPosition(0);
    }

}
