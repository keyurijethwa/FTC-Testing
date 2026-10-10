package org.firstinspires.ftc.teamcode.TeleopTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterTest {

    private DcMotorEx shooterL;


    public ShooterTest(HardwareMap hardwareMap){

        shooterL = hardwareMap.get(DcMotorEx.class, "SH");

        shooterL.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void forwardsh(){
        shooterL.setVelocity(200);
    }
    public void stopsh(){
        shooterL.setPower(0);
    }
    public void reversesh(){
        shooterL.setVelocity(-200);
    }
}