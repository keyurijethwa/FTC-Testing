package org.firstinspires.ftc.teamcode.pedropath_test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShootTest{

    private DcMotorEx shooterL;


    public ShootTest(HardwareMap hardwareMap){

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