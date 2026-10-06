package org.firstinspires.ftc.teamcode.pedropath_test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class BioBuzzShooterTest {

    DcMotorEx shooter;
    public BioBuzzShooterTest(HardwareMap hardwareMap){
        shooter=hardwareMap.get(DcMotorEx.class,"SH");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void forward(double power){
        shooter.setVelocity(200);
    }
    public void stop(){
        shooter.setPower(0);
    }
    public void reverse(double power){
        shooter.setVelocity(-200);
    }

}
