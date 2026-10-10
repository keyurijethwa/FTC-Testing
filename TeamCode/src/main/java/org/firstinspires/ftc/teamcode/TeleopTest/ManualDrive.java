package org.firstinspires.ftc.teamcode.TeleopTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ManualDrive {
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;
    public ManualDrive(HardwareMap hardwareMap){

        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        br = hardwareMap.get(DcMotor.class, "BR");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveRobot(double drive,double strafe,double turn){

        double FL = drive + strafe + turn;
        double FR = drive - strafe - turn;
        double BL = drive - strafe + turn;
        double BR = drive + strafe - turn;


        fl.setPower(FL);
        fr.setPower(FR);
        bl.setPower(BL);
        br.setPower(BR);

    }
    public void drive(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

    }

    public void driveLeft(double power){
        // STRAFE LEFT
        fl.setPower(-power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(-power);
    }
    public void driveRight(double power){
        // STRAFE RIGHT
        fl.setPower(power);
        bl.setPower(-power);
        fr.setPower(-power);
        br.setPower(power);
    }
}
