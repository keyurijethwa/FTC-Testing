package org.firstinspires.ftc.teamcode.TeleopTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeTest {
    DcMotor i;
    public IntakeTest(HardwareMap hardwareMap){
        i=hardwareMap.get(DcMotor.class,"IB");
        i.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeIn(double power){
        i.setPower(power);
    }

    public void intakeStop(){
        i.setPower(0);
    }

    public void intakeOut(double power){
        i.setPower(-power);
    }
}
