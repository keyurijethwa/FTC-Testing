package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.test_robot;

import java.util.List;

@TeleOp(name = "Main")
public class Main extends LinearOpMode {
    private test_robot tr;
    private Intake_Balls ib;
    private servo_d s;
    private Limelight3A limelight3A;
    double drive_power;
    private static final int TARGET_TAG_ID = 30;

    // ==========================================
    // TARGET DISTANCE
    // ==========================================

    private static final double TARGET_DISTANCE_INCH = 30.0;

    // Allowed horizontal angle error

    // ==========================================
    // DISTANCE CALIBRATION
    // ==========================================

    // Your previous calibration
    private static final double DISTANCE_OFFSET_INCH = 5.0;
    double intake_power;
    private Servo clr;
    private boolean PRESSED=false;
    private ShootBalls sb;
    GoBildaPinpointDriver pinpoint;


    @Override
    public void runOpMode() throws InterruptedException {
        tr=new test_robot(hardwareMap);
        ib=new Intake_Balls(hardwareMap);
        s=new servo_d(hardwareMap);
        sb=new ShootBalls(hardwareMap);
        limelight3A=hardwareMap.get(Limelight3A.class,"limelight");
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

//        clr=hardwareMap.get(Servo.class,"CLED");

        drive_power=0.8;
        limelight3A.pipelineSwitch(0);

        limelight3A.start();

        // ==========================================
        // TELEMETRY BEFORE START
        // ==========================================

        telemetry.addLine("================================");
        telemetry.addLine(" APRILTAG CENTER + DISTANCE");
        telemetry.addLine("================================");

        telemetry.addData(
                "Target Tag",
                TARGET_TAG_ID
        );

        telemetry.addData(
                "Target Distance",
                "%.1f inches",
                TARGET_DISTANCE_INCH
        );

        telemetry.addData(
                "Distance Offset",
                "%.1f inches",
                DISTANCE_OFFSET_INCH
        );

        telemetry.update();
//        intake_power=0.2;
//
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//        pinpoint.resetPosAndIMU();
//
//        telemetry.addLine("Pinpoint Ready");
//        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            tr.driveRobot(
                    -gamepad1.left_stick_y * drive_power,
                    gamepad1.left_stick_x * drive_power,
                    -gamepad1.right_stick_x * drive_power
            );

            // Limelight

            LLResult result = limelight3A.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid Limelight result");
                telemetry.update();
                continue;
            }
            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            LLResultTypes.FiducialResult targetTag = null;

            // Find AprilTag ID 30
            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() == TARGET_TAG_ID) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag == null) {

                telemetry.addLine("AprilTag 30 not detected");
                telemetry.update();
                continue;
            }

            // Get target position
            Pose3D targetPose =
                    targetTag.getTargetPoseCameraSpace();

            if (targetPose == null) {
                telemetry.addLine("Target pose unavailable");
                telemetry.update();
                continue;
            }

            double x = targetPose.getPosition().x;
            double y = targetPose.getPosition().y;
            double z = targetPose.getPosition().z;

            // Convert X/Z position into horizontal angle
            double horizontalAngle =
                    Math.toDegrees(Math.atan2(x, z));

            // Convert Y/Z position into vertical angle
            double verticalAngle =
                    Math.toDegrees(Math.atan2(y, z));

            telemetry.addData(
                    "Horizontal Angle",
                    "%.2f°",
                    horizontalAngle
            );

            telemetry.addData(
                    "Vertical Angle",
                    "%.2f°",
                    verticalAngle
            );

            telemetry.addData(
                    "X",
                    "%.3f",
                    x
            );

            telemetry.addData(
                    "Y",
                    "%.3f",
                    y
            );

            telemetry.addData(
                    "Z",
                    "%.3f",
                    z
            );

//            if(gamepad1.left_bumper){
//                // You could use readings from April Tags here to give a new known position to the pinpoint
//                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//            }
//            if(gamepad1.left_trigger>0.2){
//
//                tr.turn90(90,0,Math.toDegrees(pinpoint.getHeading(AngleUnit.DEGREES)));
//
//            }
//            pinpoint.update();
            if(gamepad1.dpad_left){
                PRESSED=true;
            }
//            if (PRESSED){
//                if (horizontalAngle > 2.0) {
//                    tr.driveRight(0.6);
//                    telemetry.addLine("→ STRAFE RIGHT");
//
//                } else if (horizontalAngle < -2.0) {
//                    tr.driveLeft(0.6);
//                    telemetry.addLine("← STRAFE LEFT");
//
//                } else {
//                    tr.stopDrive();
//                    PRESSED=false;
//                    telemetry.addLine("✓ CENTERED - STOP");
//                }
//            }else {
//
//            }


            if(gamepad1.left_bumper){
                ib.in(0.7);
            }
            else {
                ib.stop1();
            }
            if(gamepad1.left_trigger>0.2){
                ib.out(0.5);
            }
            else {
                ib.stop1();
            }

            telemetry.addData(
                    "X (in)",
                    "%.2f",
                    pinpoint.getPosX(DistanceUnit.INCH));

            telemetry.addData(
                    "Y (in)",
                    "%.2f",
                    pinpoint.getPosY(DistanceUnit.INCH));

            telemetry.addData(
                    "Heading (deg)",
                    "%.2f",
                    pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.addData(
                    "Frequency",
                    "%.0f Hz",
                    pinpoint.getFrequency());

            telemetry.update();

            if(gamepad1.right_bumper){
                sb.forward(0.5);
            }
            else {
                sb.stop();
            }
            if(gamepad1.right_trigger>0.2){
                sb.reverse(0.5);
            }
            else {
                sb.stop();
            }

//            if(gamepad1.right_bumper){
//                ib.in(0.5);
//            }
//            else {
//                ib.stop1();
//            }
//            if(gamepad1.right_trigger>0.2){
//                ib.out(0.5);
//            }
//            else {
//                ib.stop1();
//            }
//            if(gamepad1.right_bumper ){
//                ib.in(intake_power);
//                if(ib.invelo()>150) {
//                    clr.setPosition(0.5);
//                }
//
//            } else if  (gamepad1.right_trigger>0.2){
//                ib.out(intake_power);
//                if(ib.invelo()>150) {
//                    clr.setPosition(0.722);
//                }
//            }else {
//                ib.stop1();
//                if (ib.invelo()==0){
//                    clr.setPosition(0);
//                }
//            }

            if(gamepad1.b){
                s.setPB();
                if(s.getP()==0){
                    clr.setPosition(0.333);
                }else {
                    clr.setPosition(0);
                }

            }
            if(gamepad1.a){
                s.setPA();
                if(s.getP()==1){
                    clr.setPosition(0.555);
                }else{
                    clr.setPosition(0);
                }
            }
            if(gamepad1.y){
                s.setPY();
            }
            if(gamepad1.x){
                s.setPX();
            }

        }

    }
}
