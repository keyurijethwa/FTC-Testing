package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.ShooterSubsystem1;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.test_robot;

import java.util.List;

@Autonomous(name = "BioBuzzAutoPath3", group = "Autonomous")
public class BioBuzzAutoPath3 extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.6796, 133.6867, 90);
    private final Pose path1 = poseFactory.of(58.6796, 105.6867, 90);
    private final Pose point2 = poseFactory.of(46.5765, 133.6867, 90);
    private final Pose point3 = poseFactory.of(58.5031, 105.6867, 90);
    private final Pose point4Start = poseFactory.of(58.5031, 105.6867, 180);
    private final Pose point4 = poseFactory.of(12.6459, 47.7531, 180);
    private final Pose point5Start = poseFactory.of(12.6459, 47.7531, 270);
    private final Pose point5 = poseFactory.of(54.6796, 38.8173, 270);
    private final Pose point6Start = poseFactory.of(59.6796, 38.8173, 180);
    private final Pose point6 = poseFactory.of(12.5235, 115.5194, 180);

    private ShooterSubsystem1 s;
    private double horizontalAngle;
    double verticalAngle;
    private test_robot tr;
    private Intake_Balls ib;
    private Limelight3A limelight;
    private double TARGET_TAG_ID=30;
    private Path lastPath = null;
    private boolean actionTriggered = false;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                        follow(follower, path1()),
                        waitMs(2000),
                        alignWithLimelight(),
                        follow(follower,path2()),
                        waitMs(2000),
                        follow(follower,path3()),
                        waitMs(2000),
                        follow(follower,path4()),
                        waitMs(2000),
                        follow(follower,path5()),
                        waitMs(2000),
                        follow(follower,path6())
                );
    }
    public Command alignWithLimelight() {
        return Command.build()
                .setStart(() -> {
                    horizontalAngle = 999;
                })
                .setExecute(() -> {
                    LLResult result = limelight.getLatestResult();

                    if (result == null || !result.isValid()) {
                        tr.stopDrive();
                        return;
                    }

                    List<LLResultTypes.FiducialResult> tags =
                            result.getFiducialResults();

                    LLResultTypes.FiducialResult targetTag = null;

                    for (LLResultTypes.FiducialResult tag : tags) {
                        if (tag.getFiducialId() == (int) TARGET_TAG_ID) {
                            targetTag = tag;
                            break;
                        }
                    }

                    if (targetTag == null) {
                        tr.stopDrive();
                        return;
                    }

                    Pose3D targetPose = targetTag.getTargetPoseCameraSpace();

                    if (targetPose == null) {
                        tr.stopDrive();
                        return;
                    }

                    double x = targetPose.getPosition().x;
                    double z = targetPose.getPosition().z;

                    horizontalAngle = Math.toDegrees(Math.atan2(x, z));

                    if (horizontalAngle > 2.0) {
                        tr.driveRight(0.6);
                    } else if (horizontalAngle < -2.0) {
                        tr.driveLeft(0.6);
                    } else {
                        tr.stopDrive();
                    }
                })
                .setDone(() -> Math.abs(horizontalAngle) <= 2.0);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
//        s=new ShooterSubsystem1(hardwareMap);
//        limelight=hardwareMap.get(Limelight3A.class,"limelight");
//        tr=new test_robot(hardwareMap);
//        ib=new Intake_Balls(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
//            LLResult result = limelight.getLatestResult();
//
//            if (result == null || !result.isValid()) {
//                telemetry.addLine("No valid Limelight result");
//                telemetry.update();
//                continue;
//            }
//
//            List<LLResultTypes.FiducialResult> tags =
//                    result.getFiducialResults();
//
//            LLResultTypes.FiducialResult targetTag = null;
//
//            // Find AprilTag ID 30
//            for (LLResultTypes.FiducialResult tag : tags) {
//
//                if (tag.getFiducialId() == TARGET_TAG_ID) {
//                    targetTag = tag;
//                    break;
//                }
//            }
//
//            if (targetTag == null) {
//
//                telemetry.addLine("AprilTag 30 not detected");
//                telemetry.update();
//                continue;
//            }
//
//            // Get target position
//            Pose3D targetPose =
//                    targetTag.getTargetPoseCameraSpace();
//
//            if (targetPose == null) {
//                telemetry.addLine("Target pose unavailable");
//                telemetry.update();
//                continue;
//            }
//
//            double x = targetPose.getPosition().x;
//            double y = targetPose.getPosition().y;
//            double z = targetPose.getPosition().z;
//
//            // Convert X/Z position into horizontal angle
//            horizontalAngle =
//                    Math.toDegrees(Math.atan2(x, z));
//
//            // Convert Y/Z position into vertical angle
//            verticalAngle =
//                    Math.toDegrees(Math.atan2(y, z));
//
//            telemetry.addData(
//                    "Horizontal Angle",
//                    "%.2f°",
//                    horizontalAngle
//            );
//
//            telemetry.addData(
//                    "Vertical Angle",
//                    "%.2f°",
//                    verticalAngle
//            );
//
//            telemetry.addData(
//                    "X",
//                    "%.3f",
//                    x
//            );
//
//            telemetry.addData(
//                    "Y",
//                    "%.3f",
//                    y
//            );
//
//            telemetry.addData(
//                    "Z",
//                    "%.3f",
//                    z
//            );
//
//            telemetry.addData("x", follower.pose().x());
//            telemetry.addData("y", follower.pose().y());
//            telemetry.addData("heading", follower.pose().heading());
//
//            if (follower.currentPath() != null) {
//                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
//                telemetry.addData("Path number", follower.pathIndex());
//            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return line(point6Start, point6).linear(point6Start, point6);
    }
    public void getPosition(){
        if (horizontalAngle > 2.0) {
            tr.driveRight(0.6);
            telemetry.addLine("→ STRAFE RIGHT");

        } else if (horizontalAngle < -2.0) {
            tr.driveLeft(0.6);
            telemetry.addLine("← STRAFE LEFT");

        } else {
            tr.stopDrive();
            telemetry.addLine("✓ CENTERED - STOP");
        }
    }
}