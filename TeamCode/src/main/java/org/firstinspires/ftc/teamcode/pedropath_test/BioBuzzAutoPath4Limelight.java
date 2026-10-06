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

@Autonomous(name = "BioBuzzAutoPath4LimeLight", group = "Autonomous")
public class BioBuzzAutoPath4Limelight extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private boolean limelightAligning = false;
    private boolean angleValid = false;
    private final Pose start = poseFactory.of(0, 0, 90);
    private final Pose path1 = poseFactory.of(0, 24, 90);
    private final Pose point2 = poseFactory.of(20, 24, 90);
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

    private long alignmentStartTime;
    private static final long ALIGNMENT_TIME = 5000;
    private boolean alignmentComplete = false;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(1000),
                instant(()->follower.stop()),
                alignWithLimelight(),
                waitMs(2000),
                follow(follower,path2())
//                waitMs(2000),
//                follow(follower,path3()),
//                waitMs(2000),
//                follow(follower,path4()),
//                waitMs(2000),
//                follow(follower,path5()),
//                waitMs(2000),
//                follow(follower,path6())
        );
    }
    public Command alignWithLimelight() {

        return Command.build()

                .setStart(() -> {

                    limelightAligning = true;

                    alignmentStartTime =
                            System.currentTimeMillis();

                    angleValid = false;

                    tr.stopDrive();

                    telemetry.addLine(
                            "=== LIMELIGHT ALIGNMENT START ==="
                    );

                    telemetry.update();
                })

                .setExecute(() -> {

                    // =====================================
                    // CHECK 5 SECOND TIME
                    // =====================================

                    long elapsed =
                            System.currentTimeMillis()
                                    - alignmentStartTime;

                    if (elapsed >= ALIGNMENT_TIME) {

                        tr.stopDrive();

                        telemetry.addLine(
                                "=== ALIGNMENT FINISHED ==="
                        );

                        telemetry.update();

                        return;
                    }


                    // =====================================
                    // GET LIMELIGHT RESULT
                    // =====================================

                    LLResult result =
                            limelight.getLatestResult();

                    if (result == null || !result.isValid()) {

                        // Don't move if Limelight has no data
                        tr.stopDrive();

                        telemetry.addLine(
                                "NO VALID LIMELIGHT RESULT"
                        );

                        telemetry.addData(
                                "Time",
                                "%.1f / 5.0 sec",
                                elapsed / 1000.0
                        );

                        telemetry.update();

                        return;
                    }


                    // =====================================
                    // FIND TAG 30
                    // =====================================

                    List<LLResultTypes.FiducialResult> tags =
                            result.getFiducialResults();

                    LLResultTypes.FiducialResult targetTag = null;

                    for (LLResultTypes.FiducialResult tag : tags) {

                        if (tag.getFiducialId()
                                == (int) TARGET_TAG_ID) {

                            targetTag = tag;
                            break;
                        }
                    }


                    // =====================================
                    // TAG NOT FOUND
                    // =====================================

                    if (targetTag == null) {

                        tr.stopDrive();

                        telemetry.addLine(
                                "TAG 30 NOT FOUND"
                        );

                        telemetry.update();

                        return;
                    }


                    // =====================================
                    // GET TAG POSE
                    // =====================================

                    Pose3D targetPose =
                            targetTag.getTargetPoseCameraSpace();

                    if (targetPose == null) {

                        tr.stopDrive();
                        return;
                    }


                    // =====================================
                    // CALCULATE ANGLE
                    // =====================================

                    double x =
                            targetPose.getPosition().x;

                    double z =
                            targetPose.getPosition().z;

                    horizontalAngle =
                            Math.toDegrees(
                                    Math.atan2(x, z)
                            );

                    angleValid = true;


                    // =====================================
                    // ALIGN ROBOT
                    // =====================================

                    if (horizontalAngle > 2.0) {

                        // Robot is NOT centered
                        tr.driveRight(0.5);

                        telemetry.addLine(
                                ">>> MOVING RIGHT"
                        );

                    }

                    else if (horizontalAngle < -2.0) {

                        // Robot is NOT centered
                        tr.driveLeft(0.5);

                        telemetry.addLine(
                                "<<< MOVING LEFT"
                        );

                    }

                    else {

                        // Robot is centered
                        // STOP COMPLETELY
                        tr.stopDrive();

                        telemetry.addLine(
                                "✓ CENTERED - STOPPED"
                        );
                    }


                    // =====================================
                    // TELEMETRY
                    // =====================================

                    telemetry.addData(
                            "Angle",
                            "%.2f°",
                            horizontalAngle
                    );

                    telemetry.addData(
                            "X",
                            "%.3f",
                            x
                    );

                    telemetry.addData(
                            "Z",
                            "%.3f",
                            z
                    );

                    telemetry.addData(
                            "Alignment Time",
                            "%.1f / 5.0 sec",
                            elapsed / 1000.0
                    );

                    telemetry.update();
                })


                // =========================================
                // ONLY FINISH AFTER 5 SECONDS
                // =========================================

                .setDone(() -> {

                    long elapsed =
                            System.currentTimeMillis()
                                    - alignmentStartTime;

                    if (elapsed >= ALIGNMENT_TIME) {

                        tr.stopDrive();

                        limelightAligning = false;

                        telemetry.addLine(
                                "=== 5 SECOND ALIGNMENT COMPLETE ==="
                        );

                        telemetry.update();

                        return true;
                    }

                    return false;
                });
    }
    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
//        s=new ShooterSubsystem1(hardwareMap);
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        tr=new test_robot(hardwareMap);
//        ib=new Intake_Balls(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {

            Scheduler.execute();
            if (!limelightAligning)
            {
                follower.update();
            }
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
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