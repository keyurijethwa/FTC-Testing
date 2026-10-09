package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedropath_test.ShootTest;

import java.util.List;

@Autonomous(name = "BioBuzzAutoRed2DistanceLimelightTest")
public class BioBuzzAutoRed2DistanceLimelightTest extends LinearOpMode {

    private Follower follower;
    private ShootTest sb;
    private Intake_Balls ib;
    private Servo rgbLight;
    private Limelight3A limelight;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(60, 15, 90);

    private final Pose path1Start =
            poseFactory.of(60, 15, 270);

    private final Pose path1 =
            poseFactory.of(65, 41, 270);

    private final Pose path2 =
            poseFactory.of(15, 50, 180);

    private final Pose path3 =
            poseFactory.of(62, 41, 270);

    private final Pose path4Start =
            poseFactory.of(62, 41, 90);

    private final Pose path4 =
            poseFactory.of(62, 118, 90);

    private final Pose path5Start =
            poseFactory.of(62, 118, 90);

    private final Pose path5 =
            poseFactory.of(40, 138, 90);

    private final Pose path6Start =
            poseFactory.of(40, 138, 90);

    private final Pose path6 =
            poseFactory.of(62, 118, 90);

    private final Pose path7 =
            poseFactory.of(13, 110, 90);

    private static final double RGB_RED = 0.30;
    private static final double RGB_YELLOW = 0.35;
    private static final double RGB_GREEN = 0.50;
    private static final double RGB_BLUE = 0.55;

    private void rgbRed() {
        rgbLight.setPosition(RGB_RED);
    }

    private void rgbYellow() {
        rgbLight.setPosition(RGB_YELLOW);
    }

    private void rgbGreen() {
        rgbLight.setPosition(RGB_GREEN);
    }

    private void rgbBlue() {
        rgbLight.setPosition(RGB_BLUE);
    }

    public Command autoRoutine() {

        return sequential(

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - PATH 1");
                }),

                follow(follower, path1()),

                conditional(
                        ()->isDistanceHigh(),
                        distanceMore(),
                        shoot()

                ),

                follow(follower, path2()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - INTAKE 1");
                }),

                instant(() -> ib.in(0.8)),

                waitMs(2000),

                instant(() -> ib.stop1()),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 3");
                }),

                follow(follower, path3()),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 4");
                }),

                follow(follower, path4()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOT 2");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - PATH 5");
                }),

                follow(follower, path5()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - INTAKE 2");
                }),

                instant(() -> ib.in(0.8)),

                waitMs(2000),

                instant(() -> ib.stop1()),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 6");
                }),

                follow(follower, path6()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOT 3");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(2000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 7");
                }),

                follow(follower, path7()),

                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - AUTO COMPLETE");
                })
        );
    }
    public Command shoot() {

        return sequential(
                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                )
        );
    }

    public Command distanceMore() {

        return sequential(
                instant(() -> {
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 51");
                }),

                follow(follower, path41()),

                instant(() -> {
                    rgbYellow();
                    telemetry.addLine("RGB: YELLOW - SECOND DISTANCE CHECK");
                }),

                conditional(
                        () -> isDistanceHigh(),
                        condition2True(),
                        condition2False()
                )
        );
    }
    public Command condition2True() {

        return sequential(
                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 11");
                }),

                follow(follower, path11()),

                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                )
        );
    }

    public Command condition2False() {

        return sequential(
                instant(() -> {
                    rgbRed();
                    telemetry.addLine("RGB: RED - SHOOTING");
                }),

                parallel(
                        instant(() -> sb.forwardsh()),
                        instant(() -> ib.in(0.8))
                ),

                waitMs(3000),

                parallel(
                        instant(() -> sb.stopsh()),
                        instant(() -> ib.stop1())
                ),

                instant(() -> {
                    rgbBlue();
                    telemetry.addLine("RGB: BLUE - PATH 11");
                }),

                follow(follower, path11())
        );
    }


    @Override
    public void runOpMode() {

        ib = new Intake_Balls(hardwareMap);
        sb = new ShootTest(hardwareMap);

        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        rgbLight = hardwareMap.get(
                Servo.class,
                "rgb"
        );

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(start);
        follower.update();
        limelight.pipelineSwitch(0);
        limelight.start();

        rgbRed();


        telemetry.addLine("AUTO RED PATH 1");
        telemetry.addLine("RGB PATH INDICATOR");
        telemetry.addLine("RGB: RED");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        rgbRed();

        schedule(autoRoutine());

        while (opModeIsActive()) {

            follower.update();
            Scheduler.execute();

            telemetry.addData(
                    "X",
                    "%.2f",
                    follower.pose().x()
            );

            telemetry.addData(
                    "Y",
                    "%.2f",
                    follower.pose().y()
            );

            telemetry.addData(
                    "Heading",
                    "%.2f",
                    follower.pose().heading()
            );

            if (follower.currentPath() != null) {
                telemetry.addData(
                        "Distance Remaining",
                        "%.2f",
                        follower.distanceToEndpoint()
                );

                telemetry.addData(
                        "Path Index",
                        follower.pathIndex()
                );
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(
                path1Start,
                path1
        ).linear(
                path1Start,
                path1
        );
    }
    public Path path11(){
        return line(path4,path1).linear(path4,path1);
    }

    public Path path2() {
        return line(
                path1,
                path2
        ).linear(
                path1,
                path2
        );
    }

    public Path path3() {
        return line(
                path2,
                path3
        ).linear(
                path2,
                path3
        );
    }

    public Path path4() {
        return line(
                path4Start,
                path4
        ).linear(
                path4Start,
                path4
        );
    }

    public Path path41(){
        return line(path1,path4).linear(path1,path4);
    }
    public Path path5() {
        return line(
                path5Start,
                path5
        ).linear(
                path5Start,
                path5
        );
    }

    public Path path6() {
        return line(
                path6Start,
                path6
        ).linear(
                path6Start,
                path6
        );
    }

    public Path path7() {
        return line(
                path6,
                path7
        ).linear(
                path6,
                path7
        );
    }
    private double getLimelightDistance() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag.getFiducialId() == 30) {

                Pose3D targetPose =
                        tag.getTargetPoseCameraSpace();

                if (targetPose == null) {
                    return -1;
                }

                double x = targetPose.getPosition().x;
                double y = targetPose.getPosition().y;
                double z = targetPose.getPosition().z;

                return Math.sqrt(
                        x * x +
                                y * y +
                                z * z
                ) * 39.3701;
            }
        }

        return -1;
    }

    private boolean isDistanceHigh() {

        double distance = getLimelightDistance();

        telemetry.addData(
                "Limelight Distance",
                "%.2f in",
                distance
        );

        telemetry.update();

        return distance < 18.5;
    }
}