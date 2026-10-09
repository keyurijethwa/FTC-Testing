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
import org.firstinspires.ftc.teamcode.PinpointTest;
import org.firstinspires.ftc.teamcode.ShootBalls;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "BioBuzzAutoBlueDistanceLimelightTest")
public class BioBuzzAutoBlueDistanceLimelightTest extends LinearOpMode {

    private Follower follower;
    private Intake_Balls ib;
    private ShootTest sb;
    private Servo rgbLight;
    private Limelight3A limelight;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(82.7079, 134.7056, 90);

    private final Pose path1Start =
            poseFactory.of(82.7079, 134.7056, 90);

    private final Pose path1 =
            poseFactory.of(83.2981, 102.2404, 90);

    private final Pose point2 =
            poseFactory.of(131.6217, 133.2102, 90);

    private final Pose point3 =
            poseFactory.of(83.2981, 102.2404, 90);

    private final Pose point4Start =
            poseFactory.of(83.2981, 102.2402, 270);


    private final Pose point4 =
            poseFactory.of(83.2981, 37.2404, 270);

    private final Pose point5 =
            poseFactory.of(135.3566, 36.5009, 270);

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
                    rgbGreen();
                    telemetry.addLine("RGB: GREEN - PATH 1");
                }),

                follow(follower, path1()),
                waitMs(4000),
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
                waitMs(4000),

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
        rgbGreen();


        telemetry.addLine("AUTO BLUE PATH 1");
        telemetry.addLine("RGB PATH INDICATOR");
        telemetry.addLine("RGB: GREEN");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        rgbGreen();

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
                        "Current Path Distance",
                        "%.2f",
                        follower.distanceToEndpoint()
                );

                telemetry.addData(
                        "Path Number",
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
        return line(point4,path1).linear(point4,path1);
    }

    public Path path2() {

        return line(
                path1,
                point2
        ).linear(
                path1,
                point2
        );
    }

    public Path path3() {

        return line(
                point2,
                point3
        ).linear(
                point2,
                point3
        );
    }

    public Path path4() {

        return line(
                point4Start,
                point4
        ).linear(
                point4Start,
                point4
        );
    }

    public Path path5() {

        return line(
                point4,
                point5
        ).linear(
                point4,
                point5
        );
    }
    public Path path41(){
        return line(path1,point4).linear(path1,point4);
    }
    private double getLimelightDistance() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> tags =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {

            if (tag.getFiducialId() == 30 || tag.getFiducialId()==35) {

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