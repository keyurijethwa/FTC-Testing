package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "AutoPathBioBuzzPath1", group = "Autonomous")
public class BioBuzzAutoPath1 extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(0, 0, 90);
    private final Pose path1 = poseFactory.of(0, 24, 90);
    private final Pose point2 = poseFactory.of(46.5765, 133.6867, 90);
    private ShooterSubsystem1 s;
    private test_robot tr;
    private Intake_Balls ib;
    private Limelight3A limelight;
    private double TARGET_TAG_ID=30;
    private Path lastPath = null;
    private boolean actionTriggered = false;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                        follow(follower, path1())
                );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("Busy", follower.isBusy());
            telemetry.addData("Path Index", follower.pathIndex());
            telemetry.addData("Completion", follower.completion());
            telemetry.addData("Distance", follower.distanceToEndpoint());

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading",Math.toDegrees(follower.pose().heading()));

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

}