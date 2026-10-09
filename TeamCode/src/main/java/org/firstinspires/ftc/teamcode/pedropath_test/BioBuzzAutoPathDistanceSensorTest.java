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

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Intake_Balls;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BioBuzzAutoPathDistanceSensorTest", group = "Autonomous")
public class BioBuzzAutoPathDistanceSensorTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(0, 0, 90);
    private final Pose path1 = poseFactory.of(0, 50, 90);
    private final Pose path2=poseFactory.of(50,50,90);


    double distance ;
    private Intake_Balls ib;
    private DistanceSensor ds;
//    private servo_d s;


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(2000),
                infinite(()-> ib.in(0.8)).until(this::getDistance),
                follow(follower, path2())

        );
    }

    @Override
    public void runOpMode() {
        ib=new Intake_Balls(hardwareMap);
        ds=hardwareMap.get(DistanceSensor.class,"DS");

//        s=new servo_d(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            distance =
                    ds.getDistance(DistanceUnit.CM);
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    private boolean getDistance() {
        return Double.isFinite(distance) && distance > 0
                && distance < 6.0;
    }

    public Path path1() {

        return line(start, path1).linear(start, path1);
    }
    public Path path2() {

        return line(path1, path2).linear(path1, path2);
    }
}

