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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Autonomous(name = "BioBuzzAutoPathDistanceLimelightTest", group = "Autonomous")
public class BioBuzzAutoPathDistanceLimelightTest extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(0, 0, 90);
    private final Pose path1 = poseFactory.of(0, 50, 90);
    private final Pose path2=poseFactory.of(20,50,90);
    private final Pose path3=poseFactory.of(0,50,90);
    private final Pose path4=poseFactory.of(20,30,90);
    private Limelight3A limelight;

    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(2000),
                follow(follower, path2()),
                waitMs(5000),
                conditional(
                        () -> isDistanceHigh(),

                        // TRUE
                        follow(follower, path3()),

                        // FALSE
                        follow(follower, path4())
                )


                );
    }

    @Override
    public void runOpMode() {
//        ib=new Intake_Balls(hardwareMap);
//        s=new servo_d(hardwareMap);
        limelight=hardwareMap.get(Limelight3A.class,"limelight");

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
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

    public Path path1() {

        return line(start, path1).linear(start, path1);
    }
    public Path path2() {

        return line(path1, path2).linear(path1, path2);
    }

    public Path path3(){
        return line(path2,path3).linear(path2,path3);
    }
    public Path path4(){
        return line(path2,path4).linear(path2,path4);
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

                // meters -> inches
                return Math.sqrt(x * x + y * y + z * z)
                        * 39.3701;
            }
        }

        return -1;
    }
    private boolean isDistanceHigh() {

        double distance = getLimelightDistance();

        telemetry.addData("Limelight Distance", distance);
        telemetry.update();

        return distance > 64.0;
    }
}

