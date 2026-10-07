package org.firstinspires.ftc.teamcode.pedropath_test;

import static com.pedropathing.api.Paths.line;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auto Recovery")
public class AutoRecovery extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    // Robot starting position
    private final Pose startPose = p.of(0, 0, 90);

    // Path 1 endpoint
    private final Pose path1End = p.of(0, 24, 90);

    // Path 2 endpoint
    private final Pose path2End = p.of(24, 24, 90);

    private Path path1;
    private Path path2;

    private int state = 0;

    // If robot is more than 3 inches away from
    // the expected position on Path 1,
    // consider it manually moved.
    private static final double RECOVERY_THRESHOLD = 3.0;


    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);

        // -------------------------
        // PATH 1
        // -------------------------

        path1 = line(startPose, path1End)
                .constant(90);

        // -------------------------
        // PATH 2
        // -------------------------

        path2 = line(path1End, path2End)
                .constant(90);
    }


    @Override
    public void start() {

        // Start Path 1
        follower.follow(path1);

        state = 1;
    }


    @Override
    public void loop() {

        // Pedro must be updated continuously
        follower.update();


        // =================================================
        // STATE 1 = FOLLOWING PATH 1
        // =================================================

        if (state == 1) {

            // Check whether robot was manually moved
            if (robotWasMoved()) {

                createRecoveryPath();

                state = 2;
            }

            // Path 1 completed normally
            else if (!follower.isBusy()) {

                follower.follow(path2);

                state = 3;
            }
        }


        // =================================================
        // STATE 2 = FOLLOWING RECOVERY PATH
        // =================================================

        else if (state == 2) {

            // Recovery reached Path 1 endpoint
            if (!follower.isBusy()) {

                follower.follow(path2);

                state = 3;
            }
        }


        // =================================================
        // STATE 3 = FOLLOWING PATH 2
        // =================================================

        else if (state == 3) {

            if (!follower.isBusy()) {

                state = 4;
            }
        }


        // =================================================
        // TELEMETRY
        // =================================================

        Pose actual = follower.pose();
        Pose expected = follower.closestPose();

        telemetry.addData("State", state);

        telemetry.addData(
                "Actual X",
                actual.x()
        );

        telemetry.addData(
                "Actual Y",
                actual.y()
        );

        telemetry.addData(
                "Expected X",
                expected.x()
        );

        telemetry.addData(
                "Expected Y",
                expected.y()
        );

        telemetry.addData(
                "Error",
                getPathError()
        );

        telemetry.addData(
                "Busy",
                follower.isBusy()
        );

        telemetry.update();
    }


    // =====================================================
    // CHECK IF ROBOT WAS MOVED
    // =====================================================

    private boolean robotWasMoved() {

        // Actual robot position
        Pose actual = follower.pose();

        // Closest position on Path 1
        Pose expected = follower.closestPose();

        // Difference in X
        double dx = actual.x() - expected.x();

        // Difference in Y
        double dy = actual.y() - expected.y();

        // Straight-line distance between them
        double error = Math.sqrt(
                dx * dx + dy * dy
        );

        return error > RECOVERY_THRESHOLD;
    }


    // =====================================================
    // CREATE DYNAMIC RECOVERY PATH
    // =====================================================

    private void createRecoveryPath() {

        // Robot's REAL current position
        Pose currentPose = follower.pose();

        /*
         * Create a NEW path:
         *
         * Current robot position
         *          |
         *          |
         *          v
         *     Path 1 endpoint
         */
        Path recoveryPath =
                line(currentPose, path1End)
                        .constant(90);

        // Start recovery
        follower.follow(recoveryPath);
    }


    // =====================================================
    // GET ERROR FROM CURRENT PATH
    // =====================================================

    private double getPathError() {

        Pose actual = follower.pose();
        Pose expected = follower.closestPose();

        double dx = actual.x() - expected.x();
        double dy = actual.y() - expected.y();

        return Math.sqrt(
                dx * dx + dy * dy
        );
    }
}