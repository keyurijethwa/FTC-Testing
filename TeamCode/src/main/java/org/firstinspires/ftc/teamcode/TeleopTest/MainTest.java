
    package org.firstinspires.ftc.teamcode.TeleopTest;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    @TeleOp(name = "Main Test")
    public class MainTest extends LinearOpMode {

        ManualDrive md;
        IntakeTest it;
        ShooterTest st;
        RGBTest rt;
        OrdometryTest odometry;

        static final double DRIVE_POWER = 0.8;

        // Target position in inches
        static final double TARGET_X = 0;
        static final double TARGET_Y = 0;

        static final double TARGET_HEADING = 0;

        // Tune on your robot
        static final double KP_POSITION = 0.08;
        static final double KP_HEADING = 0.015;

        static final double MAX_AUTO_POWER = 0.2;
        static final double MAX_TURN_POWER = 0.25;

        static final double POSITION_TOLERANCE = 1.0;
        static final double HEADING_TOLERANCE = 3.0;

        boolean autoMove = false;
        boolean previousA = false;

        @Override
        public void runOpMode() throws InterruptedException {

            md = new ManualDrive(hardwareMap);
            it = new IntakeTest(hardwareMap);
            st = new ShooterTest(hardwareMap);
            rt = new RGBTest(hardwareMap);
            odometry = new OrdometryTest(hardwareMap);

            waitForStart();

            while (opModeIsActive()) {

                odometry.update();

                double x = odometry.getX();
                double y = odometry.getY();
                double heading = odometry.getHeading();

                // Press A once to start moving to the target.
                boolean currentA = gamepad1.a;

                if (currentA && !previousA) {
                    autoMove = !autoMove;
                }

                previousA = currentA;

                if (autoMove) {

                    double errorX = TARGET_X - y;
                    double errorY = TARGET_Y - x;
                    double headingError =
                            ((TARGET_HEADING - heading + 540) % 360) - 180;

                    if (Math.abs(errorX) < POSITION_TOLERANCE
                            && Math.abs(errorY) < POSITION_TOLERANCE
                            && Math.abs(headingError) < HEADING_TOLERANCE) {

                        md.driveRobot(0, 0, 0);
                        autoMove = false;

                    } else {
                        double strafe = clip(
                                errorX * KP_POSITION, MAX_AUTO_POWER);

                        double forward = clip(
                                errorY * KP_POSITION, MAX_AUTO_POWER);

                        double turn = clip(
                                headingError * KP_HEADING, MAX_TURN_POWER);


                        rt.setOrange();
                        // Assumes X = strafe and Y = forward
                        md.driveRobot(forward, strafe, turn);

                    }

                } else {
                    // Normal manual driving
                    md.driveRobot(
                            -gamepad1.left_stick_y * DRIVE_POWER,
                            gamepad1.left_stick_x * DRIVE_POWER,
                            gamepad1.right_stick_x * DRIVE_POWER);
                }

                if(gamepad1.b){
                    md.driveRobot(0,0,0);
                }
                // Intake controls
                if (gamepad1.left_bumper) {
                    it.intakeIn(0.8);
                    rt.setBlue();
                } else if (gamepad1.left_trigger > 0.3) {
                    it.intakeOut(0.8);
                    rt.setGreen();
                } else {
                    it.intakeStop();
                    rt.setOff();
                }

                // Shooter controls
                if (gamepad1.right_bumper ) {
                    st.forwardsh();
                    rt.setRed();
                } else if (gamepad1.right_trigger > 0.3) {
                    st.reversesh();
                    rt.setYellow();
                } else {
                    st.stopsh();
                    rt.setOff();
                }

                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.addData("Target X", TARGET_X);
                telemetry.addData("Target Y", TARGET_Y);
                telemetry.addData("Heading", "%.2f deg", heading);
                telemetry.addData("Auto Move", autoMove);
                telemetry.update();
            }
        }

        private double clip(double value, double max) {
            return Math.max(-max, Math.min(max, value));
        }
    }
