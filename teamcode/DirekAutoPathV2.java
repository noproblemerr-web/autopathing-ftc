package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "DirekAutoPathV2 (Visual Planner)", group = "Direk")
public class DirekAutoPathV2 extends LinearOpMode {

    public enum ActionType {
        STOP_AND_WAIT,
        PASS_THROUGH,
        STOP_AND_HOLD
    }

    public static class Waypoint {
        public final double xInches, yInches, headingDeg, waitSeconds, maxSpeed;
        public final ActionType action;
        public final String label;

        public Waypoint(double xInches, double yInches, double headingDeg,
                        ActionType action, double waitSeconds, double maxSpeed, String label) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingDeg = headingDeg;
            this.action = action;
            this.waitSeconds = waitSeconds;
            this.maxSpeed = maxSpeed;
            this.label = label;
        }
    }

    // ================= VISUAL PLANNER WAYPOINTS (DOTS) =================
    private static final Waypoint[] PATH = new Waypoint[] {
            new Waypoint(24.00, 0.00, 90.0, ActionType.PASS_THROUGH, 0.00, 1.00, "Dot 1"),
            new Waypoint(24.00, 24.00, 180.0, ActionType.PASS_THROUGH, 0.00, 1.00, "Dot 2"),
            new Waypoint(0.00, 24.00, -90.0, ActionType.PASS_THROUGH, 0.00, 1.00, "Dot 3"),
            new Waypoint(0.00, 0.00, 0.0, ActionType.STOP_AND_HOLD, 0.00, 1.00, "Dot 4")
    };

    private static final String PINPOINT_NAME    = "odo";
    private static final String FRONT_RIGHT_NAME = "motor1";
    private static final String BACK_RIGHT_NAME  = "motor2";
    private static final String FRONT_LEFT_NAME  = "motor3";
    private static final String BACK_LEFT_NAME   = "motor4";

    private static final double KP_FORWARD = 0.085, KI_FORWARD = 0.025, KD_FORWARD = 0.012, MIN_FORWARD_POWER = 0.075;
    private static final double KP_STRAFE  = 0.130, KI_STRAFE  = 0.045, KD_STRAFE  = 0.015, MIN_STRAFE_POWER  = 0.135;
    private static final double KP_HEADING = 1.05,  KI_HEADING = 0.15,  KD_HEADING = 0.04,  MIN_TURN_POWER    = 0.09;
    private static final double MAX_TURN_SPEED = 0.55;

    private static final double STOP_TOLERANCE_INCHES  = 0.20;
    private static final double PASS_THROUGH_RADIUS_IN = 3.50;
    private static final double HEADING_TOLERANCE_DEG  = 1.0;
    private static final double INTEGRAL_ZONE_INCHES   = 3.5;
    private static final double MAX_INTEGRAL_POWER     = 0.12;

    private GoBildaPinpointDriver odo;
    private DcMotorEx frontRight, backRight, frontLeft, backLeft;

    private double poseX = 0.0, poseY = 0.0, poseHeading = 0.0;
    private double velX = 0.0, velY = 0.0, velHeading = 0.0;
    private double integralFwd = 0.0, integralStrafe = 0.0, integralTurn = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean currentlyBraking = false;

    private double forwardSign = -1.0, strafeSign = 1.0, turnSign = 1.0;
    private int currentIndex = 0;
    private boolean waitingAtDot = false;
    private boolean lastX = false, lastY = false, lastB = false, lastRB = false;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backRight  = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        frontLeft  = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        backLeft   = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        odo.setOffsets(0.0, 0.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();

        telemetry.addData("Status", "DirekAutoPathV2 Ready (%d Waypoints)", PATH.length);
        telemetry.update();

        waitForStart();
        resetOriginAndRestart();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0.0 || dt > 0.2) dt = 0.02;

            updateOdometry();

            if (gamepad1.a) resetOriginAndRestart();
            if ((gamepad1.right_bumper && !lastRB) || gamepad1.dpad_up) {
                currentIndex = 0;
                waitingAtDot = false;
                clearIntegrals();
            }
            if (gamepad1.x && !lastX) forwardSign = -forwardSign;
            if (gamepad1.y && !lastY) strafeSign  = -strafeSign;
            if (gamepad1.b && !lastB) turnSign    = -turnSign;
            lastX = gamepad1.x; lastY = gamepad1.y; lastB = gamepad1.b; lastRB = gamepad1.right_bumper;

            if (PATH.length == 0) {
                setDrivePowers(0, 0, 0, 0);
                continue;
            }

            // Advance immediately through any PASS_THROUGH dots within PASS_THROUGH_RADIUS_IN
            while (currentIndex < PATH.length - 1
                    && PATH[currentIndex].action == ActionType.PASS_THROUGH
                    && Math.hypot(PATH[currentIndex].xInches - poseX, PATH[currentIndex].yInches - poseY) <= PASS_THROUGH_RADIUS_IN) {
                currentIndex++;
                clearIntegrals();
            }

            Waypoint target = PATH[Math.min(currentIndex, PATH.length - 1)];
            boolean isPassThroughSegment = (target.action == ActionType.PASS_THROUGH && currentIndex < PATH.length - 1);
            double targetHeadingRad = Math.toRadians(target.headingDeg);

            double errorX = target.xInches - poseX;
            double errorY = target.yInches - poseY;
            double errorHeading = AngleUnit.normalizeRadians(targetHeadingRad - poseHeading);

            double cosH = Math.cos(-poseHeading);
            double sinH = Math.sin(-poseHeading);

            double robotForwardError = errorX * cosH - errorY * sinH;
            double robotLeftError    = errorX * sinH + errorY * cosH;

            double robotVelFwd  = velX * cosH - velY * sinH;
            double robotVelLeft = velX * sinH + velY * cosH;

            double distToTarget = Math.hypot(errorX, errorY);
            double headingErrorDeg = Math.abs(Math.toDegrees(errorHeading));

            boolean isAtForward = Math.abs(robotForwardError) < STOP_TOLERANCE_INCHES;
            boolean isAtStrafe  = Math.abs(robotLeftError) < STOP_TOLERANCE_INCHES;
            boolean isAtHeading = headingErrorDeg < HEADING_TOLERANCE_DEG;
            boolean isSettledAtDot = !isPassThroughSegment && isAtForward && isAtStrafe && isAtHeading;

            if (gamepad1.left_bumper) {
                setZeroPowerMode(DcMotor.ZeroPowerBehavior.FLOAT);
                setDrivePowers(0, 0, 0, 0);
                clearIntegrals();
            } else if (isSettledAtDot) {
                setDrivePowers(0, 0, 0, 0);
                clearIntegrals();

                if (target.action == ActionType.STOP_AND_WAIT && currentIndex < PATH.length - 1) {
                    if (target.waitSeconds <= 0.01) {
                        waitingAtDot = false;
                        currentIndex++;
                    } else {
                        if (!waitingAtDot) {
                            waitingAtDot = true;
                            waitTimer.reset();
                        }
                        if (waitTimer.seconds() >= target.waitSeconds) {
                            waitingAtDot = false;
                            currentIndex++;
                        }
                    }
                }
            } else {
                waitingAtDot = false;
                setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

                double maxSpeed = target.maxSpeed;
                double forwardCmd = 0.0;
                double strafeRightCmd = 0.0;

                if (isPassThroughSegment && distToTarget > 0.05) {
                    // PASS_THROUGH Cruise Mode: Do NOT brake or throttle down for heading changes!
                    // Drive at full maxSpeed along the vector toward the waypoint so corners are seamless.
                    double dirFwd = robotForwardError / distToTarget;
                    double dirRight = (-robotLeftError) / distToTarget;
                    forwardCmd = forwardSign * dirFwd * maxSpeed;
                    strafeRightCmd = strafeSign * dirRight * maxSpeed;
                    integralFwd = 0.0;
                    integralStrafe = 0.0;
                } else {
                    if (Math.abs(robotForwardError) < INTEGRAL_ZONE_INCHES && !isAtForward) {
                        integralFwd = Range.clip(integralFwd + robotForwardError * dt,
                                -MAX_INTEGRAL_POWER / KI_FORWARD, MAX_INTEGRAL_POWER / KI_FORWARD);
                    } else integralFwd = 0.0;

                    if (Math.abs(robotLeftError) < INTEGRAL_ZONE_INCHES && !isAtStrafe) {
                        integralStrafe = Range.clip(integralStrafe + (-robotLeftError) * dt,
                                -MAX_INTEGRAL_POWER / KI_STRAFE, MAX_INTEGRAL_POWER / KI_STRAFE);
                    } else integralStrafe = 0.0;

                    // Keep high translation speed (0.85) even while rotating so the robot never pauses to turn first
                    double translationScale = (headingErrorDeg > 25.0) ? 0.85 : 1.0;

                    if (!isAtForward) {
                        double dTerm = KD_FORWARD * robotVelFwd;
                        double pidFwd = (KP_FORWARD * robotForwardError) + (KI_FORWARD * integralFwd) - dTerm;
                        forwardCmd = applyMinPower(forwardSign * pidFwd * translationScale, MIN_FORWARD_POWER, maxSpeed);
                    }

                    if (!isAtStrafe) {
                        double dTerm = KD_STRAFE * (-robotVelLeft);
                        double pidStrafe = (KP_STRAFE * (-robotLeftError)) + (KI_STRAFE * integralStrafe) - dTerm;
                        strafeRightCmd = applyMinPower(strafeSign * pidStrafe * translationScale, MIN_STRAFE_POWER, maxSpeed);
                    }
                }

                if (headingErrorDeg < 15.0 && !isAtHeading) {
                    integralTurn = Range.clip(integralTurn + (-errorHeading) * dt,
                            -MAX_INTEGRAL_POWER / KI_HEADING, MAX_INTEGRAL_POWER / KI_HEADING);
                } else integralTurn = 0.0;

                double turnRightCmd = 0.0;
                if (!isAtHeading) {
                    double dTurn = isPassThroughSegment ? 0.0 : (KD_HEADING * (-velHeading));
                    double pidTurn = (KP_HEADING * (-errorHeading)) + (KI_HEADING * integralTurn) - dTurn;
                    turnRightCmd = applyMinPower(turnSign * pidTurn, MIN_TURN_POWER, MAX_TURN_SPEED);
                }

                double flPower = forwardCmd + strafeRightCmd + turnRightCmd;
                double frPower = forwardCmd - strafeRightCmd - turnRightCmd;
                double blPower = forwardCmd - strafeRightCmd + turnRightCmd;
                double brPower = forwardCmd + strafeRightCmd - turnRightCmd;

                double maxWheel = Math.max(
                        Math.max(Math.abs(flPower), Math.abs(frPower)),
                        Math.max(Math.abs(blPower), Math.abs(brPower))
                );
                if (maxWheel > maxSpeed) {
                    flPower = (flPower / maxWheel) * maxSpeed;
                    frPower = (frPower / maxWheel) * maxSpeed;
                    blPower = (blPower / maxWheel) * maxSpeed;
                    brPower = (brPower / maxWheel) * maxSpeed;
                }

                setDrivePowers(flPower, frPower, blPower, brPower);
            }

            telemetry.addData("Active Dot", "%d / %d (%s) [%s]", currentIndex + 1, PATH.length, target.label, target.action);
            telemetry.addData("Target (X, Y, H)", "(%.1f in, %.1f in, %.1f°)",
                    target.xInches, target.yInches, target.headingDeg);
            telemetry.addData("Robot  (X, Y, H)", "(%.2f in, %.2f in, %.1f°)",
                    poseX, poseY, Math.toDegrees(poseHeading));
            telemetry.update();
        }
    }

    private void clearIntegrals() { integralFwd = 0.0; integralStrafe = 0.0; integralTurn = 0.0; }

    private void setZeroPowerMode(DcMotor.ZeroPowerBehavior mode) {
        if (currentlyBraking == (mode == DcMotor.ZeroPowerBehavior.BRAKE)) return;
        currentlyBraking = (mode == DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(mode);
        frontRight.setZeroPowerBehavior(mode);
        backLeft.setZeroPowerBehavior(mode);
        backRight.setZeroPowerBehavior(mode);
    }

    private double applyMinPower(double command, double minPower, double maxPower) {
        if (Math.abs(command) < 1e-6) return 0.0;
        double sign = Math.signum(command);
        double magnitude = Math.max(Math.abs(command), minPower);
        return Range.clip(sign * magnitude, -maxPower, maxPower);
    }

    private void updateOdometry() {
        odo.update();
        Pose2D pinpointPose = odo.getPosition();
        poseX = pinpointPose.getX(DistanceUnit.INCH);
        poseY = pinpointPose.getY(DistanceUnit.INCH);
        poseHeading = pinpointPose.getHeading(AngleUnit.RADIANS);
        velX = odo.getVelX(DistanceUnit.INCH);
        velY = odo.getVelY(DistanceUnit.INCH);
        velHeading = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    private void resetOriginAndRestart() {
        setDrivePowers(0, 0, 0, 0);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        odo.update();
        poseX = 0.0; poseY = 0.0; poseHeading = 0.0;
        currentIndex = 0; waitingAtDot = false;
        clearIntegrals(); loopTimer.reset();
    }

    private void setDrivePowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl); frontRight.setPower(fr);
        backLeft.setPower(bl);  backRight.setPower(br);
    }
}
