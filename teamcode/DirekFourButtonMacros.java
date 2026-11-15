package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * Multi-Waypoint 4-Button Macro Follower for "Direk".
 *
 * Press A, B, X, or Y on Gamepad 1 to run that button's own multi-dot Waypoint[] path
 * (e.g., A1 -> A2 -> ... -> Start (0, 0, 0°)) using GoBILDA Pinpoint Odometry PID
 * with smooth PASS_THROUGH cornering and velocity braking.
 *
 * Controls (Gamepad 1):
 * - [A / B / X / Y]   : Run that button's multi-point path
 * - [Left/Right Stick]: Manual Mecanum drive (cancels any active button path safely)
 * - [Right Bumper]    : Emergency cancel active button path
 * - [D-Pad Up]        : Return directly to Starting Point (0, 0, 0°)
 * - [D-Pad Down]      : Reset Starting Origin (0, 0, 0°) at current robot pose
 * - [Hold Left Bumper]: Float wheels (push robot freely by hand)
 */
@TeleOp(name = "Direk 4-Button Point Macros", group = "Direk")
public class DirekFourButtonMacros extends LinearOpMode {

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

    private enum MacroState {
        IDLE,
        RUNNING_PATH
    }

    // ================= 4-BUTTON MULTI-DOT PATHS =================
    private static final Waypoint[] PATH_A = new Waypoint[] {
            new Waypoint(24.00, 0.00, 0.0, ActionType.PASS_THROUGH, 0.00, 0.65, "A1"),
            new Waypoint(24.00, 12.00, 0.0, ActionType.STOP_AND_WAIT, 0.35, 0.65, "A2"),
            new Waypoint(0.00, 0.00, 0.0, ActionType.STOP_AND_HOLD, 0.00, 0.65, "Return Start")
    };

    private static final Waypoint[] PATH_B = new Waypoint[] {
            new Waypoint(24.00, 0.00, 0.0, ActionType.PASS_THROUGH, 0.00, 0.65, "B1"),
            new Waypoint(24.00, -18.00, -90.0, ActionType.STOP_AND_WAIT, 0.35, 0.65, "B2"),
            new Waypoint(0.00, 0.00, 0.0, ActionType.STOP_AND_HOLD, 0.00, 0.65, "Return Start")
    };

    private static final Waypoint[] PATH_X = new Waypoint[] {
            new Waypoint(18.00, 18.00, 45.0, ActionType.STOP_AND_WAIT, 0.35, 0.60, "X1"),
            new Waypoint(0.00, 0.00, 0.0, ActionType.STOP_AND_HOLD, 0.00, 0.60, "Return Start")
    };

    private static final Waypoint[] PATH_Y = new Waypoint[] {
            new Waypoint(36.00, 0.00, 90.0, ActionType.STOP_AND_WAIT, 0.35, 0.65, "Y1"),
            new Waypoint(0.00, 0.00, 0.0, ActionType.STOP_AND_HOLD, 0.00, 0.65, "Return Start")
    };

    private static final Waypoint[] RETURN_HOME_PATH = new Waypoint[] {
            new Waypoint(0.00, 0.00, 0.0, ActionType.STOP_AND_HOLD, 0.00, 0.55, "Return Start")
    };

    // ================= VERIFIED "Direk" HARDWARE CONFIG =================
    private static final String PINPOINT_NAME    = "odo";
    private static final String FRONT_RIGHT_NAME = "motor1";
    private static final String BACK_RIGHT_NAME  = "motor2";
    private static final String FRONT_LEFT_NAME  = "motor3";
    private static final String BACK_LEFT_NAME   = "motor4";

    // ================= VERIFIED PID + SMOOTH PASS_THROUGH GAINS =================
    private static final double KP_FORWARD = 0.085, KI_FORWARD = 0.025, KD_FORWARD = 0.012, MIN_FORWARD_POWER = 0.075;
    private static final double KP_STRAFE  = 0.130, KI_STRAFE  = 0.045, KD_STRAFE  = 0.015, MIN_STRAFE_POWER  = 0.135;
    private static final double KP_HEADING = 1.05,  KI_HEADING = 0.15,  KD_HEADING = 0.04,  MIN_TURN_POWER    = 0.09;
    private static final double MAX_TURN_SPEED = 0.55;

    private static final double STOP_TOLERANCE_INCHES  = 0.25;
    private static final double PASS_THROUGH_RADIUS_IN = 3.50;
    private static final double HEADING_TOLERANCE_DEG  = 1.2;
    private static final double INTEGRAL_ZONE_INCHES   = 3.5;
    private static final double MAX_INTEGRAL_POWER     = 0.12;
    private static final double STICK_DEADBAND         = 0.10;

    private static final double FORWARD_SIGN = -1.0;
    private static final double STRAFE_SIGN  =  1.0;
    private static final double TURN_SIGN    =  1.0;

    private GoBildaPinpointDriver odo;
    private DcMotorEx frontRight, backRight, frontLeft, backLeft;

    private double poseX = 0.0, poseY = 0.0, poseHeading = 0.0;
    private double velX = 0.0, velY = 0.0, velHeading = 0.0;
    private double integralFwd = 0.0, integralStrafe = 0.0, integralTurn = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean currentlyBraking = false;

    private MacroState state = MacroState.IDLE;
    private String activeButtonName = "NONE";
    private Waypoint[] activePath = RETURN_HOME_PATH;
    private int currentIndex = 0;
    private boolean waitingAtDot = false;

    private boolean lastA = false, lastB = false, lastX = false, lastY = false;
    private boolean lastDpadUp = false, lastDpadDown = false, lastRB = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Direk 4-Button Multi-Point Macros Ready");
        telemetry.addLine("Keep robot still ~1s during INIT for Pinpoint IMU calibration.");
        telemetry.update();

        waitForStart();
        resetOriginAndStop();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (dt <= 0.0 || dt > 0.2) dt = 0.02;

            updateOdometry();
            handleControllerInputs();

            if (gamepad1.left_bumper) {
                setZeroPowerMode(DcMotor.ZeroPowerBehavior.FLOAT);
                setDrivePowers(0, 0, 0, 0);
                clearIntegrals();
            } else if (state == MacroState.IDLE) {
                driveManualSticks();
            } else {
                stepActivePath(dt);
            }

            updateTelemetry();
        }
    }

    private void initHardware() {
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
    }

    private void handleControllerInputs() {
        boolean stickMoved = Math.abs(gamepad1.left_stick_y) > STICK_DEADBAND
                || Math.abs(gamepad1.left_stick_x) > STICK_DEADBAND
                || Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND;

        if (stickMoved || (gamepad1.right_bumper && !lastRB)) {
            cancelMacro();
        } else if (gamepad1.dpad_down && !lastDpadDown) {
            resetOriginAndStop();
        } else if (gamepad1.dpad_up && !lastDpadUp) {
            startButtonPath("HOME", RETURN_HOME_PATH);
        } else if (gamepad1.a && !lastA) {
            startButtonPath("A", PATH_A);
        } else if (gamepad1.b && !lastB) {
            startButtonPath("B", PATH_B);
        } else if (gamepad1.x && !lastX) {
            startButtonPath("X", PATH_X);
        } else if (gamepad1.y && !lastY) {
            startButtonPath("Y", PATH_Y);
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastRB = gamepad1.right_bumper;
    }

    private void startButtonPath(String buttonName, Waypoint[] path) {
        if (path == null || path.length == 0) {
            cancelMacro();
            return;
        }
        activeButtonName = buttonName;
        activePath = path;
        currentIndex = 0;
        waitingAtDot = false;
        state = MacroState.RUNNING_PATH;
        clearIntegrals();
    }

    private void cancelMacro() {
        state = MacroState.IDLE;
        activeButtonName = "NONE";
        currentIndex = 0;
        waitingAtDot = false;
        clearIntegrals();
    }

    private void stepActivePath(double dt) {
        if (activePath == null || activePath.length == 0 || currentIndex >= activePath.length) {
            setDrivePowers(0, 0, 0, 0);
            cancelMacro();
            return;
        }

        while (currentIndex < activePath.length - 1
                && activePath[currentIndex].action == ActionType.PASS_THROUGH
                && Math.hypot(activePath[currentIndex].xInches - poseX, activePath[currentIndex].yInches - poseY) <= PASS_THROUGH_RADIUS_IN) {
            currentIndex++;
            waitingAtDot = false;
            clearIntegrals();
        }

        Waypoint target = activePath[currentIndex];
        boolean isPassThroughSegment = (target.action == ActionType.PASS_THROUGH && currentIndex < activePath.length - 1);

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

        if (isSettledAtDot) {
            setDrivePowers(0, 0, 0, 0);
            clearIntegrals();

            if (currentIndex >= activePath.length - 1) {
                cancelMacro();
                return;
            }

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
            return;
        }

        waitingAtDot = false;
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        double maxSpeed = target.maxSpeed;
        double forwardCmd = 0.0;
        double strafeRightCmd = 0.0;

        if (isPassThroughSegment && distToTarget > 0.05) {
            double dirFwd = robotForwardError / distToTarget;
            double dirRight = (-robotLeftError) / distToTarget;
            forwardCmd = FORWARD_SIGN * dirFwd * maxSpeed;
            strafeRightCmd = STRAFE_SIGN * dirRight * maxSpeed;
            integralFwd = 0.0;
            integralStrafe = 0.0;
        } else {
            if (Math.abs(robotForwardError) < INTEGRAL_ZONE_INCHES && !isAtForward) {
                integralFwd = Range.clip(integralFwd + robotForwardError * dt,
                        -MAX_INTEGRAL_POWER / KI_FORWARD, MAX_INTEGRAL_POWER / KI_FORWARD);
            } else {
                integralFwd = 0.0;
            }

            if (Math.abs(robotLeftError) < INTEGRAL_ZONE_INCHES && !isAtStrafe) {
                integralStrafe = Range.clip(integralStrafe + (-robotLeftError) * dt,
                        -MAX_INTEGRAL_POWER / KI_STRAFE, MAX_INTEGRAL_POWER / KI_STRAFE);
            } else {
                integralStrafe = 0.0;
            }

            double translationScale = (headingErrorDeg > 25.0) ? 0.85 : 1.0;

            if (!isAtForward) {
                double pidFwd = (KP_FORWARD * robotForwardError) + (KI_FORWARD * integralFwd) - (KD_FORWARD * robotVelFwd);
                forwardCmd = applyMinPower(FORWARD_SIGN * pidFwd * translationScale, MIN_FORWARD_POWER, maxSpeed);
            }

            if (!isAtStrafe) {
                double pidStrafe = (KP_STRAFE * (-robotLeftError)) + (KI_STRAFE * integralStrafe) - (KD_STRAFE * (-robotVelLeft));
                strafeRightCmd = applyMinPower(STRAFE_SIGN * pidStrafe * translationScale, MIN_STRAFE_POWER, maxSpeed);
            }
        }

        if (headingErrorDeg < 15.0 && !isAtHeading) {
            integralTurn = Range.clip(integralTurn + (-errorHeading) * dt,
                    -MAX_INTEGRAL_POWER / KI_HEADING, MAX_INTEGRAL_POWER / KI_HEADING);
        } else {
            integralTurn = 0.0;
        }

        double turnRightCmd = 0.0;
        if (!isAtHeading) {
            double dTurn = isPassThroughSegment ? 0.0 : (KD_HEADING * (-velHeading));
            double pidTurn = (KP_HEADING * (-errorHeading)) + (KI_HEADING * integralTurn) - dTurn;
            turnRightCmd = applyMinPower(TURN_SIGN * pidTurn, MIN_TURN_POWER, MAX_TURN_SPEED);
        }

        mixAndSetDrivePowers(forwardCmd, strafeRightCmd, turnRightCmd, maxSpeed);
    }

    private void driveManualSticks() {
        setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);

        double fwdInput   = Math.abs(gamepad1.left_stick_y)  > STICK_DEADBAND ? -gamepad1.left_stick_y  : 0.0;
        double rightInput = Math.abs(gamepad1.left_stick_x)  > STICK_DEADBAND ?  gamepad1.left_stick_x  : 0.0;
        double turnInput  = Math.abs(gamepad1.right_stick_x) > STICK_DEADBAND ?  gamepad1.right_stick_x : 0.0;

        mixAndSetDrivePowers(FORWARD_SIGN * fwdInput * 0.70, STRAFE_SIGN * rightInput * 0.70, TURN_SIGN * turnInput * 0.55, 0.85);
    }

    private void mixAndSetDrivePowers(double forwardCmd, double strafeRightCmd, double turnRightCmd, double maxSpeed) {
        double flPower = forwardCmd + strafeRightCmd + turnRightCmd;
        double frPower = forwardCmd - strafeRightCmd - turnRightCmd;
        double blPower = forwardCmd - strafeRightCmd + turnRightCmd;
        double brPower = forwardCmd + strafeRightCmd - turnRightCmd;

        double maxWheel = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        );
        if (maxWheel > maxSpeed && maxWheel > 1e-6) {
            flPower = (flPower / maxWheel) * maxSpeed;
            frPower = (frPower / maxWheel) * maxSpeed;
            blPower = (blPower / maxWheel) * maxSpeed;
            brPower = (brPower / maxWheel) * maxSpeed;
        }

        setDrivePowers(flPower, frPower, blPower, brPower);
    }

    private void clearIntegrals() {
        integralFwd = 0.0;
        integralStrafe = 0.0;
        integralTurn = 0.0;
    }

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
        return Range.clip(Math.signum(command) * Math.max(Math.abs(command), minPower), -maxPower, maxPower);
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

    private void resetOriginAndStop() {
        setDrivePowers(0, 0, 0, 0);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        odo.update();
        poseX = 0.0;
        poseY = 0.0;
        poseHeading = 0.0;
        cancelMacro();
        loopTimer.reset();
    }

    private void setDrivePowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void updateTelemetry() {
        if (state == MacroState.RUNNING_PATH && currentIndex < activePath.length) {
            Waypoint target = activePath[currentIndex];
            telemetry.addData("Macro", "[%s] Dot %d / %d (%s)",
                    activeButtonName, currentIndex + 1, activePath.length, target.label);
        } else {
            telemetry.addData("Macro", "IDLE (Manual Sticks Active)");
        }
        telemetry.addData("Robot Pose", "(X=%.2f\", Y=%.2f\", H=%.1f°)",
                poseX, poseY, Math.toDegrees(poseHeading));
        telemetry.addData("Dots per Btn", "A:%d | B:%d | X:%d | Y:%d",
                PATH_A.length, PATH_B.length, PATH_X.length, PATH_Y.length);
        telemetry.addLine("[A/B/X/Y] Run Path | [Sticks/RB] Cancel | [D-Up] Home | [D-Down] Zero");
        telemetry.update();
    }
}
