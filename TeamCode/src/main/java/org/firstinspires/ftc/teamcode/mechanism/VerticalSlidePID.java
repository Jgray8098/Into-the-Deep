package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalSlidePID {
    public final DcMotor leftLift;
    public final DcMotor rightLift;

    // PID coefficients
    private static final double KP = 0.014;
    private static final double KI = 0.002;
    private static final double KD = 0.013;

    // Allowable error to stop corrections near the target
    private static final int DEADBAND = 20;

    // Hold power for the low position
    private static final double HOLD_POWER = 0.1;

    // Maximum value for integral sum
    private static final double INTEGRAL_MAX = 1000;

    // Target positions for the lift to deposit in buckets
    public static final int LOW_POSITION = 10;
    public static final int MID_POSITION = 2400;
    public static final int HIGH_POSITION = 4250;

    //Target positions for the lift to score specimens in the high position
    public static final int HIGH_CHAMBER_UP = 2800;
    public static final int HIGH_CHAMBER_DOWN = 2150;
    public static final int GRAB_SPECIMEN = 825;

    //Target positions for level 2 ascent
    public static final int ASCENT_POSITION_UP = 4100;
    public static final int ASCENT_POSITION_DOWN = 3200;

    private int targetPosition = LOW_POSITION;
    private boolean motorsOff = true;
    private double integralSum = 0;
    private double lastError = 0;

    // Variables to handle encoder reset delay
    private long lastResetTime = 0;
    private boolean resettingEncoders = false;

    public VerticalSlidePID(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotor.class, "LeftLift");
        rightLift = hardwareMap.get(DcMotor.class, "RightLift");

        rightLift.setDirection(DcMotor.Direction.REVERSE);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
        motorsOff = position == LOW_POSITION;
    }

    public boolean isAtTarget() {
        int leftError = Math.abs(targetPosition - leftLift.getCurrentPosition());
        int rightError = Math.abs(targetPosition - rightLift.getCurrentPosition());
        return leftError < DEADBAND && rightError < DEADBAND;
    }

    public void update() {
        int leftPosition = leftLift.getCurrentPosition();
        int rightPosition = rightLift.getCurrentPosition();

        // Handle encoder reset at the LOW_POSITION
        if (targetPosition == LOW_POSITION
                && Math.abs(leftPosition - LOW_POSITION) < DEADBAND
                && Math.abs(rightPosition - LOW_POSITION) < DEADBAND) {

            if (!resettingEncoders) {
                // Begin the reset process
                resettingEncoders = true;

                // Stop motors and reset encoders
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Reset PID state
                integralSum = 0;
                lastError = 0;

                System.out.println("Encoders reset: Left = " + leftLift.getCurrentPosition() + ", Right = " + rightLift.getCurrentPosition());
            }

            // Set motors back to normal operation
            leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Apply hold power to maintain position
            leftLift.setPower(HOLD_POWER);
            rightLift.setPower(HOLD_POWER);

            resettingEncoders = false;
            return; // Exit to prevent further updates during reset
        }

        // Calculate errors for each motor
        double leftError = targetPosition - leftPosition;
        double rightError = targetPosition - rightPosition;

        // Synchronize motors by correcting right motor error based on left motor's position
        final double CORRECTION_FACTOR = 0.7;
        if (Math.abs(leftPosition - rightPosition) > DEADBAND) {
            double rightTargetCorrection = leftPosition - rightPosition;
            rightError += rightTargetCorrection * CORRECTION_FACTOR;
        }

        // If within deadband, hold position
        if (Math.abs(leftError) < DEADBAND && Math.abs(rightError) < DEADBAND) {
            integralSum = 0; // Reset integral sum
            leftLift.setPower(HOLD_POWER);
            rightLift.setPower(HOLD_POWER);
            return;
        }

        // Update integral sum and clamp it to prevent windup
        integralSum += (leftError + rightError) / 2.0;
        integralSum = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, integralSum));

        // Calculate derivative term
        double averageError = (leftError + rightError) / 2.0;
        double derivative = averageError - lastError;
        lastError = averageError;

        // Calculate PID output
        double scaleFactor = Math.max(0.1, Math.abs(averageError) / 100.0); // Dynamic scaling
        double leftPower = scaleFactor * (KP * leftError + KI * integralSum + KD * derivative);
        double rightPower = scaleFactor * (KP * rightError + KI * integralSum + KD * derivative);

        // Normalize power to prevent exceeding [-1.0, 1.0]
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Ensure minimum power to overcome static friction
        if (Math.abs(leftPower) < HOLD_POWER && Math.abs(leftError) > DEADBAND) {
            leftPower = Math.signum(leftPower) * HOLD_POWER;
        }
        if (Math.abs(rightPower) < HOLD_POWER && Math.abs(rightError) > DEADBAND) {
            rightPower = Math.signum(rightPower) * HOLD_POWER;
        }

        // Set motor powers
        leftLift.setPower(leftPower);
        rightLift.setPower(rightPower);
    }
}
