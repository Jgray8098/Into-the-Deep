package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalSlidePID {
    public final DcMotor leftLift;
    public final DcMotor rightLift;

    // PID coefficients
    private static final double KP = 0.03;
    private static final double KI = 0.0;
    private static final double KD = 0.02;

    // Allowable error to stop corrections near the target
    private static final int DEADBAND = 5;

    // Hold power for the low position
    private static final double HOLD_POWER = 0.1;

    // Maximum value for integral sum
    private static final double INTEGRAL_MAX = 1000;

    // Target positions for the lift
    public static final int LOW_POSITION = 0;
    public static final int MID_POSITION = 1500;
    public static final int HIGH_POSITION = 3500;

    private int targetPosition = LOW_POSITION;
    private boolean motorsOff = true;
    private double integralSum = 0;
    private double lastError = 0;

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

    public void update() {
        int leftPosition = leftLift.getCurrentPosition();
        int rightPosition = rightLift.getCurrentPosition();

        if (motorsOff) {
            if (targetPosition == LOW_POSITION
                    && Math.abs(leftPosition - LOW_POSITION) < DEADBAND
                    && Math.abs(rightPosition - LOW_POSITION) < DEADBAND) {
                leftLift.setPower(HOLD_POWER);
                rightLift.setPower(HOLD_POWER);
            } else {
                motorsOff = false;
            }
            return;
        }

        double leftError = targetPosition - leftPosition;
        double rightError = targetPosition - rightPosition;

        // Correct right motor target based on left motor's position
        double rightTargetCorrection = leftPosition - rightPosition;
        rightError += rightTargetCorrection * 0.7; // Adjust the correction weight if needed

        // Deadband check
        if (Math.abs(leftError) < DEADBAND && Math.abs(rightError) < DEADBAND) {
            leftLift.setPower(HOLD_POWER);
            rightLift.setPower(HOLD_POWER);
            return;
        }

        integralSum += (leftError + rightError) / 2;
        integralSum = Math.max(Math.min(integralSum, INTEGRAL_MAX), -INTEGRAL_MAX);

        double derivative = (leftError - lastError);

        double scaleFactor = Math.max(0.1, Math.abs(leftError) / 100.0);
        double leftPower = scaleFactor * (KP * leftError + KI * integralSum + KD * derivative);
        double rightPower = scaleFactor * (KP * rightError + KI * integralSum + KD * derivative);

        // Normalize power
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        leftLift.setPower(leftPower);
        rightLift.setPower(rightPower);

        lastError = leftError;
    }
}
