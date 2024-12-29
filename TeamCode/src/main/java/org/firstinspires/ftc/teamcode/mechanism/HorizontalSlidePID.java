package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalSlidePID {

    private DcMotor HorizontalSlide;

    // PID constants
    private final double kP = 0.008;
    private final double kI = 0.00;
    private final double kD = 0.008;

    // PID state
    private double integral = 0;
    private double previousError = 0;

    private static final int POSITION_TOLERANCE = 15;

    // Slide positions
    private final int POSITION_0 = 0;   // Starting position
    public static final int TRANSFER_POSITION = -130; // Transfer position
    public static final int INTAKE_POSITION = -1500; // Full extension
    public static final int INTAKE_POSITION_CLOSE = -700; // Midway extension
    private int targetPosition = POSITION_0;

    public HorizontalSlidePID(HardwareMap hardwareMap) {
        HorizontalSlide = hardwareMap.get(DcMotor.class, "HorizontalSlide");

        // Initialize the motor
        HorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HorizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Using custom PID
        HorizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getCurrentPosition() {
        return HorizontalSlide.getCurrentPosition();
    }

    public void update() {
        // Get current position and calculate error
        int currentPosition = HorizontalSlide.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // If within tolerance, stop adjusting
        if (Math.abs(error) < POSITION_TOLERANCE) {
            HorizontalSlide.setPower(0);
            integral = 0; // Reset integral when at target
            previousError = 0; // Reset derivative term
            return;
        }

        // Calculate PID terms
        integral += error;
        double derivative = error - previousError;

        // Calculate motor power
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // Limit motor power to [-1, 1]
        power = Math.max(-0.60, Math.min(0.60, power));

        // Set motor power
        HorizontalSlide.setPower(power);

        // Update previous error
        previousError = error;
    }
}