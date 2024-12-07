package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalSlidePID {

    private DcMotor HorizontalSlide;

    // PID constants
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.02;

    // PID state
    private double integral = 0;
    private double previousError = 0;

    private static final int POSITION_TOLERANCE = 15;

    // Slide positions
    private final int POSITION_0 = 0;   // Starting position
    public static final int TRANSFER_POSITION = -135; // Midway position
    public static final int INTAKE_POSITION = -1500; // Full extension
    private int targetPosition = POSITION_0;

    // Constructor
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

    public void update() {
        // Get current position and calculate error
        int currentPosition = HorizontalSlide.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Calculate PID terms
        integral += error;
        double derivative = error - previousError;

        // Calculate motor power
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // Limit motor power to [-1, 1]
        power = Math.max(-0.70, Math.min(0.70, power));

        // Set motor power
        HorizontalSlide.setPower(power);

        // Update previous error
        previousError = error;
    }

    public int getCurrentPosition() {
        return HorizontalSlide.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public boolean isAtTargetPosition(int targetPosition) {
        int currentPosition = HorizontalSlide.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE;
    }
}
