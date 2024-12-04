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

    // Slide positions
    private final int POSITION_0 = 0;   // Starting position
    private final int POSITION_1 = -200; // Midway position (adjust based on your setup)
    private final int POSITION_2 = -1500; // Full extension (adjust based on your setup)
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
        power = Math.max(-0.80, Math.min(0.80, power));

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
}
