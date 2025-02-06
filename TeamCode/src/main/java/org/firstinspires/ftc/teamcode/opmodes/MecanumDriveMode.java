package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanism.HorizontalSlidePID;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDriveTele;
import org.firstinspires.ftc.teamcode.mechanism.VerticalSlidePID;

@TeleOp()
public class MecanumDriveMode extends OpMode {
    private DcMotor IntakeMotor;
    private Servo intakeArmServo;
    private Servo depositServo;
    private Servo leftHookServo;
    private Servo rightHookServo;

    //Positions for Intake Mechanism Servo
    private static final double INIT_POSITION = 0.5;
    private static final double INTAKE_POSITION_OUT = 0.2;
    private static final double INTAKE_POSITION_IN = 0.15;
    private static final double TRANSFER_POSITION = 0.78;

    //Positions for Deposit Mechanism Servo
    private static final double DEPOSIT_TRANSFER = 0.14;
    private static final double DEPOSIT_DUMP = 0.8;

    //Pincher Positions for level 2 ascent
    private static final double OPEN_POSITION_RIGHT = 0.5;
    private static final double CLOSED_POSITION_RIGHT = 0.97;
    private static final double OPEN_POSITION_LEFT = 0.5;
    private static final double CLOSED_POSITION_LEFT = 0.97;

    //Pincher Positions for scoring specimen
    private static final double OPEN_PINCH_RIGHT = 0.85;
    private static final double OPEN_PINCH_LEFT = 0.85;
    private static final double CLOSED_PINCH_RIGHT = 1.0;
    private static final double CLOSED_PINCH_LEFT = 1.0;

    private HorizontalSlidePID horizontalSlide;
    private VerticalSlidePID verticalSlide;
    MecanumDriveTele mecanumDrive = new MecanumDriveTele();

    private boolean isYPressed = false;
    private boolean isBPressed = false;
    private boolean firstPressComplete = false;
    private boolean rightBumperPressed = false;
    private boolean bButtonPressed = false;
    private boolean isSampleScoringMode = true;
    private boolean psButtonPressed = false;
    boolean isIntakeRunning = false;

    //Variables for Horizontal Slide Timer
    private long startTime = 0;
    private boolean isWaiting = false;

    //Exponential scaling for teleop driving
    private double applyExponentialScaling(double input, double exponent) {
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    @Override
    public void init() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        leftHookServo = hardwareMap.get(Servo.class, "leftHookServo");
        rightHookServo = hardwareMap.get(Servo.class, "rightHookServo");
        intakeArmServo.setPosition(INIT_POSITION);
        depositServo.setPosition(DEPOSIT_TRANSFER);
        leftHookServo.setPosition(OPEN_POSITION_LEFT);
        rightHookServo.setPosition(OPEN_POSITION_RIGHT);
        horizontalSlide = new HorizontalSlidePID(hardwareMap);
        verticalSlide = new VerticalSlidePID(hardwareMap);
        mecanumDrive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        //Mecanum Drive Program
        double rawForward = -gamepad1.right_stick_y;
        double rawDrive = gamepad1.right_stick_x;
        double rawRotate = gamepad1.left_stick_x;

        double forward = applyExponentialScaling(rawForward, 2.0);
        double drive = applyExponentialScaling(rawDrive, 2.0);
        double rotate = applyExponentialScaling(rawRotate, 2.0);

        mecanumDrive.drive(forward, drive, rotate, false);

        // Toggle mode using the PS button - Sample scoring mode and Specimen scoring mode
        if (gamepad2.ps && !psButtonPressed) {
            isSampleScoringMode = !isSampleScoringMode; // Toggle the mode
            psButtonPressed = true; // Prevent multiple detections
            telemetry.addData("Mode", isSampleScoringMode ? "Sample Scoring Mode" : "Specimen Scoring Mode");
        } else if (!gamepad2.ps) {
            psButtonPressed = false; // Reset when the button is released
        }

        if (isSampleScoringMode) {
            //Intake Program for Intake Motor
            if (gamepad2.x) {
                // Reverse intake and turn off the motor
                IntakeMotor.setPower(-0.45);
                isIntakeRunning = false; // Reset the state
            } else if (gamepad2.right_bumper) {
                // Turn on the intake motor and set the state
                IntakeMotor.setPower(1.0);
                isIntakeRunning = true;
            } else if (gamepad2.a) {
                // Lower the intake motor power when gamepad2.a is pushed
                IntakeMotor.setPower(0.1);
                isIntakeRunning = true; // Keep the state as running
            } else if (isIntakeRunning) {
                // Keep the intake motor running at full power if the state is true
                IntakeMotor.setPower(1.0);
            } else {
                // Turn off the motor when no input and the state is false
                IntakeMotor.setPower(0);
            }


            // Intake program for Horizontal Slide
            if (gamepad2.y && !isYPressed) {  // Detect rising edge of gamepad2.y
                if (!firstPressComplete) {
                    // First press: Move the horizontal slide to the intake position
                    horizontalSlide.setTargetPosition(HorizontalSlidePID.INTAKE_POSITION);
                    intakeArmServo.setPosition(TRANSFER_POSITION); // Move servo to transfer position
                    firstPressComplete = true;
                } else {
                    // Second press: Move the intake arm servo to the intake position
                    intakeArmServo.setPosition(INTAKE_POSITION_OUT);
                    firstPressComplete = false; // Reset for future button presses
                }
                isYPressed = true; // Prevent multiple detections
            } else if (!gamepad2.y) {
                isYPressed = false; // Reset when button is released
            }

            if (gamepad2.b && !isBPressed) {  // Detect rising edge of gamepad2.b
                if (!firstPressComplete) {
                    // First press: Move the horizontal slide to the intake position close
                    horizontalSlide.setTargetPosition(HorizontalSlidePID.INTAKE_POSITION_CLOSE);
                    intakeArmServo.setPosition(TRANSFER_POSITION); // Move servo to transfer position
                    firstPressComplete = true;
                } else {
                    // Second press: Move the intake arm servo to the intake position
                    intakeArmServo.setPosition(INTAKE_POSITION_IN);
                    firstPressComplete = false; // Reset for future button presses
                }
                isBPressed = true; // Prevent multiple detections
            } else if (!gamepad2.b) {
                isBPressed = false; // Reset when button is released
            }

            // Move horizontal slide to the transfer position
            if (gamepad2.a && !isWaiting) {
                // Start the timer when the button is pressed
                startTime = System.currentTimeMillis();
                isWaiting = true;

                // Set the servo position immediately
                intakeArmServo.setPosition(TRANSFER_POSITION);
            }

            // Check if the waiting period is over
            if (isWaiting && System.currentTimeMillis() - startTime >= 500) {
                // Move the horizontal slide after 0.5 seconds
                horizontalSlide.setTargetPosition(HorizontalSlidePID.TRANSFER_POSITION);

                // End the waiting period
                isWaiting = false;
            }

            //Vertical Lift Program for sample deposit to baskets
            if (gamepad2.left_bumper) {
                depositServo.setPosition(DEPOSIT_DUMP);
            } else {
                depositServo.setPosition(DEPOSIT_TRANSFER);
            }
            // Set target positions with buttons
            if (gamepad2.dpad_down) {
                verticalSlide.setTargetPosition(VerticalSlidePID.LOW_POSITION);
                leftHookServo.setPosition(OPEN_POSITION_LEFT);
                rightHookServo.setPosition(OPEN_POSITION_RIGHT);
                telemetry.addData("Target Position", "Low");
            } else if (gamepad2.dpad_right) {
                verticalSlide.setTargetPosition(VerticalSlidePID.MID_POSITION);
                telemetry.addData("Target Position", "Mid");
            } else if (gamepad2.dpad_up) {
                verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_POSITION);
                telemetry.addData("Target Position", "High");
            }
        } else {

            //Vertical lift program for specimen deposit to submersible

            // Handle right bumper (close servos and hold position)
            if (gamepad2.right_bumper) {
                rightHookServo.setPosition(CLOSED_PINCH_RIGHT);
                leftHookServo.setPosition(CLOSED_PINCH_LEFT);
                rightBumperPressed = true; // Update state
                bButtonPressed = false;   // Reset xButtonPressed
            }

            // Handle b button (lower lift and open servos)
            if (gamepad2.b && rightBumperPressed) {
                verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_CHAMBER_DOWN);
                bButtonPressed = true; // Update state
            }

            // Handle servo opening when lift reaches the HIGH_CHAMBER_DOWN position
            if (bButtonPressed && Math.abs(verticalSlide.leftLift.getCurrentPosition() - VerticalSlidePID.HIGH_CHAMBER_DOWN) < 20) {
                leftHookServo.setPosition(OPEN_PINCH_LEFT);
                rightHookServo.setPosition(OPEN_PINCH_RIGHT);
                rightBumperPressed = false; // Reset state
                bButtonPressed = false;     // Reset state
            }

            // Handle y button (lift to HIGH_CHAMBER_UP and open servos)
            if (gamepad2.a) {
                verticalSlide.setTargetPosition(VerticalSlidePID.GRAB_SPECIMEN);
                leftHookServo.setPosition(OPEN_PINCH_LEFT);
                rightHookServo.setPosition(OPEN_PINCH_RIGHT);
            }

            if (gamepad2.y) {
                verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_CHAMBER_UP);
            }
        }

        //Vertical lift program for level 2 Ascent
        if (gamepad1.dpad_up) {
            leftHookServo.setPosition(CLOSED_POSITION_LEFT);
            rightHookServo.setPosition(CLOSED_POSITION_RIGHT);
            verticalSlide.setTargetPosition(VerticalSlidePID.ASCENT_POSITION_UP);
        } else if (gamepad1.dpad_down) {
            verticalSlide.setTargetPosition(VerticalSlidePID.ASCENT_POSITION_DOWN);
        }

        // Update the vertical slide and horizontal slide controller
        verticalSlide.update();
        horizontalSlide.update();

        // Telemetry for debugging
        telemetry.addData("Left Lift Position", verticalSlide.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift Position", verticalSlide.rightLift.getCurrentPosition());
        telemetry.addData("Current Slide Position", horizontalSlide.getCurrentPosition());
        telemetry.addData("Servo Position", intakeArmServo.getPosition());
        telemetry.addData("Target Slide Position", horizontalSlide.getTargetPosition());
        telemetry.addData("Mode", isSampleScoringMode ? "Sample Scoring Mode" : "Specimen Scoring Mode");
        telemetry.update();

    }
}


