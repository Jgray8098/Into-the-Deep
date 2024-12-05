package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.HorizontalSlidePID;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.VerticalSlidePID;

@TeleOp()
public class MecanumDriveMode extends OpMode {
    private DcMotor IntakeMotor;
    private boolean isIntakeOn = false; // Tracks the intake state
    private boolean buttonPreviouslyPressed = false; // Tracks button state
    //private DcMotor IntakeMotorLeft;

    //private DcMotor HorizontalSlide;
    //int HorizontalSlideFarPosition = -1500;
    //int HorizontalSlideMidPosition = 0;
    //int HorizontalSLideTransferPosition = -110;
    //int HorizontalSlideInitPosition = 0;
    private HorizontalSlidePID horizontalSlide;
    private VerticalSlidePID verticalSlide;
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void init() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        // Ensure motor is initially off
        IntakeMotor.setPower(0);
        //IntakeMotorLeft = hardwareMap.dcMotor.get("IntakeMotorLeft");

        //HorizontalSlide = hardwareMap.dcMotor.get("HorizontalSlide");
        //HorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //HorizontalSlide.setTargetPosition(HorizontalSlideInitPosition);
        //HorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //HorizontalSlide.setPower(0);
        horizontalSlide = new HorizontalSlidePID(hardwareMap);
        verticalSlide = new VerticalSlidePID(hardwareMap);
        mecanumDrive.init(hardwareMap);
        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void loop() {
        //Mecanum Drive Program
        double forward = -gamepad1.right_stick_y;
        double drive = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        mecanumDrive.drive(forward, drive, rotate, false);

        //Intake Program
        if (gamepad1.y) {
            horizontalSlide.setTargetPosition(-1500);
            //HorizontalSlide.setTargetPosition(HorizontalSlideFarPosition);
            //HorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //HorizontalSlide.setPower(-0.7);
            }
        if (gamepad1.a) {
            horizontalSlide.setTargetPosition(-200);
            //HorizontalSlide.setTargetPosition(HorizontalSLideTransferPosition);
            //HorizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //HorizontalSlide.setPower(0.7);
            }

            horizontalSlide.update();

            //double position = HorizontalSlide.getCurrentPosition();
            //double desiredPosition = HorizontalSlide.getTargetPosition();
            telemetry.addData("Current Position",horizontalSlide.getCurrentPosition());
            telemetry.addData("Desired Position",horizontalSlide.getTargetPosition());
            telemetry.update();

            //Vertical Lift Program
        // Set target positions with buttons
        if (gamepad1.dpad_down) {
            verticalSlide.setTargetPosition(VerticalSlidePID.LOW_POSITION);
            telemetry.addData("Target Position", "Low");
        } else if (gamepad1.dpad_right) {
            verticalSlide.setTargetPosition(VerticalSlidePID.MID_POSITION);
            telemetry.addData("Target Position", "Mid");
        } else if (gamepad1.dpad_up) {
            verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_POSITION);
            telemetry.addData("Target Position", "High");
        }

        // Update the lift controller
        verticalSlide.update();

        // Telemetry for debugging
        telemetry.addData("Left Lift Position", verticalSlide.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift Position", verticalSlide.rightLift.getCurrentPosition());
        telemetry.update();


        //public void loop() {
            // Check if the button is currently pressed (e.g., gamepad1.a)
            boolean isButtonPressed = gamepad1.right_bumper;

            // Toggle logic
            if (isButtonPressed && !buttonPreviouslyPressed) {
                // Button just pressed, toggle intake state
                isIntakeOn = !isIntakeOn;

                // Set motor power based on the new state
                if (isIntakeOn) {
                    IntakeMotor.setPower(1.0); // Turn on intake
                } else {
                    IntakeMotor.setPower(0); // Turn off intake
                }
            }

            //Lift Program
            //if (gamepad1.y) {
            //   LiftMotor.setTargetPosition(liftUpPosition);
            //  LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //   LiftMotor.setPower(-0.5);
            // }
            // if(gamepad1.b) {
            // LiftMotor.setTargetPosition(liftDownPosition);
            //LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //LiftMotor.setPower(0.5);
            // }
            //double position = LiftMotor.getCurrentPosition();
            // double desiredPosition = LiftMotor.getTargetPosition();
            // telemetry.addData("Encoder Position",position);
            // telemetry.addData("Desired Position",desiredPosition);
            // telemetry.update();
            //Gripper Program


        }
    }



