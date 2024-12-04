package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.HorizontalSlidePID;

@TeleOp()
public class MecanumDriveMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
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


    @Override
    public void init() {
        drive.init(hardwareMap);
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

    }

    @Override
    public void loop() {
        //Mecanum Drive Program
        double forward = -gamepad1.right_stick_y;
        double right = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        drive.drive(forward, right, rotate, false);

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



