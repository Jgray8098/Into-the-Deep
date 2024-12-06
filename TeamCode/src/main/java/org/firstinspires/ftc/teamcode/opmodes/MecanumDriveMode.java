package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.HorizontalSlidePID;
import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.VerticalSlidePID;

@TeleOp()
public class MecanumDriveMode extends OpMode {
    private DcMotor IntakeMotor;
    private Servo intakeArmServo;
    private Servo depositServo;

    private static final double INIT_POSITION = 0.5;
    private static final double INTAKE_POSITION = 0.2;
    private static final double TRANSFER_POSITION = 0.8;

    private static final double DEPOSIT_TRANSFER = 0.7;
    private static final double DEPOSIT_DUMP = 0.8;

    //private static final int REVERSE_DURATION_MS = 3000;
    //private ElapsedTime reverseTimer = new ElapsedTime();
    //private boolean isReversing = false;
    //private boolean hasReversedAtTransfer = false;

    private HorizontalSlidePID horizontalSlide;
    private VerticalSlidePID verticalSlide;
    MecanumDrive mecanumDrive = new MecanumDrive();

    @Override
    public void init() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        intakeArmServo.setPosition(INIT_POSITION);
        depositServo.setPosition(DEPOSIT_TRANSFER);
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
        if (gamepad1.x) {
            IntakeMotor.setPower(-0.5);
        } else {
            IntakeMotor.setPower(0);
        }

        if (gamepad1.y) {
            horizontalSlide.setTargetPosition(HorizontalSlidePID.INTAKE_POSITION);
            if (horizontalSlide.isAtTargetPosition(HorizontalSlidePID.INTAKE_POSITION)) {
                intakeArmServo.setPosition(INTAKE_POSITION);
            }
        } else if (gamepad1.a) {
            IntakeMotor.setPower(0.1);
            horizontalSlide.setTargetPosition(HorizontalSlidePID.TRANSFER_POSITION);
            intakeArmServo.setPosition(TRANSFER_POSITION);
        } else if (gamepad1.b && Math.abs(intakeArmServo.getPosition() - TRANSFER_POSITION) > 0.01) {
            IntakeMotor.setPower(1.0);
        }

        horizontalSlide.update();

        //double position = HorizontalSlide.getCurrentPosition();
        //double desiredPosition = HorizontalSlide.getTargetPosition();
        telemetry.addData("Current Position", horizontalSlide.getCurrentPosition());
        telemetry.addData("Desired Position", horizontalSlide.getTargetPosition());
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


    }
}


