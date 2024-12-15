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
    private Servo leftHookServo;
    private Servo rightHookServo;

    //Positions for Intake Mechanism
    private static final double INIT_POSITION = 0.5;
    private static final double INTAKE_POSITION = 0.2;
    private static final double TRANSFER_POSITION = 0.8;

    //Positions for Deposit Mechanism
    private static final double DEPOSIT_TRANSFER = 0.12;
    private static final double DEPOSIT_DUMP = 0.8;

    //Positions for Ascent Mecanism
    private static final double OPEN_POSITION_RIGHT = 0.5;
    private static final double CLOSED_POSITION_RIGHT = 0.97;
    private static final double OPEN_POSITION_LEFT = 0.47;
    private static final double CLOSED_POSITION_LEFT = 0.97;

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
        double forward = -gamepad1.right_stick_y;
        double drive = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        mecanumDrive.drive(forward, drive, rotate, false);

        //Intake Program
        if (gamepad2.x) {
            IntakeMotor.setPower(-0.5);
        } else {
            IntakeMotor.setPower(0);
        }

        // Set target positions based on gamepad inputs
        if (gamepad2.y) {
            horizontalSlide.setTargetPosition(HorizontalSlidePID.INTAKE_POSITION);
            intakeArmServo.setPosition(INTAKE_POSITION);
        } else if (gamepad2.a) {
            horizontalSlide.setTargetPosition(HorizontalSlidePID.TRANSFER_POSITION);
            intakeArmServo.setPosition(TRANSFER_POSITION);
            IntakeMotor.setPower(0.1); // Adjust intake motor power if needed
        } else if (gamepad2.b && Math.abs(intakeArmServo.getPosition() - TRANSFER_POSITION) > 0.01) {
            IntakeMotor.setPower(1.0);
        }

        // Continuously update horizontal slide PID logic
        horizontalSlide.update();

        // Telemetry for debugging
        telemetry.addData("Current Position", horizontalSlide.getCurrentPosition());
        telemetry.addData("Desired Position", horizontalSlide.getTargetPosition());
        telemetry.update();

        //Vertical Lift Program
        if (gamepad2.left_bumper) {
            depositServo.setPosition(DEPOSIT_DUMP);
        } else {
            depositServo.setPosition(DEPOSIT_TRANSFER);
        }
        // Set target positions with buttons
        if (gamepad2.dpad_down) {
            verticalSlide.setTargetPosition(VerticalSlidePID.LOW_POSITION);
            telemetry.addData("Target Position", "Low");
        } else if (gamepad2.dpad_right) {
            verticalSlide.setTargetPosition(VerticalSlidePID.MID_POSITION);
            telemetry.addData("Target Position", "Mid");
        } else if (gamepad2.dpad_up) {
            verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_POSITION);
            telemetry.addData("Target Position", "High");
        }

        //Ascent Program
        if (gamepad1.dpad_up) {
            leftHookServo.setPosition(CLOSED_POSITION_LEFT);
            rightHookServo.setPosition(CLOSED_POSITION_RIGHT);
            verticalSlide.setTargetPosition(VerticalSlidePID.ASCENT_POSITION_UP);
        } else if (gamepad1.dpad_down) {
            verticalSlide.setTargetPosition(VerticalSlidePID.ASCENT_POSITION_DOWN);
        }

        // Update the lift controller
        verticalSlide.update();

        // Telemetry for debugging
        telemetry.addData("Left Lift Position", verticalSlide.leftLift.getCurrentPosition());
        telemetry.addData("Right Lift Position", verticalSlide.rightLift.getCurrentPosition());
        telemetry.update();

    }
}


