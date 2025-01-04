package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.VerticalSlidePID;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Auto Sample Scoring", group = "Autonomous")
public class AutoOpModeSample extends LinearOpMode {

    private VerticalSlidePID verticalSlide;
    private Servo intakeArmServo;
    private Servo depositServo;
    private Servo leftHookServo;
    private Servo rightHookServo;

    //Positions for Intake Mechanism Servo
    private static final double INIT_POSITION = 0.5;

    //Positions for Deposit Mechanism Servo
    private static final double DEPOSIT_TRANSFER = 0.14;
    private static final double DEPOSIT_DUMP = 0.8;

    //Pincher Positions for level 2 ascent
    private static final double OPEN_POSITION_RIGHT = 0.5;
    private static final double OPEN_POSITION_LEFT = 0.47;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(63.0, -39.0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        verticalSlide = new VerticalSlidePID(hardwareMap);

        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        depositServo = hardwareMap.get(Servo.class, "depositServo");
        leftHookServo = hardwareMap.get(Servo.class, "leftHookServo");
        rightHookServo = hardwareMap.get(Servo.class, "rightHookServo");

        //intakeArmServo.setPosition(INIT_POSITION);
        depositServo.setPosition(DEPOSIT_TRANSFER);
        leftHookServo.setPosition(OPEN_POSITION_LEFT);
        rightHookServo.setPosition(OPEN_POSITION_RIGHT);

        Action trajectoryMoveToBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(51, -68))
                .turnTo(Math.toRadians(135))
                .build();

        Pose2d Pose1 = new Pose2d(50, -39, Math.toRadians(135));

        Action trajectoryMoveToSamples = drive.actionBuilder(Pose1)
                .lineToYConstantHeading(-28)
                .turnTo(Math.toRadians(245))
                .strafeToConstantHeading(new Vector2d(50, -67))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        intakeArmServo.setPosition(INIT_POSITION);

        // Follow trajectory
        Actions.runBlocking(new SequentialAction(trajectoryMoveToBasket));

        // Move lift to high position with timeout
        verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_POSITION);
        long liftStartTime = System.currentTimeMillis();
        while (!verticalSlide.isAtTarget() && System.currentTimeMillis() - liftStartTime < 3000 && opModeIsActive()) {
            verticalSlide.update();
            sleep(2); // Update PID frequently
        }

// Proceed regardless of whether the lift has reached the target
        if (!verticalSlide.isAtTarget()) {
            telemetry.addData("Warning", "Lift did not reach target position. Timing out.");
            telemetry.update();
        }

// Perform scoring while holding lift position
        long scoringStartTime = System.currentTimeMillis();
        boolean scoringDone = false;
        while (System.currentTimeMillis() - scoringStartTime < 1500 && opModeIsActive()) {
            verticalSlide.update(); // Ensure lift holds position

            if (!scoringDone) {
                // Execute servo scoring actions only once
                depositServo.setPosition(DEPOSIT_DUMP); // Move to scoring position
                sleep(1000); // Wait momentarily
                depositServo.setPosition(DEPOSIT_TRANSFER); // Return to home position
                scoringDone = true; // Prevent repeated scoring actions
            }

            sleep(2); // Prevent CPU overload
        }

// Move lift to low position with timeout
        verticalSlide.setTargetPosition(VerticalSlidePID.LOW_POSITION);
        liftStartTime = System.currentTimeMillis();
        while (!verticalSlide.isAtTarget() && System.currentTimeMillis() - liftStartTime < 3000 && opModeIsActive()) {
            verticalSlide.update();
            sleep(2); // Update PID frequently
        }

// Proceed regardless of whether the lift has reached the target
        if (!verticalSlide.isAtTarget()) {
            telemetry.addData("Warning", "Lift did not fully return to low position. Timing out.");
            telemetry.update();
        }
        //Actions.runBlocking(new SequentialAction(trajectoryMoveToSamples));
    }
}


