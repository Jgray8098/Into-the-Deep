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
import org.firstinspires.ftc.teamcode.mechanism.HorizontalSlidePID;
import org.firstinspires.ftc.teamcode.mechanism.VerticalSlidePID;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Auto Specimen Scoring", group = "Autonomous")
public class AutoOpModeSpecimen extends LinearOpMode {

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

    //Pincher Positions
    private static final double OPEN_POSITION_RIGHT = 0.5;
    private static final double OPEN_POSITION_LEFT = 0.47;
    private static final double OPEN_PINCH_RIGHT = 0.85;
    private static final double OPEN_PINCH_LEFT = 0.85;
    private static final double CLOSED_PINCH_RIGHT = 1.0;
    private static final double CLOSED_PINCH_LEFT = 1.0;

    // Method for scoring specimen
    public void performScoreSpecimen1(Action trajectoryMoveToGrab, Action trajectoryMoveToHighChamber,
                                      Action trajectoryMoveBackToGrab, Action trajectoryMoveToGrab2,
                                      Action trajectoryMoveToHighChamber2, Action trajectoryMoveToPark) {
        // Extend the vertical slide to the grab specimen position
        verticalSlide.setTargetPosition(VerticalSlidePID.GRAB_SPECIMEN);

        // Allow time for the slide to move
        long slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        //Allow time for pinchers and move to occur
        long pincherStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - pincherStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving

            // Move pinchers to open pinch position
            leftHookServo.setPosition(OPEN_PINCH_LEFT);
            rightHookServo.setPosition(OPEN_PINCH_RIGHT);

            // Perform the driving action to move to grab specimen position
            if (trajectoryMoveToGrab != null) {
                trajectoryMoveToGrab.run(new TelemetryPacket());
            }
            sleep(10); // Prevent CPU overload
        }

        //Move pinchers to the closed position
        leftHookServo.setPosition(CLOSED_PINCH_LEFT);
        rightHookServo.setPosition(CLOSED_PINCH_RIGHT);

        //Extend vertical slide to the High Chamber Up position
        verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_CHAMBER_UP);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Allow time for robot to move
        long moveStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - moveStartTime < 4000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            // Perform the driving action to move to grab specimen position
            if (trajectoryMoveToHighChamber != null) {
                trajectoryMoveToHighChamber.run(new TelemetryPacket());
            }
            sleep(10); // Prevent CPU overload
        }

        //Extend vertical slide to the High Chamber Down position
        verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_CHAMBER_DOWN);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving

            sleep(10); // Prevent CPU overload
        }

            //Open pinchers to release speciment
            leftHookServo.setPosition(OPEN_PINCH_LEFT);
            rightHookServo.setPosition(OPEN_PINCH_RIGHT);
            //sleep(300);


        // Allow time for robot to move
        moveStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - moveStartTime < 3500 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            // Perform the driving action to move to grab specimen position
            if (trajectoryMoveBackToGrab != null) {
                trajectoryMoveBackToGrab.run(new TelemetryPacket());
            }
            sleep(10); // Prevent CPU overload
        }

        //Extend vertical slide to the Grab Position
        verticalSlide.setTargetPosition(VerticalSlidePID.GRAB_SPECIMEN);

        //Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Allow time for robot to move
        moveStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - moveStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            // Perform the driving action to move to grab specimen position
            if (trajectoryMoveToGrab2 != null) {
                trajectoryMoveToGrab2.run(new TelemetryPacket());
            }
            sleep(10); // Prevent CPU overload
        }

        //Move pinchers to the closed position
        leftHookServo.setPosition(CLOSED_PINCH_LEFT);
        rightHookServo.setPosition(CLOSED_PINCH_RIGHT);
        sleep(500);

        //Extend vertical slide to the High Chamber Up position
        verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_CHAMBER_UP);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Allow time for robot to move
        moveStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - moveStartTime < 4000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            // Perform the driving action to move to grab specimen position
            if (trajectoryMoveToHighChamber2 != null) {
                trajectoryMoveToHighChamber2.run(new TelemetryPacket());
            }
            sleep(10); // Prevent CPU overload
        }

        //Extend vertical slide to the High Chamber Down position
        verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_CHAMBER_DOWN);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving

            sleep(10); // Prevent CPU overload
        }

        //Open pinchers to release specimen
        leftHookServo.setPosition(OPEN_PINCH_LEFT);
        rightHookServo.setPosition(OPEN_PINCH_RIGHT);
        //sleep(300);

        // Allow time for robot to move
        moveStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - moveStartTime < 3500 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving
            // Perform the driving action to move to grab specimen position
            if (trajectoryMoveToPark != null) {
                trajectoryMoveToPark.run(new TelemetryPacket());
            }
            sleep(10); // Prevent CPU overload
        }

        //Extend vertical slide to the High Chamber Down position
        verticalSlide.setTargetPosition(VerticalSlidePID.LOW_POSITION);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            verticalSlide.update(); // Ensure the slide continues moving

            sleep(10); // Prevent CPU overload
        }
    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(63.0, 20.0, Math.toRadians(180));
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

        Action trajectoryMoveToObservation = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(55, 65))
                .build();

        Pose2d Pose1 = new Pose2d(55, 65, Math.toRadians(180));

        Action trajectoryMoveToGrab = drive.actionBuilder(Pose1)
                .strafeTo(new Vector2d(58, 65))
                .build();

        Pose2d Pose2 = new Pose2d(58, 65, Math.toRadians(180));

        Action trajectoryMoveToHighChamber = drive.actionBuilder(Pose2)
                .strafeToLinearHeading(new Vector2d(18, 20), Math.toRadians(359.8))
                .build();

        Pose2d Pose3 = new Pose2d(18, 20, Math.toRadians(359.8));

        Action trajectoryMoveBackToGrab = drive.actionBuilder(Pose3)
                .strafeToLinearHeading(new Vector2d(45, 65), Math.toRadians(180))
                .build();

        Pose2d Pose4 = new Pose2d(45, 65, Math.toRadians(180));

        Action trajectoryMoveToGrab2 = drive.actionBuilder(Pose4)
                .strafeTo(new Vector2d(54, 65))
                .build();

        Pose2d Pose5 = new Pose2d(54, 65, Math.toRadians(180));

        Action trajectoryMoveToHighChamber2 = drive.actionBuilder(Pose5)
                .strafeToLinearHeading(new Vector2d(17, 18), Math.toRadians(359.8))
                .build();

        Pose2d Pose6 = new Pose2d(17, 18, Math.toRadians(359.8));

        Action trajectoryMoveToPark = drive.actionBuilder(Pose6)
                .strafeToLinearHeading(new Vector2d(45, 65), Math.toRadians(180))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        intakeArmServo.setPosition(INIT_POSITION);

        // Follow trajectory to the observation location
        Actions.runBlocking(new SequentialAction(trajectoryMoveToObservation));

        performScoreSpecimen1(trajectoryMoveToGrab, trajectoryMoveToHighChamber, trajectoryMoveBackToGrab, trajectoryMoveToGrab2,
                trajectoryMoveToHighChamber2, trajectoryMoveToPark);

    }

}
