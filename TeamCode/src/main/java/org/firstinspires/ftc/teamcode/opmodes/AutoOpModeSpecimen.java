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

    //Pincher Positions for level 2 ascent
    private static final double OPEN_POSITION_RIGHT = 0.5;
    private static final double OPEN_POSITION_LEFT = 0.47;

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
                .strafeTo(new Vector2d(60, 65))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        intakeArmServo.setPosition(INIT_POSITION);

        // Follow trajectory
        Actions.runBlocking(new SequentialAction(trajectoryMoveToObservation));

    }

}
