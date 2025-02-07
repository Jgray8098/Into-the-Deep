package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.HorizontalSlidePID;
import org.firstinspires.ftc.teamcode.mechanism.VerticalSlidePID;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Auto Sample Scoring", group = "Autonomous")
public class AutoOpModeSample extends LinearOpMode {

    private VerticalSlidePID verticalSlide;
    private HorizontalSlidePID horizontalSlide;
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
    private static final double OPEN_POSITION_LEFT = 0.5;

    // Reusable method for lift and scoring
    public void performLiftAndScore(int targetPosition, double dumpPosition, double transferPosition) {
        // Move lift to target position with timeout
        verticalSlide.setTargetPosition(targetPosition);
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
                depositServo.setPosition(dumpPosition); // Move to scoring position
                sleep(1000); // Wait momentarily
                depositServo.setPosition(transferPosition); // Return to home position
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
    }

    //Reusable method for horizontal slide motion - Sample 1
    public void performIntakeSequence1(Action trajectoryMoveToSample1) {
        // Step 1: Extend the horizontal slide to the intake position close
        horizontalSlide.setTargetPosition(HorizontalSlidePID.INTAKE_POSITION_CLOSE);

        // Allow time for the slide to move
        long slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 100 && opModeIsActive()) {
            horizontalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Step 2: Move the intake servo to the intake position
        intakeArmServo.setPosition(INTAKE_POSITION_IN);

        // Step 3: Start the intake motor at full power
        long intakeStartTime = System.currentTimeMillis();
        IntakeMotor.setPower(1.0);

        // Perform the driving action and run the intake for 2 seconds
        while (System.currentTimeMillis() - intakeStartTime < 2000 && opModeIsActive()) {
            // Update the driving action if provided
            if (trajectoryMoveToSample1 != null) {
                trajectoryMoveToSample1.run(new TelemetryPacket()); // Execute the action
            }

            // Keep the intake motor running
            IntakeMotor.setPower(1.0);

            // Update the horizontal slide
            horizontalSlide.update();

            // Add a small delay to prevent CPU overload
            sleep(10);
        }

        // Step 4: Move the intake servo to the transfer position
        intakeArmServo.setPosition(TRANSFER_POSITION);

        // Step 5: Reduce motor power to hold the game piece
        IntakeMotor.setPower(0.1);

        // Step 6: Move the horizontal slide to the transfer position
        horizontalSlide.setTargetPosition(HorizontalSlidePID.TRANSFER_POSITION);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            horizontalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Step 7: Set the intake motor to -0.4 power to eject the game piece
        long ejectStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - ejectStartTime < 1000 && opModeIsActive()) {
            IntakeMotor.setPower(-0.4); // Eject the game piece
            sleep(10);
        }

        // Step 8: Turn off the intake motor after ejecting
        IntakeMotor.setPower(0);
    }

    //Reusable method for horizontal slide motion - Sample 2
    public void performIntakeSequence2(Action trajectoryMoveToSample2) {
        // Step 1: Extend the horizontal slide to the intake position close
        horizontalSlide.setTargetPosition(HorizontalSlidePID.INTAKE_POSITION_CLOSE);

        // Allow time for the slide to move
        long slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 100 && opModeIsActive()) {
            horizontalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Step 2: Move the intake servo to the intake position
        intakeArmServo.setPosition(INTAKE_POSITION_IN);

        // Step 3: Start the intake motor at full power
        long intakeStartTime = System.currentTimeMillis();
        IntakeMotor.setPower(1.0);

        // Perform the driving action and run the intake for 2 seconds
        while (System.currentTimeMillis() - intakeStartTime < 2000 && opModeIsActive()) {
            // Update the driving action if provided
            if (trajectoryMoveToSample2 != null) {
                trajectoryMoveToSample2.run(new TelemetryPacket()); // Execute the action
            }

            // Keep the intake motor running
            IntakeMotor.setPower(1.0);

            // Update the horizontal slide
            horizontalSlide.update();

            // Add a small delay to prevent CPU overload
            sleep(10);
        }

        // Step 4: Move the intake servo to the transfer position
        intakeArmServo.setPosition(TRANSFER_POSITION);

        // Step 5: Reduce motor power to hold the game piece
        IntakeMotor.setPower(0.1);

        // Step 6: Move the horizontal slide to the transfer position
        horizontalSlide.setTargetPosition(HorizontalSlidePID.TRANSFER_POSITION);

        // Allow time for the slide to move
        slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 1000 && opModeIsActive()) {
            horizontalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }

        // Step 7: Set the intake motor to -0.4 power to eject the game piece
        long ejectStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - ejectStartTime < 1000 && opModeIsActive()) {
            IntakeMotor.setPower(-0.4); // Eject the game piece
            sleep(10);
        }

        // Step 8: Turn off the intake motor after ejecting
        IntakeMotor.setPower(0);
    }

    public void performIntakeSequence3() {
        // Step 2: Move the intake servo to the init position
        intakeArmServo.setPosition(INIT_POSITION);

        // Step 1: Extend the horizontal slide to the intake position close
        horizontalSlide.setTargetPosition(HorizontalSlidePID.HOME);

        // Allow time for the slide to move
        long slideStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - slideStartTime < 100 && opModeIsActive()) {
            horizontalSlide.update(); // Ensure the slide continues moving
            sleep(10); // Prevent CPU overload
        }
    }

    public class LiftAndScoreAction implements Action {
        private final VerticalSlidePID verticalSlide;
        private final Servo depositServo;
        private final double dumpPosition;
        private final double transferPosition;
        private long stateStartTime = -1;
        private int state = 0;

        public LiftAndScoreAction(VerticalSlidePID verticalSlide, Servo depositServo, double dumpPosition, double transferPosition) {
            this.verticalSlide = verticalSlide;
            this.depositServo = depositServo;
            this.dumpPosition = dumpPosition;
            this.transferPosition = transferPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            verticalSlide.update(); // Continuously update the PID loop

            // Debugging telemetry
            packet.put("LiftAndScore State", state);
            packet.put("Lift Target Position", verticalSlide.targetPosition);
            packet.put("Lift Current Position", verticalSlide.leftLift.getCurrentPosition());
            packet.put("Lift At Target", verticalSlide.isAtTarget());

            switch (state) {
                case 0: // Move to high position
                    if (stateStartTime == -1) {
                        verticalSlide.setTargetPosition(VerticalSlidePID.HIGH_POSITION);
                        stateStartTime = System.currentTimeMillis();
                    }

                    if (verticalSlide.isAtTarget() || System.currentTimeMillis() - stateStartTime > 3000) {
                        state++;
                        stateStartTime = -1;
                    }
                    break;

                case 1: // Dump the game piece
                    if (stateStartTime == -1) {
                        depositServo.setPosition(dumpPosition);
                        stateStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stateStartTime > 1000) { // 1-second delay
                        state++;
                        stateStartTime = -1;
                    }
                    break;

                case 2: // Return deposit to transfer position
                    if (stateStartTime == -1) {
                        depositServo.setPosition(transferPosition);
                        stateStartTime = System.currentTimeMillis();
                    }

                    if (System.currentTimeMillis() - stateStartTime > 200) { // Short delay for servo to move
                        state++;
                        stateStartTime = -1;
                    }
                    break;

                case 3: // Move back to low position
                    if (stateStartTime == -1) {
                        verticalSlide.setTargetPosition(VerticalSlidePID.LOW_POSITION);
                        stateStartTime = System.currentTimeMillis();
                    }

                    if (verticalSlide.isAtTarget() || System.currentTimeMillis() - stateStartTime > 3000) {
                        return true; // Action complete
                    }
                    break;
            }

            return false; // Continue running until all states are complete
        }
    }

    // Method to manage lift updates in a separate thread
    public void manageLiftUpdates(LiftAndScoreAction liftAction) {
        new Thread(() -> {
            while (!liftAction.run(new TelemetryPacket()) && opModeIsActive()) {
                verticalSlide.update(); // Ensure PID updates
                sleep(10); // Prevent CPU overload
            }
        }).start();
    }

        @Override
        public void runOpMode() {
            Pose2d initialPose = new Pose2d(63.0, -39.0, Math.toRadians(180));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
            verticalSlide = new VerticalSlidePID(hardwareMap);
            horizontalSlide = new HorizontalSlidePID(hardwareMap);

            intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
            depositServo = hardwareMap.get(Servo.class, "depositServo");
            leftHookServo = hardwareMap.get(Servo.class, "leftHookServo");
            rightHookServo = hardwareMap.get(Servo.class, "rightHookServo");
            IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");

            //intakeArmServo.setPosition(INIT_POSITION);
            depositServo.setPosition(DEPOSIT_TRANSFER);
            leftHookServo.setPosition(OPEN_POSITION_LEFT);
            rightHookServo.setPosition(OPEN_POSITION_RIGHT);

            // Only reset encoders if the slide is physically at the bottom
            verticalSlide.resetVerticalEncoders();
            horizontalSlide.resetHorizontalEncoders();

            //Trajectory to move robot to basket and score preload.
            Action trajectoryMoveToBasket1 = drive.actionBuilder(initialPose)
                    .strafeTo(new Vector2d(51, -68))
                    .turnTo(Math.toRadians(135))
                    .waitSeconds(3.5)
                    .build();

            Pose2d Pose1 = new Pose2d(51, -68, Math.toRadians(135));

            //Trajectory to move robot to first sample on the field to intake
            Action trajectoryMoveToSample1 = drive.actionBuilder(Pose1)
                    .splineTo(new Vector2d(40, -60), Math.toRadians(182))
                    //.turnTo(Math.toRadians(163))
                    //.strafeToConstantHeading(new Vector2d(42, -65))
                    .build();

            Pose2d Pose2 = new Pose2d(40, -60, Math.toRadians(182));

            //Trajectory to move robot back to basket to score
            Action trajectoryMoveToBasket2 = drive.actionBuilder(Pose2)
                    .strafeTo(new Vector2d(51, -68))
                    .turnTo(Math.toRadians(135))
                    .waitSeconds(4.3)
                    .build();

            Pose2d Pose3 = new Pose2d(51, -68, Math.toRadians(135));

            //Trajectory to move robot to second sample on the field to intake
            Action trajectoryMoveToSample2 = drive.actionBuilder(Pose3)
                    .strafeToLinearHeading(new Vector2d(40, -70), Math.toRadians(199))
                    //.strafeTo(new Vector2d(40, -75))
                    //.turnTo(Math.toRadians(188))
                    //.splineTo(new Vector2d(40, -70), Math.toRadians(188))
                    //.turnTo(Math.toRadians(188))
                    //.strafeTo(new Vector2d(42, -70))
                    .build();

            Pose2d Pose4 = new Pose2d(40, -70, Math.toRadians(199));

            //Trajectory to move robot to back to basket to score
            Action trajectoryMoveToBasket3 = drive.actionBuilder(Pose4)
                    .strafeTo(new Vector2d(51, -68))
                    .turnTo(Math.toRadians(135))
                    .waitSeconds(4.3)
                    .build();

            waitForStart();

            if (isStopRequested()) return;

            intakeArmServo.setPosition(INIT_POSITION);

            // Create the LiftAndScoreAction
            LiftAndScoreAction liftAction1 = new LiftAndScoreAction(verticalSlide, depositServo, DEPOSIT_DUMP, DEPOSIT_TRANSFER);

            // Start the lift updates in a separate thread
            manageLiftUpdates(liftAction1);

            // Create the LiftAndScoreAction
            Actions.runBlocking(
                    new ParallelAction(
                            trajectoryMoveToBasket1, // Trajectory Action
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    // Return true when the lift is complete
                                    return liftAction1.run(packet);
                                }
                            }
                    )
            );

            //Perform the intake sequence with the driving Action
            performIntakeSequence1(trajectoryMoveToSample1);

            // Create the LiftAndScoreAction
            LiftAndScoreAction liftAction2 = new LiftAndScoreAction(verticalSlide, depositServo, DEPOSIT_DUMP, DEPOSIT_TRANSFER);
            manageLiftUpdates(liftAction2);
            Actions.runBlocking(
                    new ParallelAction(
                            trajectoryMoveToBasket2, // Trajectory Action
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                     //Return true when the lift is complete
                                    return liftAction2.run(packet);
                                }
                            }
                    )
            );

            //Perform the intake sequence with the driving action
            performIntakeSequence2(trajectoryMoveToSample2);

            // Create the LiftAndScoreAction
            LiftAndScoreAction liftAction3 = new LiftAndScoreAction(verticalSlide, depositServo, DEPOSIT_DUMP, DEPOSIT_TRANSFER);
            manageLiftUpdates(liftAction3);
            Actions.runBlocking(
                    new ParallelAction(
                            trajectoryMoveToBasket3, // Trajectory Action
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket packet) {
                                    //Return true when the lift is complete
                                    return liftAction3.run(packet);
                                }
                            }
                    )
            );
            //Move intake to home position for transfer to TeleOp
            performIntakeSequence3();
        }
    }


