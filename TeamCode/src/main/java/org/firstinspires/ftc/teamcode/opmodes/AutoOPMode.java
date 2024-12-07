package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Do Nothing Auto -> TeleOp", group = "Test")
public class AutoOPMode extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Autonomous period (30 seconds)
        long autoDuration = 30_000; // 30 seconds in milliseconds
        long startTime = System.currentTimeMillis();

        while (opModeIsActive() && (System.currentTimeMillis() - startTime < autoDuration)) {
            telemetry.addData("Status", "Autonomous Running - Do Nothing");
            telemetry.update();
        }

        // Notify the drivers to switch to teleop
        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.addData("Next Step", "Switch to TeleOp: MecanumDriveMode");
        telemetry.update();

        // End the OpMode
        sleep(1000); // Optional: Small delay before the OpMode stops
    }
}