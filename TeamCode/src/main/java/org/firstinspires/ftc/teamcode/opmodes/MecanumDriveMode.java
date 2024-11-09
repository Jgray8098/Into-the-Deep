package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanism.MecanumDrive;

@TeleOp()
public class MecanumDriveMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    //private DcMotor IntakeMotorRight;
    //private DcMotor IntakeMotorLeft;
   private DcMotor LiftMotor;
    //int liftUpPosition = -2800;
    //int liftDownPosition = 0;

    @Override
    public void init() {
        drive.init(hardwareMap);
        //IntakeMotorRight = hardwareMap.dcMotor.get("IntakeMotorRight");
        //IntakeMotorLeft = hardwareMap.dcMotor.get("IntakeMotorLeft");
       // LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        //LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LiftMotor.setTargetPosition(liftDownPosition);
        //LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        //Mecanum Drive Program
        double forward = -gamepad1.right_stick_y;
        double right = gamepad1.right_stick_x;
        double rotate = gamepad1.left_stick_x;

        drive.drive(forward, right, rotate);

        //Intake Program
        //if (gamepad1.a) {
         //   IntakeMotorRight.setPower(1.0);
         //   IntakeMotorLeft.setPower(-1.0);
       // }
        //IntakeMotorRight.setPower(0);
        //IntakeMotorLeft.setPower(0);

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
