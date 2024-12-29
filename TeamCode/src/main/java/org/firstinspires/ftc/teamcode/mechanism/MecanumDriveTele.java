package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveTele {
    //Drive Motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    //IMU
    private IMU imu;

    //PID constants
    private static final double kP = 1.0;
    private static final double kI = 0.001;
    private static final double kD = 0.3;

    //PID variables
    private double previousError = 0.0;
    private double integral = 0.0;

    public void init(HardwareMap hardwareMap) {

        //code sets orientation of the control hub so imu values are correctly read
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                        LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

        frontLeftMotor = hardwareMap.dcMotor.get("LeftFrontMotor");
        frontRightMotor = hardwareMap.dcMotor.get("RightFrontMotor");
        backLeftMotor = hardwareMap.dcMotor.get("LeftBackMotor");
        backRightMotor = hardwareMap.dcMotor.get("RightBackMotor");

        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set motors to RUN_WITHOUT_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void setPowers(double frontLeftPower, double frontRightPower, double
                           backLeftPower, double backRightPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;

        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void drive(double forward, double right, double rotate, boolean usePID) {

        //Get Yaw angle from IMU
        double currentHeading = getHeading();

        // Traction correction factor for strafing
        double backTractionCorrection = 0.7; //Adjust between 0.0 and 1.0

        //Calculate PID correction if enabled
        double correction = 0;
        if (usePID && Math.abs(rotate) < 0.1) {
            correction = calculatePID(0, currentHeading);
            rotate += correction;
        }

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = (forward - right + rotate);
        double backRightPower = (forward + right - rotate);

        // Apply correction only when strafing is significant
        if (Math.abs(right) > Math.abs(forward) && Math.abs(right) > Math.abs(rotate)) {
            backLeftPower *= backTractionCorrection;
            backRightPower *= backTractionCorrection;
        }

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        //return currentHeading;
        //telemetry.addData("Current Heading", currentHeading);
        //telemetry.update();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

        private double calculatePID(double targetHeading, double currentHeading) {
            //Calculate error
            double error = targetHeading - currentHeading;

            //Normalize error to range [-180, 180]
            if (error > 180) {
                error -= 360;
            }
                else if (error < -180) {
                  error += 360;
                }
                //Calculate integral and derivative
            integral += error;
                double derivative = error - previousError;
                //PID output
            double output = (kP * error) + (kI * integral) + (kD * derivative);
            //Update previous error
            previousError = error;

            return output;
            }

        }


