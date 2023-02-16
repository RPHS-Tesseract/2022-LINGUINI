package org.firstinspires.ftc.teamcode;

import java.lang.*;
import java.util.Arrays;
import java.util.Collections;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.geometry.spherical.oned.S1Point;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import org.firstinspires.ftc.teamcode.config.TesseractConfig;

@TeleOp(name = "Tesseract")
public class TesseractTeleOp extends OpMode {
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    // Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;
    public IMU controlIMU;
    public IMU.Parameters IMUparams;
    public YawPitchRollAngles robotOrientation;
    // Servos
    public Servo handServo;
    public Servo wristServo;
    //final double ServoOpenPos = 0.9; // Tune this later; keeps breaking the claw from opening too much
    //final double ServoClosePos = 1.0;
    final int CraneMax = 61000;
    final int CraneMin = 0;
    final int SlowPosition = 3000;
    boolean EncoderToggle = true;
    double OriginalYaw = 0;
    double OriginalPitch = 0;
    double OriginalRoll = 0;
    ElapsedTime time;
    double oldTime = 0;
    double leftCraneOldTime = 0;
    double rightCraneOldTime = 0;
    double leftCraneReference = 0;
    double leftCraneIntegral = 0;
    double leftCraneLastError = 0;
    double leftCranePosition = 0;
    double rightCraneReference = 0;
    double rightCraneIntegral = 0;
    double rightCraneLastError = 0;
    double rightCranePosition = 0;
    double craneSpeed = .5;
    boolean PIDEnabled = false; // Toggle PID
    boolean GyroEnabled = false; // Toggle Gyro Drive
    boolean PIDClass = false; // True: Try to use the PIDHandler file i added; will probably break everything more than it already is lol
    int vbucks = 100000;

    @Override
    public void init() {
        time = new ElapsedTime();
        oldTime = time.milliseconds();
        // Motor Setup
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RR");
        leftCraneMotor = hardwareMap.get(DcMotor.class, "LCRANE");
        rightCraneMotor = hardwareMap.get(DcMotor.class, "RCRANE");

        telemetry = new MultipleTelemetry(dashboard.getTelemetry(), super.telemetry);

        // IMU Setup
        controlIMU = hardwareMap.get(IMU.class, "imu");
        IMUparams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        controlIMU.initialize(IMUparams);
        controlIMU.resetYaw();

        // Servo Setup
        handServo = hardwareMap.get(Servo.class, "HAND");
        wristServo = hardwareMap.get(Servo.class, "WRIST");

        // Motor Properties
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCraneMotor.setDirection(DcMotor.Direction.REVERSE);
        rightCraneMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Encoder Setup
        leftCraneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftCraneReference = leftCraneMotor.getCurrentPosition();
        rightCraneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightCraneReference = rightCraneMotor.getCurrentPosition();

        leftCraneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCraneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftCraneReference = leftCraneMotor.getCurrentPosition();

        // Servo Properties
        handServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        leftCraneReference = leftCraneMotor.getCurrentPosition();
        rightCraneReference = rightCraneMotor.getCurrentPosition();
        // IMU Gyroscope
        robotOrientation = controlIMU.getRobotYawPitchRollAngles();
        double Yaw = robotOrientation.getYaw(AngleUnit.RADIANS);
        double Pitch = robotOrientation.getPitch(AngleUnit.RADIANS);
        double Roll = robotOrientation.getRoll(AngleUnit.RADIANS);
        // Buttons
        boolean AButton = gamepad1.a;
        boolean BButton = gamepad1.b;
        // Bumpers
        boolean DPadUp = gamepad1.dpad_up;
        boolean DPadDown = gamepad1.dpad_down;
        // Joysticks
        double LJoyX = (Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x));
        double LJoyY = (Math.pow(gamepad1.left_stick_y, 2) * Math.signum(gamepad1.left_stick_y));
        double RJoyX = (Math.pow(gamepad1.right_stick_x, 2) * Math.signum(gamepad1.right_stick_x));

        Vector2D LJoyVector = new Vector2D(LJoyX, LJoyY);
        double LJoyM = LJoyVector.getNorm(); // Magnitude
        double LJoyA = 0;

        if (!(LJoyM == 0)) {
            LJoyA = Vector2D.angle(new Vector2D(0, 1) , LJoyVector); // Angle in Radians
        } else {
            LJoyA = 0;
        }

        S1Point DriveAngle = new S1Point(Yaw + LJoyA);
        VectorDrive(GyroEnabled ? Yaw : 0, LJoyX, LJoyY, RJoyX);

        /*if (!(RobotVector.getDistance(new ArrayRealVector(new double[] {0, 0})) == 0)) {
            RobotVector.normalize();
        }*/
        /*RealVector OffsetVector = LJoyVector.add(RobotVector);
        double OffsetX = OffsetVector.getEntry(1);
        double OffsetY = OffsetVector.getEntry(2);*/

        // Old Algorithm
        /*double FLPower = Math.sin(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag+RJoyX;
        double FRPower = Math.cos(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag-RJoyX;
        double RLPower = Math.cos(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag+RJoyX;
        double RRPower = Math.sin(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag-RJoyX;*/


        // Crane Motor Speed
        // double cranePower = 0;
        /*if (DPadUp && !(leftCraneMotor.getCurrentPosition() > CraneMax)) {
            if (!(EncoderToggle) || !(leftCraneMotor.getCurrentPosition() > CraneMax)) {
                // Go upwards
                leftCranePosition += craneSpeed;
                rightCranePosition += craneSpeed;
            }
        } else if (DPadDown && !(leftCraneMotor.getCurrentPosition() < CraneMin)) {
            if (!(EncoderToggle) || !(leftCraneMotor.getCurrentPosition() < CraneMin)) {
               // Go downwards
                leftCranePosition -= craneSpeed;
                rightCranePosition -= craneSpeed;
            }
        }*/

        if (PIDEnabled) {
            if (PIDClass) {
                leftCraneMotor.setPower(PIDHandler.PIDTO(
                        leftCranePosition,
                        time.milliseconds(),
                        leftCraneOldTime,
                        leftCraneReference,
                        leftCraneIntegral,
                        leftCraneLastError,
                        TesseractConfig.kP,
                        TesseractConfig.kI,
                        TesseractConfig.kD
                ));
                rightCraneMotor.setPower(PIDHandler.PIDTO(
                        rightCranePosition,
                        time.milliseconds(),
                        rightCraneOldTime,
                        rightCraneReference,
                        rightCraneIntegral,
                        rightCraneLastError,
                        TesseractConfig.kP,
                        TesseractConfig.kI,
                        TesseractConfig.kD
                ));
            } else {
                // Left Crane PID Controller
                leftCraneOldTime = time.milliseconds() - leftCraneOldTime;
                double leftCraneError = leftCraneReference - leftCraneMotor.getCurrentPosition();
                leftCraneIntegral = leftCraneIntegral + (leftCraneError * leftCraneOldTime);
                double leftCraneDerivative = (leftCraneError - leftCraneLastError) / leftCraneOldTime;
                double leftCraneOutput = (TesseractConfig.kP * leftCraneError) + (TesseractConfig.kI * leftCraneIntegral) + (TesseractConfig.kD * leftCraneDerivative);
                leftCraneMotor.setPower(leftCraneOutput);
                leftCraneLastError = leftCraneError;

                // Right Crane PID Controller
                double rightCraneError = rightCraneReference - rightCraneMotor.getCurrentPosition();
                rightCraneIntegral = rightCraneIntegral + (rightCraneError * rightCraneOldTime);
                double rightCraneDerivative = (rightCraneError - rightCraneLastError) / rightCraneOldTime;
                double rightCraneOutput = (TesseractConfig.kP * rightCraneError) + (TesseractConfig.kI * rightCraneIntegral) + (TesseractConfig.kD * rightCraneDerivative);
                rightCraneMotor.setPower(rightCraneOutput);
                rightCraneLastError = rightCraneError;

                rightCraneOldTime = time.milliseconds();
            }
        } else {
            /*if (DPadDown && leftCraneMotor.getCurrentPosition() > CraneMin) {
                leftCraneMotor.setPower(-1.0*craneSpeed);
                rightCraneMotor.setPower(-1.0*craneSpeed);
            }
            else if (DPadUp && leftCraneMotor.getCurrentPosition() < CraneMax) {
                leftCraneMotor.setPower(1.0*craneSpeed);
                rightCraneMotor.setPower(1.0*craneSpeed);
            }
            else {
                leftCraneMotor.setPower(0.0);
                rightCraneMotor.setPower(0.0);
            }*/

            leftCraneMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            rightCraneMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        }

        if (DPadDown/* && leftCraneMotor.getCurrentPosition() > CraneMin*/) {
            leftCraneMotor.setPower(-1.0);
            rightCraneMotor.setPower(-1.0);
        }
        else if (DPadUp/* && leftCraneMotor.getCurrentPosition() < CraneMax*/) {
            leftCraneMotor.setPower(1.0);
            rightCraneMotor.setPower(1.0);
        }
        else {
            leftCraneMotor.setPower(0.0);
            rightCraneMotor.setPower(0.0);
        }

        /*
        if (craneMotor.getCurrentPosition() > CraneMax || craneMotor.getCurrentPosition() < CraneMin) {
            cranePower = 0;
        }
        */

        // craneMotor.setPower(cranePower);

        // Setting Servo Speed
        if (AButton) {
            handServo.setPosition(TesseractConfig.openServo);
        } 
        else if (BButton) {
            handServo.setPosition(TesseractConfig.closeServo);
        }

        if (gamepad1.left_bumper) {
            wristServo.setPosition(wristServo.getPosition() + .1);
        }
        else if (gamepad1.right_bumper) {
            wristServo.setPosition(wristServo.getPosition() - .1);
        }

        // Encoder Reset
        /*
        if (gamepad1.dpad_left && gamepad1.start && gamepad1.right_bumper && gamepad1.left_bumper) {

        }*/

        telemetry.addData("LJoy Angle ", LJoyA);
        telemetry.addData("LJoy Magnitude:" , LJoyM);
        telemetry.addData("Original Gyro: ", "YAW: %.3f, PITCH: %.3f, ROLL: %.3f", Yaw, Pitch, Roll);
        telemetry.addData("Gyro: ", "YAW: %.3f, PITCH: %.3f, ROLL: %.3f", Yaw, Pitch, Roll);
        telemetry.addData("Gyro Drive: ", GyroEnabled);
        telemetry.addData("Hand:", handServo.getPosition());

        telemetry.addData(
                "PID: ",
                "kP: %.3f, kI: %.3f, kD: %.3f",
                TesseractConfig.kP,
                TesseractConfig.kI,
                TesseractConfig.kD
        );
    }
    /*private void PIDTo(double Target, double Time, double Reference, double Integral, double  double kP, double kI, double kD) {
        Time = time.milliseconds() - craneOldTime;
        double Error = leftCraneReference - leftCraneMotor.getCurrentPosition();
        leftCraneIntegral = leftCraneIntegral + (Error * craneOldTime);
        double leftCraneDerivative = (Error - leftCraneLastError) / craneOldTime;
        double leftCraneOutput = (kP * Error) + (kI * leftCraneIntegral) + (kD * leftCraneDerivative);
        leftCraneMotor.setPower(leftCraneOutput);
        leftCraneLastError = Error;
    }*/

    private void VectorDrive(double Angle, double LJoyX, double LJoyY, double RJoyX) {
        // S1Point OffsetPoint = new S1Point(Angle);
        // Vector2D OffsetVector = OffsetPoint.getVector().scalarMultiply(Mag);
        double OffsetX = Math.cos(Angle)*LJoyX - Math.sin(Angle)*LJoyY;
        double OffsetY = Math.sin(Angle)*LJoyX + Math.cos(Angle)*LJoyY;

        // Motor Power
        double FLPower = -OffsetX + OffsetY - RJoyX;
        double FRPower = OffsetX + OffsetY + RJoyX;
        double RLPower = OffsetX + OffsetY - RJoyX;
        double RRPower = -OffsetX + OffsetY + RJoyX;
        double largestPower = Collections.max(Arrays.asList(FLPower, FRPower, RLPower, RRPower, 1.0));

        // Motor Power after scale down
        double ScaledFLPower = FLPower / largestPower;
        double ScaledFRPower = FRPower / largestPower;
        double ScaledRLPower = RLPower / largestPower;
        double ScaledRRPower = RRPower / largestPower;

        // Setting Motor Speeds
        frontLeftMotor.setPower(ScaledFLPower); // Reversed Motor
        frontRightMotor.setPower(ScaledFRPower);
        rearLeftMotor.setPower(ScaledRLPower); // Reversed Motor
        rearRightMotor.setPower(ScaledRRPower);

        telemetry.addData("Front Motors: ","FL: %.3f, FR: %.3f",FLPower, FRPower);
        telemetry.addData("Rear Motors", "RL: %.3f, RR: %.3f", RLPower, RRPower);
        telemetry.addData("Front Offset Motors: ","FL: %.3f, FR: %.3f",FLPower, FRPower);
        telemetry.addData("Rear Offset Motors", "RL: %.3f, RR: %.3f", RLPower, RRPower);
    }
}