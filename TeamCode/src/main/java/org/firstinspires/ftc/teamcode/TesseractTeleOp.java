package org.firstinspires.ftc.teamcode;

import java.lang.*;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@TeleOp(name = "Tesseract")
public class TesseractTeleOp extends OpMode {
    // Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor craneMotor;
    public IMU controlIMU;
    public IMU.Parameters IMUparams;
    public YawPitchRollAngles robotOrientation;
    // Servos
    public Servo handServo;
    final double ServoOpenPos = 0.85;
    final double ServoClosePos = 1.0;
    final int CraneMax = 61000;
    final int CraneMin = 0;
    final int SlowPosition = 3000;
    boolean EncoderToggle = true;

    @Override
    public void init() {
        // Motor Setup
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RR");
        craneMotor = hardwareMap.get(DcMotor.class, "CRANE");

        // IMU Setup
        controlIMU = hardwareMap.get(IMU.class, "imu");
        IMUparams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        controlIMU.initialize(IMUparams);


        // Servo Setup
        handServo = hardwareMap.get(Servo.class, "HAND");

        // Motor Properties
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setDirection(DcMotor.Direction.REVERSE);

        craneMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        craneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Servo Properties
        handServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
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
        double LJoyM = Math.sqrt(Math.pow(LJoyX, 2) + Math.pow(LJoyY, 2)); // Magnitude
        double LJoyA = Math.atan2(LJoyY, LJoyX); // Angle in Radians
        double RobotX = Math.sin(LJoyA);
        double RobotY = Math.cos(LJoyA);

        RealVector LJoyVector = new ArrayRealVector(new double[] {LJoyX, LJoyY});
        RealVector RobotVector = new ArrayRealVector(new double[] {RobotX, RobotY});
        if (!(LJoyVector.getDistance(new ArrayRealVector(new double[] {0, 0})) == 0)) {
            LJoyVector.unitize();
        }
        if (!(RobotVector.getDistance(new ArrayRealVector(new double[] {0, 0})) == 0)) {
            RobotVector.unitize();
        }
        RealVector OffsetVector = LJoyVector.add(RobotVector);
        double OffsetX = OffsetVector.getEntry(1);
        double OffsetY = OffsetVector.getEntry(2);

        // Motor Power
        double FLPower = -OffsetX + OffsetY - RJoyX;
        double FRPower = OffsetX + OffsetY + RJoyX;
        double RLPower = OffsetX + OffsetY - RJoyX;
        double RRPower = -OffsetX + OffsetY + RJoyX;

        // Old Algorithm
        /*double FLPower = Math.sin(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag+RJoyX;
        double FRPower = Math.cos(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag-RJoyX;
        double RLPower = Math.cos(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag+RJoyX;
        double RRPower = Math.sin(Math.atan2(LJoyX, LJoyY)+Math.PI/4)*LJoyMag-RJoyX;*/

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

        // Crane Motor Speed
        double cranePower = 0;
        if (DPadUp && !(craneMotor.getCurrentPosition() > CraneMax)) {
            if (!(EncoderToggle) || !(craneMotor.getCurrentPosition() > CraneMax)) {
                cranePower = -1;
            }
        } else if (DPadDown && !(craneMotor.getCurrentPosition() < CraneMin)) {
            if (!(EncoderToggle) || !(craneMotor.getCurrentPosition() < CraneMin)) {
                if (craneMotor.getCurrentPosition() < SlowPosition) {
                    cranePower = 0.2;
                } else {
                    cranePower = 0.5;
                }
            }
        } else {
            cranePower = 0;
        }

        /*
        if (craneMotor.getCurrentPosition() > CraneMax || craneMotor.getCurrentPosition() < CraneMin) {
            cranePower = 0;
        }
        */

        craneMotor.setPower(cranePower);

        /*if (DPadDown && craneMotor.getCurrentPosition() > CraneMin) {
            craneMotor.setPower(-1.0);
        }
        else if (DPadUp && craneMotor.getCurrentPosition() < CraneMax) {
            craneMotor.setPower(1.0);
        }
        else {
            craneMotor.setPower(0.0);
        }*/


        // Setting Servo Speed
        if (AButton) {
            handServo.setPosition(ServoOpenPos);
        }
        else if (BButton) {
            handServo.setPosition(ServoClosePos);
        }

        // Encoder Reset
        /*
        if (gamepad1.dpad_left && gamepad1.start && gamepad1.right_bumper && gamepad1.left_bumper) {

        }*/

        telemetry.addData("Gyro:", "YAW: %.3f, PITCH: %.3f, ROLL: %.3f", Yaw, Pitch, Roll);
        telemetry.addData("Gigachad Motor:", craneMotor.getCurrentPosition());
        telemetry.addData("FrontMotors: ","FL: %.3f, FR: %.3f",FLPower, FRPower);
        telemetry.addData("RearMotors", "RL: %.3f, RR: %.3f", RLPower, RRPower);
        telemetry.addData("FrontOffsetMotors: ","FL: %.3f, FR: %.3f",FLPower, FRPower);
        telemetry.addData("RearOffsetMotors", "RL: %.3f, RR: %.3f", RLPower, RRPower);
    }
}