package org.firstinspires.ftc.teamcode;

import java.lang.*;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Tesseract")
public class TesseractTeleOp extends OpMode {
    // Motors
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor craneMotor;
    // Servos
    public Servo handServo;
    final double ServoSpeed = 0.01;
    double ServoPosition = 0.0;

    @Override
    public void init() {
        // Motor Setup
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RR");
        craneMotor = hardwareMap.get(DcMotor.class, "CRANE");

        // Servo Setup
        handServo = hardwareMap.get(Servo.class, "HAND");

        // Motor Properties
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Servo Properties
        handServo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        // Buttons
        boolean AButton = gamepad1.a;
        boolean BButton = gamepad1.b;
        // Bumpers
        boolean LBumper = gamepad1.right_bumper;
        boolean RBumper = gamepad1.left_bumper;
        // Joysticks
        float LJoyX = gamepad1.left_stick_x;
        float LJoyY = gamepad1.left_stick_y;
        float RJoyX = gamepad1.right_stick_x;
        double LJoyMag = Math.sqrt(Math.pow(LJoyX, 2) + Math.pow(LJoyY, 2));
        // Motor Power
        double FLPower = LJoyX - LJoyY + RJoyX;
        double FRPower = LJoyX + LJoyY - RJoyX;
        double RLPower = LJoyX + LJoyY + RJoyX;
        double RRPower = LJoyX - LJoyY - RJoyX;

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
        if (RBumper) {
            craneMotor.setPower(1.0);
        }
        else if (LBumper) {
            craneMotor.setPower(-1.0);
        }
        else {
            craneMotor.setPower(0.0);
        }
        // Setting Servo Speed
        if (AButton) {
            ServoPosition += ServoSpeed;
        }
        else if (BButton) {
            ServoPosition -= ServoSpeed;
        }
        ServoPosition = Range.clip(ServoPosition, 0.0, 1.0);
        handServo.setPosition(ServoPosition);
        telemetry.addData("Hand Position", handServo.getPosition());
        telemetry.addData("A Button", gamepad1.a);
    }
}