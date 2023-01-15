package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.Collections;

@Autonomous(name = "RightAuto")
public class RightAuto extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor craneMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RR");
        craneMotor = hardwareMap.get(DcMotor.class, "CRANE");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setDirection(DcMotor.Direction.REVERSE);

        // Joysticks
        float LJoyX = -1.0f;
        float LJoyY = 0.0f;
        float RJoyX = 0.0f;
        double LJoyMag = Math.sqrt(Math.pow(LJoyX, 2) + Math.pow(LJoyY, 2));

        // Motor Power
        double FLPower = LJoyX + LJoyY + RJoyX;
        double FRPower = -LJoyX + LJoyY - RJoyX;
        double RLPower = -LJoyX + LJoyY + RJoyX;
        double RRPower = LJoyX + LJoyY - RJoyX;

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

        Thread.sleep(750);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);
    }
}
