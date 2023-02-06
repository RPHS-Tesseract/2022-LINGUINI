package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.Collections;

@Autonomous(name = "MoveForwardAuto")
public class MoveForwardAuto extends LinearOpMode {
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

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setDirection(DcMotor.Direction.REVERSE);

        // Setting Motor Speeds
        frontLeftMotor.setPower(1); // Reversed Motor
        frontRightMotor.setPower(1);
        rearLeftMotor.setPower(1); // Reversed Motor
        rearRightMotor.setPower(1);

        Thread.sleep(1750);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);
    }
}
