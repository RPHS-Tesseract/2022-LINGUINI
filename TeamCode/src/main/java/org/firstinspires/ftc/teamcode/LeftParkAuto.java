package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LeftParkAuto")
public class LeftParkAuto extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RR");
        leftCraneMotor = hardwareMap.get(DcMotor.class, "LCRANE");
        rightCraneMotor = hardwareMap.get(DcMotor.class, "RCRANE");

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftCraneMotor.setDirection(DcMotor.Direction.REVERSE);
        rightCraneMotor.setDirection(DcMotor.Direction.FORWARD);

        // Movement

        // Move Left
        frontLeftMotor.setPower(-0.25); // Reversed Motor
        frontRightMotor.setPower(0.25);
        rearLeftMotor.setPower(0.25); // Reversed Motor
        rearRightMotor.setPower(-0.25);

        Thread.sleep(500);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);
    }
}
