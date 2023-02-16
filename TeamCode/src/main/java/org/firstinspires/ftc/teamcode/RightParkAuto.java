package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.TesseractConfig;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RightParkAuto")
public class RightParkAuto extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;
    public Servo handServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "RL");
        rearRightMotor = hardwareMap.get(DcMotor.class, "RR");
        leftCraneMotor = hardwareMap.get(DcMotor.class, "LCRANE");
        rightCraneMotor = hardwareMap.get(DcMotor.class, "RCRANE");
        handServo = hardwareMap.get(Servo.class, "HAND");

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
        waitForStart();

        // Firmly grasp cone
        handServo.setPosition(TesseractConfig.closeServo);

        // Move Right
        frontLeftMotor.setPower(0.25); // Reversed Motor
        frontRightMotor.setPower(-0.25);
        rearLeftMotor.setPower(-0.25); // Reversed Motor
        rearRightMotor.setPower(0.25);

        Thread.sleep(5000);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);

        // Drop Cone
        handServo.setPosition(TesseractConfig.openServo);
    }
}
