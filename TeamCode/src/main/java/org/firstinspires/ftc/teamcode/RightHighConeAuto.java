package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.TesseractConfig;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RightHighConeAuto")
public class RightHighConeAuto extends LinearOpMode {
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor rearLeftMotor;
    public DcMotor rearRightMotor;
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;
    public Servo handServo;

    ElapsedTime time;
    double craneOldTime = 0;
    double leftCraneReference = 0;
    double leftCraneIntegral = 0;
    double leftCraneLastError = 0;
    double leftCranePosition = 1000;
    double rightCraneReference = 0;
    double rightCraneIntegral = 0;
    double rightCraneLastError = 0;
    double rightCranePosition = 1000;

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
        handServo.setPosition(1);

        // Move to in between the Back High Pole and the Right High Pole
        frontLeftMotor.setPower(0.25); // Reversed Motor
        frontRightMotor.setPower(0.25);
        rearLeftMotor.setPower(0.25); // Reversed Motor
        rearRightMotor.setPower(0.25);

        Thread.sleep(5000);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);

        Thread.sleep(500);
        // Align to the Right High Pole
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

        Thread.sleep(500);
        // Raise Crane
        while (!(leftCraneMotor.getCurrentPosition() == leftCranePosition && rightCranePosition == rightCraneMotor.getCurrentPosition())) {
            // Left Crane PID Controller
            craneOldTime = time.milliseconds() - craneOldTime;
            double leftCraneError = leftCraneReference - leftCraneMotor.getCurrentPosition();
            leftCraneIntegral = leftCraneIntegral + (leftCraneError * craneOldTime);
            double leftCraneDerivative = (leftCraneError - leftCraneLastError) / craneOldTime;
            double leftCraneOutput = (TesseractConfig.kP * leftCraneError) + (TesseractConfig.kI * leftCraneIntegral) + (TesseractConfig.kD * leftCraneDerivative);
            leftCraneMotor.setPower(leftCraneOutput);
            leftCraneLastError = leftCraneError;

            // Right Crane PID Controller
            double rightCraneError = rightCraneReference - rightCraneMotor.getCurrentPosition();
            rightCraneIntegral = rightCraneIntegral + (rightCraneError * craneOldTime);
            double rightCraneDerivative = (rightCraneError - rightCraneLastError) / craneOldTime;
            double rightCraneOutput = (TesseractConfig.kP * rightCraneError) + (TesseractConfig.kI * rightCraneIntegral) + (TesseractConfig.kD * rightCraneDerivative);
            rightCraneMotor.setPower(rightCraneOutput);
            rightCraneLastError = rightCraneError;

            craneOldTime = time.milliseconds();
        }

        Thread.sleep(500);

        handServo.setPosition(0);

        Thread.sleep(500);
        // Go back to the left
        frontLeftMotor.setPower(0.25); // Reversed Motor
        frontRightMotor.setPower(-0.25);
        rearLeftMotor.setPower(-0.25); // Reversed Motor
        rearRightMotor.setPower(0.25);

        Thread.sleep(500);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);

        Thread.sleep(500);

        // Go back to the starting tile
        frontLeftMotor.setPower(-0.25); // Reversed Motor
        frontRightMotor.setPower(-0.25);
        rearLeftMotor.setPower(-0.25); // Reversed Motor
        rearRightMotor.setPower(-0.25);

        Thread.sleep(500);
        // Stop Everything
        frontLeftMotor.setPower(0); // Reversed Motor
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0); // Reversed Motor
        rearRightMotor.setPower(0);
    }
}
