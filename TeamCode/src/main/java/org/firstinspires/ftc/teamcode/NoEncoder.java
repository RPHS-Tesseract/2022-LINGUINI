package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "NoEncoder")
public class NoEncoder extends OpMode {
    public DcMotor leftCraneMotor;
    public DcMotor rightCraneMotor;

    @Override
    public void init() {
        leftCraneMotor = hardwareMap.get(DcMotor.class, "LCRANE");
        rightCraneMotor = hardwareMap.get(DcMotor.class, "RCRANE");
        leftCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightCraneMotor.setDirection(DcMotor.Direction.REVERSE);
        leftCraneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCraneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double JoyY = gamepad1.left_stick_y;
        leftCraneMotor.setPower(JoyY);
        rightCraneMotor.setPower(JoyY);
        telemetry.addData("LCRANE:", leftCraneMotor.getCurrentPosition());
        telemetry.addData("RCRANE:", leftCraneMotor.getCurrentPosition());
    }
}
