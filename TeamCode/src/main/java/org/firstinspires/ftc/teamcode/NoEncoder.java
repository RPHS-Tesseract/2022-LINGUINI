package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "NoEncoder")
public class NoEncoder extends OpMode {
    public DcMotor craneMotor;

    @Override
    public void init() {
        craneMotor = hardwareMap.get(DcMotor.class, "CRANE");
        craneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        craneMotor.setDirection(DcMotor.Direction.REVERSE);
        craneMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double JoyY = gamepad1.left_stick_y;
        craneMotor.setPower(JoyY);
    }
}
