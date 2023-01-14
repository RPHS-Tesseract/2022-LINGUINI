package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "encoder test")
public class testencoder extends LinearOpMode {
    private DcMotor liftMotor;

    @Override
    public void runOpMode() {
        liftMotor = hardwareMap.get(DcMotor.class, "CRANE");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setPower(1);

        liftMotor.setTargetPosition(200);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (liftMotor.isBusy()) {
            telemetry.addData("encoder", "%d", liftMotor.getCurrentPosition());
            telemetry.update();

        }

    }
}