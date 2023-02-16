package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;

@TeleOp(name = "Color Test")
public class ColorTest extends LinearOpMode {
    public Camera webcam;

    @Override
    public void runOpMode()  {
        try {
            webcam.createCaptureRequest(1, new Size(1920, 1080), 30);
        } catch (CameraException e) {
            e.printStackTrace();
        }
        waitForStart();
    }
}
