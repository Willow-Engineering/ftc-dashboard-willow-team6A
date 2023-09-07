package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp()
public class Variable_FMwithTampering extends OpMode {
    @Override
    public void init() {



        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int teamNumber = 16072;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Stick x", gamepad1.left_stick_x);
        telemetry.addData("Left Stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
        double speedForward = -gamepad1.left_stick_y / 2.0;
        telemetry.addData("speed Forward", speedForward);

        if(gamepad1.left_stick_y < 0) {
            telemetry.addData("Left Stick", " is Negative");
        }
        else{
            telemetry.addData("Left Stick", " is Negative");
        }
    }
}