package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


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
        telemetry.addData("Right Stick x", gamepad1.right_stick_x);
        telemetry.addData("Right Stick y", gamepad1.right_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("B button", gamepad1.b);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        double speedForward = -gamepad1.left_stick_y / 2.0;
        telemetry.addData("speed Forward", speedForward);
        telemetry.addData("Joystick Difference",(gamepad1.left_stick_y - gamepad1.right_stick_y));
        telemetry.addData("Trigger Sum",(gamepad1.left_trigger + gamepad1.right_trigger));
        if(gamepad1.left_stick_y < 0) {
            telemetry.addData("Left Stick", " is Negative");
        }
        else{
            telemetry.addData("Left Stick", " is Positive");
        }

        boolean button_pressed = gamepad1.a;

        if(!button_pressed) {
            gamepad1.left_stick_y = (float) (gamepad1.left_stick_y * .5);
        }
        else{
            gamepad1.left_stick_y = (float) (gamepad1.left_stick_y * 1);
            telemetry.addData("Forward Speed: ", speedForward);

        }
        boolean button2_pressed = gamepad1.b;

        while (!button2_pressed) {
            gamepad1.x = gamepad1.y;
            gamepad1.y = gamepad1.x;
            //crazymode note
        }

        if (button2_pressed) {
            motor1.setPower(1)
            //crazymode note
        }
        //If driver presses 'b' run motor1
        }
    }
