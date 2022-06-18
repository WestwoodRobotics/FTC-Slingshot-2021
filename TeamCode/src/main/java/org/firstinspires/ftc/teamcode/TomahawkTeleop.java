package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp(name="tomahawk")
public class TomahawkTeleop extends LinearOpMode {

    DcMotorEx[] motors = new DcMotorEx[6];
    Servo servoArm;
    Servo wheel;
    String[] names = {"flm","frm", "blm","brm","carousel", "armMotor"};
    @Override
    public void runOpMode() {
        for (int i = 0; i < motors.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
        }
        waitForStart();
        if (isStopRequested()) {
            return;
        }
            while (opModeIsActive()) {
                double x = gamepad1.left_stick_y;
                double y = gamepad1.left_stick_x;
                double r = gamepad1.right_stick_x;
                double[] velocity = {
                        (y + x + r), // left front
                        (y - x - r), // right front
                        (y - x + r), // left                                                                                                   back
                        (y + x - r)  // left back
                };

                for (int i = 0; i < velocity.length; i++) {
                    motors[i].setVelocity(velocity[i] * 1000);
                }
                double armPower = gamepad2.right_stick_y * 0.05;
                motors[5].setPower(armPower);
                double armMovement = gamepad2.left_stick_y * 0.01 + servoArm.getPosition();
                if (Math.abs(armMovement) < 1) {
                    servoArm.setPosition(armMovement);
                }
                if (gamepad2.right_trigger > 0) {
                    wheel.setPosition(gamepad2.right_trigger / 2.0 + 1);
                } else if (gamepad2.left_trigger > 0) {
                    wheel.setPosition(-gamepad2.left_trigger / 2.0 + 1);
                } else {
                    wheel.setPosition(0.5);
                }
                telemetry.addData("motorVelocities", Arrays.toString(velocity));
                telemetry.addData("armPower", armPower);
                telemetry.addData("servo arm", armMovement);
                telemetry.addData("wheel speed", wheel.getPosition());


            }

        }
}

