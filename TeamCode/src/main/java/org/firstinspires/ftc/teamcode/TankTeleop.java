package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@TeleOp(name = "test Tank", group = "TeleOp")
public class TankTeleop extends OpMode{
    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightFront"),              //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("leftBack"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightBack")                //new PIDCoefficients(15, 0, 1))
    };

    double velocityMultiplier = 1;

    public void init() {
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");

        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);

        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
    }

    public void loop() {
        // setting pid
        if (gamepad1.right_bumper) {
            if (velocityMultiplier == 1) {
                velocityMultiplier = 0.2;
            } else {
                velocityMultiplier = 1;
            }
        }

        double y = -gamepad1.left_stick_y; //forward per wheel
        double r = gamepad1.right_stick_x; //turn

        double[] velocity = {
                (y+r),
                (y-r),
                (y+r),
                (y-r),
        };

        double highestValue = 0;
        for (double ix : velocity) {
            if (Math.abs(ix) > highestValue) {
                highestValue = Math.abs(ix);
            }
        }
        if (highestValue > 1) {
            for (double ix : velocity) {
                ix /= highestValue;
            }
        }

        telemetry.addData("Status",             "Run Time: " + runtime.toString());

        telemetry.addData("FRONT LEFT Motor",   motors [0].motor.getVelocity() + "rps");
        telemetry.addData("FRONT RIGHT Motor",  motors [1].motor.getVelocity() + "rps");
        telemetry.addData("BACK LEFT Motor",    motors [2].motor.getVelocity() + "rps");
        telemetry.addData("BACK RIGHT Motor",   motors [3].motor.getVelocity() + "rps");
    }
}
