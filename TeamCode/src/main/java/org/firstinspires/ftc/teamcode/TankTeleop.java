package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "test Tank", group = "TeleOp")
public class TankTeleop extends OpMode {

//    public class DriveTank extends TankDrive {
//
//    }

    boolean carouselDirection = true;
    float armPower = 1;

    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightFront"),              //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("leftBack"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightBack"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("carousel"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rotateArm"),                //new PIDCoefficients(15, 0, 1))
            new CustomMotor("intake")
    };

    public void init() {

        //Motor Initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");
        motors[4].motor = hardwareMap.get(DcMotorEx.class, "carousel");
        motors[5].motor = hardwareMap.get(DcMotorEx.class, "rotating Arm");
        motors[6].motor = hardwareMap.get(DcMotorEx.class, "intake");

        //Motor Direction
        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[4].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[5].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[6].motor.setDirection(DcMotorEx.Direction.FORWARD);

        //Motor Zero Power Behavior
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[6].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //Motor PID Coefficients
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[6].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);

    }

    public void start() {
        runtime.reset();
    }

    double velocityMultiplier = 1;

    public void loop() {

        //Velocity Multiplier, Setting PID
        if (gamepad1.right_bumper) {
            if (velocityMultiplier == 1) {
                velocityMultiplier = 0.2;
            } else {
                velocityMultiplier = 1;
            }
        }

        double y = -gamepad1.left_stick_y; //forward
        double r = gamepad1.right_stick_x; //turn

        //Individual Wheel Velocity
        double[] velocity = {
                (y + r),    //Front Left
                (y - r),    //Front Right
                (y + r),    //Back Left
                (y - r)     //Back Right
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

        //Velocity Multiplier, Motor RPM
        for (int i = 0; i < 4; i++) {
            motors[i].motor.setVelocity(velocity[i]*5000*velocityMultiplier);
        }

        //Carousel
        if(gamepad1.right_bumper){
            carouselDirection = !carouselDirection;
            motors[4].motor.setPower(0.4);
        } else if (gamepad1.left_bumper) {
            motors[4].motor.setPower(-0.4);
        } else {
            motors[4].motor.setPower(0);
        }

        //Rotating Arm
        if (gamepad2.right_trigger == armPower) {
            motors[5].motor.setPower(0.3);
        } else if (gamepad2.left_trigger == armPower) {
            motors[5].motor.setPower(-0.3);
        } else {
            motors[5].motor.setPower(0);
        }

        //Intake
        if (gamepad2.right_bumper) {
            motors[6].motor.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            motors[6].motor.setPower(-0.3);
        } else {
            motors[6].motor.setPower(0);
        }

        //Captions, Information Display
        telemetry.addData("Status",             "Run Time: " + runtime.toString());
        telemetry.addData("a: ",                       velocityMultiplier);
        telemetry.addData("FRONT LEFT Motor",   motors[0].motor.getVelocity() + "rps");
        telemetry.addData("FRONT RIGHT Motor",  motors[1].motor.getVelocity() + "rps");
        telemetry.addData("BACK LEFT Motor",    motors[2].motor.getVelocity() + "rps");
        telemetry.addData("BACK RIGHT Motor",   motors[3].motor.getVelocity() + "rps");
        telemetry.addData("Carousel",           motors[4].motor.getVelocity() + "rps");
        telemetry.addData("Rotating Arm",       motors[5].motor.getVelocity() + "rps");
        telemetry.addData("Intake",             motors[6].motor.getVelocity() + "rps");

        telemetry.update();
    }

}
