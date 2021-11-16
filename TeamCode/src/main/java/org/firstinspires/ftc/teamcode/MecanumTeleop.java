package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
import java.util.Arrays;

@TeleOp(name= "test Mecanum", group="TeleOp")
public class MecanumTeleop extends OpMode {
    boolean carouselDirection = true;

    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),
            new CustomMotor("rightFront"),
            new CustomMotor("leftBack"),
            new CustomMotor("rightBack"),
            new CustomMotor("cascadeMotor"),
            new CustomMotor("carouselMotor")
    };

    Servo           leftArm             = null;
    Servo           rightArm            = null;
    double          velocityMultiplier  = 1;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor             = hardwareMap.get(DcMotorEx.class,          "left Front");
        motors[1].motor             = hardwareMap.get(DcMotorEx.class,          "right Front");
        motors[2].motor             = hardwareMap.get(DcMotorEx.class,          "left Back");
        motors[3].motor             = hardwareMap.get(DcMotorEx.class,          "right Back");
        motors[4].motor             = hardwareMap.get(DcMotorEx.class,          "cascade");
        motors[5].motor             = hardwareMap.get(DcMotorEx.class,          "car");
        leftArm                     = hardwareMap.get(Servo.class,              "left Arm");
        rightArm                    = hardwareMap.get(Servo.class,              "right Arm");

        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[4].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[5].motor.setDirection(DcMotorEx.Direction.REVERSE);

        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);

        motors[0].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[1].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[2].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[3].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[4].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[5].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
    }
    @Override
    public void start() {

        runtime.reset();
        motors[4].motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[4].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        // setting pid
            if(gamepad1.right_bumper){
                if(velocityMultiplier == 1){
                    velocityMultiplier = 0.2;
                } else{
                    velocityMultiplier = 1;
                }
            }



            //drivetrain
            double leftPower;
            double rightPower;
            double  y =  -gamepad1.left_stick_y;    //forward(no orientation change)
            double  x =   gamepad1.left_stick_x;    //left right strafe(no orientation change)
            double rx =   gamepad1.right_stick_x;   //rotate left right(orientation change)

            double[] velocity = {
                    (y + x + rx), // left front
                    (y - x - rx), // right front
                   (y - x + rx), // left                                                                                                   back
                    (y + x - rx) // left back
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

            for (int i = 0; i < 4; i++) {
                motors[i].motor.setVelocity(velocity[i]*5000*velocityMultiplier);
            }
            double cascadeVelocity = -gamepad2.right_stick_y*10000;
            if(motors[4].motor.getCurrentPosition()>-2500 && motors[4].motor.getCurrentPosition() <= 5){
                motors[4].motor.setVelocity(-gamepad2.right_stick_y*1000);
            } else if(motors[4].motor.getCurrentPosition()<-2500 && cascadeVelocity>0){
                motors[4].motor.setVelocity(cascadeVelocity);
            } else if(motors[4].motor.getCurrentPosition() > 5 && cascadeVelocity < 0 ){
                motors[4].motor.setVelocity(cascadeVelocity);
            } else{
                motors[4].motor.setVelocity(0);
            }






        //Carousel
            if(gamepad2.right_bumper){
                carouselDirection = !carouselDirection;
                motors[5].motor.setPower(0.4);
            }
           else if (gamepad2.left_bumper) {

                motors[5].motor.setPower(-0.4);
            } else {
                motors[5].motor.setPower(0);
            }

            //Claw
            if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                leftArm.setPosition(1);
                rightArm.setPosition(1);
            } else if (!gamepad2.dpad_up && gamepad2.dpad_down) {
                leftArm.setPosition(0.85);
                rightArm.setPosition(0.85);
            }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("FRONT LEFT Motor",   motors[0].motor.getVelocity());
        telemetry.addData("FRONT RIGHT Motor",  motors[1].motor.getVelocity());
        telemetry.addData("BACK LEFT Motor",    motors[2].motor.getVelocity());
        telemetry.addData("BACK RIGHT Motor",   motors[3].motor.getVelocity());
        telemetry.addData("Cascade",            motors[4].motor.getCurrentPosition());
        telemetry.addData("Carousel",           motors[5].motor.getVelocity());
        telemetry.addData("left arm position: ", leftArm.getPosition());
        telemetry.addData("right arm position: ", rightArm.getPosition());
        telemetry.addData("a: ", velocityMultiplier);


            telemetry.update();

    }
}