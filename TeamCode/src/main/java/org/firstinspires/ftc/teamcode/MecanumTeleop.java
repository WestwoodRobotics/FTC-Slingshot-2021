package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),
            new CustomMotor("leftBack"),
            new CustomMotor("rightFront"),
            new CustomMotor("rightBack"),
            new CustomMotor("cascadeMotor"),
            new CustomMotor("carouselMotor")
    };

    CustomDistanceSensor[] sensors = {
            new CustomDistanceSensor("front sensor"),
            new CustomDistanceSensor("back sensor"),
            new CustomDistanceSensor("right sensor"),
            new CustomDistanceSensor("left sensor")

    };

    Servo           leftArm     = null;
    Servo           rightArm    = null;
    DistanceSensor  front       = null;
    DistanceSensor  right       = null;
    DistanceSensor  left        = null;
    DistanceSensor  back        = null;

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
        sensors[0].sensor           = hardwareMap.get(DistanceSensor.class,     "front Dist");
        sensors[1].sensor           = hardwareMap.get(DistanceSensor.class,     "right Dist");
        sensors[2].sensor           = hardwareMap.get(DistanceSensor.class,     "left Dist");
        sensors[3].sensor           = hardwareMap.get(DistanceSensor.class,     "back Dist");

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
        motors[4].motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motors[4].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        // setting pid




            //drivetrain
            double leftPower;
            double rightPower;
            double  y =  -gamepad1.left_stick_y;    //forward(no orientation change)
            double  x =   gamepad1.left_stick_x;    //left right strafe(no orientation change)
            double rx =   gamepad1.right_stick_x;   //rotate left right(orientation change)

            double[] velocity = {
                    y + x + rx, // left front
                    y - x - rx, // right front
                    y - x + rx, // left                                                                                                   back
                    y + x - rx // left back
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
                motors[i].motor.setVelocity(velocity[i]*5000);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FRONT LEFT Motor",   motors[0].motor.getVelocity());
            telemetry.addData("FRONT RIGHT Motor",  motors[1].motor.getVelocity());
            telemetry.addData("BACK LEFT Motor",    motors[2].motor.getVelocity());
            telemetry.addData("BACK RIGHT Motor",   motors[3].motor.getVelocity());
            telemetry.addData("Cascade",            motors[4].motor.getVelocity());
            telemetry.addData("Carousel",           motors[5].motor.getVelocity());

/*
            //Cascade
            if(gamepad1.a){
                motors[4].motor.setPower(1);
            } else if(gamepad1.b) {
                motors[4].motor.setPower(-1);
            }
*/

            //Carousel
            if (gamepad1.left_bumper) {
                motors[5].motor.setPower(0.7);
            } else {
                motors[5].motor.setPower(0.0);
            }

            //Claw
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                leftArm.setPosition(0.5);
                rightArm.setPosition(0.5);
            } else if (!gamepad1.dpad_up && gamepad1.dpad_down) {
                leftArm.setPosition(0);
                rightArm.setPosition(0);
            }

            /** NEW CODE BELOW **/
        @TeleOp
        class DistanceTest extends LinearOpMode {

            @Override
            public void runOpMode() {

                waitForStart();
                while (opModeIsActive()) {
                    if (sensors[0].sensor.getDistance(DistanceUnit.MM) < 100) {
                        motors[0].motor.setPower(0.3);
                        motors[1].motor.setPower(0.3);
                        motors[2].motor.setPower(0.3);
                        motors[3].motor.setPower(0.3);
                    } else {
                        motors[0].motor.setPower(0);
                        motors[1].motor.setPower(0);
                        motors[2].motor.setPower(0);
                        motors[3].motor.setPower(0);
                    }
                }
            }
        }
            /** NEW CODE ABOVE **/


            telemetry.addData("Cascade Motor power: ",     motors[4].motor.getVelocity());
            telemetry.addData("Cascade Motor position: ",  motors[4].motor.getCurrentPosition());
            telemetry.addData("Carousel Motor power: ",    motors[5].motor.getVelocity());
            telemetry.addData("Carousel Motor position: ", motors[5].motor.getCurrentPosition());
            telemetry.update();

    }
}