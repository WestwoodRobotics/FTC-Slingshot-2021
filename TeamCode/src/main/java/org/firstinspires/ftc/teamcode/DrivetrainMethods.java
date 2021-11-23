package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class DrivetrainMethods {
    CustomMotor[] motors = null;
    double[][] velocityTables = {
            {
                1,1,1,1
            },
            {
                1,-1,-1,1
            },
            {
                1,0,0,1
            },
            {
                0,1,1,0
            },
            {
                1,-1,1,-1
            }


    };
    public DrivetrainMethods(CustomMotor[] dcMotors){
        motors = dcMotors;
    }

    public void move(double x, double y, double rx, double angle, double currentAngle){
        double [] angleCorrection = {

        };
        double[] velocity = {
                y + x + rx, // left front
                y - x - rx, // right front
                y - x + rx, // left                                                                                                   back
                y + x - rx // left back
        };
        for (int i = 0; i < 4; i++) {
            motors[i].motor.setVelocity(velocity[i]);
        }

    }
    public double[] correction(double angleWanted, double currentAngle){
        if(angleWanted == currentAngle){
            return new double[]{0,0,0,0};
        }
        if(angleWanted < currentAngle ){
            return new double[]{
                    -1,
                    1,
                    -1,
                    1

            };
        }
        else{
            return  new double[]{
                    1,
                    -1,
                    1,
                    -1
            };
        }

    }
}
