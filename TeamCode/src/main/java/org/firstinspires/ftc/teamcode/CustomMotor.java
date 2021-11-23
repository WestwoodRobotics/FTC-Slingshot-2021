package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class CustomMotor {
    public String name;
    public DcMotorEx motor = null;
    //NORMAL PID
    //NO CONTROL
    public CustomMotor(String Name, PIDCoefficients pidCoefficients) {
        name = Name;
    }

    public CustomMotor(com.qualcomm.robotcore.hardware.PIDCoefficients pidCoefficients) {
    }

    public CustomMotor(String leftFront, com.qualcomm.robotcore.hardware.PIDCoefficients pidCoefficients) {
    }
    //FEEDFORWARD CONTROL

}