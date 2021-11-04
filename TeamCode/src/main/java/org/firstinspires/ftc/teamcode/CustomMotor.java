package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;


public class CustomMotor {
    public String name;
    public DcMotorEx motor = null;
    //NORMAL PID
    //NO CONTROL
    public CustomMotor(String Name) {
        name = Name;
    }
    //FEEDFORWARD CONTROL

}