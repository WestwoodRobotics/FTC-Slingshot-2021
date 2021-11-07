package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;


public class CustomDistanceSensor {
    public String name;
    public DistanceSensor sensor = null;
    //NORMAL PID
    //NO CONTROL
    public CustomDistanceSensor(String Name) {
        name = Name;
    }
    //FEEDFORWARD CONTROL

}