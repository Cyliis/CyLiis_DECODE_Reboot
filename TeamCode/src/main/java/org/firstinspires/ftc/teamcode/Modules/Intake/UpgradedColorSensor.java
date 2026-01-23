package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class UpgradedColorSensor {

    public Sorter.Ball state= Sorter.Ball.NOTHING;

    public ColorSensor colorSensor;

    public float red=0 , blue=0 , green=0;

    public UpgradedColorSensor()
    {
        colorSensor= Hardware.colorSensor;

    }
    private void updateColor()
    {
        red=colorSensor.red();
        green=colorSensor.green();
        blue=colorSensor.blue();
    }


    public float distance(float r1 , float g1  , float b1 , float r2 , float g2 , float b2)
    {
        return (float)Math.sqrt( (r1-r2)*(r1-r2) + (b1-b2)*(b1-b2) + (g1-g2)*(g1-g2));
    }

    private void updateState()
    {
        double purpleball = distance(red , green , blue , 180 ,0 , 180);
        double greenball = distance(red , green , blue , 0 , 255 , 0);

        if(purpleball <= greenball) state = Sorter.Ball.PURPLE;
        else state = Sorter.Ball.GREEN;
    }


    public void update()
    {
        updateColor();
        updateState();
    }
}
