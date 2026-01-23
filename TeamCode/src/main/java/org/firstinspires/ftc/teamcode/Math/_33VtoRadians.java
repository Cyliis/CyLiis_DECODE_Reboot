package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot.Hardware;


public class _33VtoRadians {

    public static double convert(double voltage)
    {
        return (voltage / 3.3) * 2.0 * Math.PI;
    }
}
