package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class PinPointCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {



        Odo.initForAuto(hardwareMap);
        Odo.calibrate();
        ElapsedTime timer=new ElapsedTime();

        while(opModeInInit())
        {
            if(timer.seconds()>1)break;

        }
    }
}
