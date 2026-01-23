package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class OdoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Odo.init(hardwareMap);
        Odo.calibrate();

        waitForStart();

        while(opModeIsActive())
        {
            Odo.update();
            telemetry.addData("x" , Odo.getX());
            telemetry.addData("y" , Odo.getY());
            telemetry.addData("heading" , Odo.getHeading());
            telemetry.update();
        }
    }
}
