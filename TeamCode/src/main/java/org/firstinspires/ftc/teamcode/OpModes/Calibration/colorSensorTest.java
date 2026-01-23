package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake.UpgradedColorSensor;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class colorSensorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        UpgradedColorSensor colorSensor=new UpgradedColorSensor();

        waitForStart();

        while(opModeIsActive())
        {

            colorSensor.update();

            telemetry.addData("color" , colorSensor.state);
            telemetry.update();
        }
    }
}
