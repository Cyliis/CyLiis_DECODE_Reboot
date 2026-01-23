package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Math._33VtoRadians;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class SpindexerEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);
        AnalogInput encoder= Hardware.analogInput;

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("angle" , _33VtoRadians.convert(encoder.getVoltage()));
            telemetry.update();
        }
    }
}
