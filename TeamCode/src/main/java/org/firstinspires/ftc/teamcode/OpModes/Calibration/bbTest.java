package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class bbTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        DigitalChannel bb=Hardware.bb;

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("bb" , bb.getState());
            telemetry.update();
        }
    }
}
