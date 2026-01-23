package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Intake.Latch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class LatchCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Latch latch=new Latch(Latch.State.GOING_FREE);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)latch.setState(Latch.State.GOING_FREE);
            if(gamepad1.x)latch.setState(Latch.State.GOING_TRANSFER);

            telemetry.addData("isMoving" , latch.isMoving());
            telemetry.update();
        }
    }
}
