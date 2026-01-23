package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Math._33VtoRadians;
import org.firstinspires.ftc.teamcode.Modules.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class SpindexerTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Sorter sorter=new Sorter(Sorter.State.Ball1);

        waitForStart();

        while(opModeIsActive())
        {
            sorter.update();
            telemetry.addData("currentPosition" , _33VtoRadians.convert(Hardware.analogInput.getVoltage()));
            telemetry.addData("targetPosiiton" , (Sorter.state.position+Sorter.zeroAngle)%(Math.PI*2));
            telemetry.addData("error" , Sorter.error);
            telemetry.update();
        }
    }
}
