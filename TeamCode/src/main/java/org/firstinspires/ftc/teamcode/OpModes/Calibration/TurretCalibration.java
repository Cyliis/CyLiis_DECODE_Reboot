package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp
public class TurretCalibration extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap);
        Turret turret=new Turret(Turret.State.Middle);

        waitForStart();

        while(opModeIsActive())
        {

            turret.update();
            Odo.update();

            telemetry.addData("angle" , turret.angle);
            telemetry.update();
        }
    }
}
