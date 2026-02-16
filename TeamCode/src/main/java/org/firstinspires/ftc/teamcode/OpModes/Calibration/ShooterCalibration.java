package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class ShooterCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Shooter shooter = new Shooter();
        Pitch pitch = new Pitch();
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while(opModeIsActive())
        {
            if(gamepad1.a){shooter.setState(Shooter.State.Close);pitch.setState(Pitch.State.CLOSE);}
            if(gamepad1.x){shooter.setState(Shooter.State.Far); pitch.setState(Pitch.State.FAR);}

            if(gamepad1.y)shooter.setState(Shooter.State.Pause);

            shooter.update();
            pitch.update();

            telemetry.addData("velocity" , shooter.encoder.getVelocity());
            telemetry.addData("targetVelocity" , shooter.state.targetVelocity);
            telemetry.update();
        }

    }

}
