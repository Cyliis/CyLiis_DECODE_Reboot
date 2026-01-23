package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorIMUOrthogonal;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        Odo.init(hardwareMap);
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Outtake outtake=new Outtake(Turret.State.BLUE);
        Intake intake=new Intake();
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.ps)Odo.reset();
            driveTrain.setMode(MecanumDriveTrain.State.DRIVE);
            double X=gamepad1.left_stick_x;
            double Y=gamepad1.left_stick_y;
            double rotation=(gamepad1.right_trigger-gamepad1.left_trigger);

            double heading =Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( y , x , rotation );

            if(gamepad1.right_bumper && intake.state!=Intake.State.SHOOTING)intake.setState(Intake.State.INTAKE);
            else if(gamepad1.left_bumper && intake.state!=Intake.State.SHOOTING)intake.setState(Intake.State.REVERSE);
            else if(intake.state!= Intake.State.SHOOTING)intake.setState(Intake.State.PAUSE);

            if(Sorter.state==Sorter.State.ReadyForTransfer)
            {
                outtake.setState(Outtake.State.SHOOT);
            }

            if(outtake.isReady() && outtake.state== Outtake.State.SHOOT && Sorter.state==Sorter.State.ReadyForTransfer && gamepad1.a)intake.setState(Intake.State.SHOOTING);
            if(Sorter.state== Sorter.State.Ball1)outtake.setState(Outtake.State.PAUSE);


            if(gamepad1.b)outtake.setState(Outtake.State.PAUSE);

            outtake.update();
            intake.update();
            Odo.update();

            telemetry.addData("intakeState" , intake.state);
            telemetry.addData("sorterState" , Sorter.state);
            telemetry.addData("outtakeState" , outtake.state);
            telemetry.addData("shooter" , outtake.shooter.state);
            telemetry.addData("X" , Odo.getX());
            telemetry.addData("Y" , Odo.getY());

            telemetry.addData("Ball1" , intake.sorter.ball1);
            telemetry.addData("Ball2" , intake.sorter.ball2);
            telemetry.addData("Ball3" , intake.sorter.ball3);

            telemetry.addData("targetPositionTurret" , Turret.targetAngle);

            telemetry.addData("VelocityShooter" , outtake.shooter.encoder.getVelocity());

            timer.reset();
            telemetry.update();
        }
    }
}