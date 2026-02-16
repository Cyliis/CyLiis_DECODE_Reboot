package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.BlueCloseAuto;
import org.firstinspires.ftc.teamcode.Modules.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_Ca_Merge_BLUE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        BlueCloseAuto nodes = new BlueCloseAuto();
        nodes.init(hardwareMap);

        while (opModeInInit())
        {
            nodes.intake.update();

            telemetry.addData("x" , Odo.predictedX);
            telemetry.addData("y" , Odo.predictedY);
            telemetry.addData("IntakeState" , nodes.intake.state);
            telemetry.addData("ShooterState" ,nodes.outtake.state);
            telemetry.addData("SorterState" , Sorter.state);
            telemetry.update();
        }

        waitForStart();
        while(opModeIsActive())
        {
            nodes.run();

            telemetry.addData("x" , Odo.predictedX);
            telemetry.addData("y" , Odo.predictedY);
            telemetry.addData("IntakeState" , nodes.intake.state);
            telemetry.addData("ShooterState" ,nodes.outtake.state);
            telemetry.addData("SorterState" , Sorter.state);
            telemetry.addData("outtakeReady" , nodes.outtake.isReady());
            telemetry.addData("node" , nodes.currentNode.name);
            telemetry.addData("OuttakeNumber" , nodes.outtake.position);
            telemetry.update();
        }
    }
}
