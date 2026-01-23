package org.firstinspires.ftc.teamcode.Auto;

import android.media.AudioRecord;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class BlueCloseAuto {

    public static Pose2D shootPosition = new Pose2D(0, 0, 0);
    public static Pose2D beforeTakeStack2Position = new Pose2D(0, 0, 0);
    public static Pose2D takeStack2Position = new Pose2D(0, 0, 0);
    public static Pose2D beforeShootAfterCollectingPosition = new Pose2D(0, 0, 0);
    public static Pose2D beforeIntakeWhileOpenGatePosition = new Pose2D(0, 0, 0);
    public static Pose2D intakeWhileOpenGatePosition = new Pose2D(0, 0, 0);

    Node beforeShootAfterCollecting, shoot,  beforeTakeStack2 ,takeStack2, beforeTakeStack1, takeStack1, beforeIntakeWhileOpenGate, intakeWhileOpenGate, openGate;
    public Node currentNode;

    MecanumDriveTrain driveTrain;
    Intake intake;
    Outtake outtake;

    ElapsedTime failSafeIntake = new ElapsedTime();

    public void init(HardwareMap hardwareMap)
    {
        Odo.init(hardwareMap);
        Hardware.init(hardwareMap);
        driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        intake = new Intake();
        intake.setState(Intake.State.READY_FOR_TRANSFER);
        outtake = new Outtake(Turret.State.BLUE);

        beforeShootAfterCollecting = new Node("beforeShootAfterCollecting");
        shoot = new Node("shoot");
        takeStack1 = new Node("takeStack1");
        beforeTakeStack1 = new Node("beforeTakeStack1");
        beforeTakeStack2 = new Node("beforeTakeStack2");
        takeStack2 = new Node("takeStack2");
        beforeIntakeWhileOpenGate = new Node("beforeIntakeWhileOpenGate");
        intakeWhileOpenGate = new Node("intakeWhileOpenGate");
        openGate = new Node("openGate");
        failSafeIntake.reset();

        shoot.addConditions(
                ()->{
                    if(Sorter.state != Sorter.State.BeforeReset && Sorter.state != Sorter.State.FastShooting) Sorter.state = Sorter.State.GoingReadyForTransfer;
                    driveTrain.setTargetPosition(shootPosition);
                    outtake.setState(Outtake.State.SHOOT);
                    if(driveTrain.inPosition(25, 25, 0.1) && outtake.isReady() )
                    {
                        intake.setState(Intake.State.SHOOTING);
                    }
                }
                ,
                ()->{
                    return Sorter.state == Sorter.State.BeforeReset;
                }
                ,
                new Node[]{beforeTakeStack2, beforeIntakeWhileOpenGate, beforeIntakeWhileOpenGate, beforeIntakeWhileOpenGate, takeStack1}
        );
        beforeTakeStack2.addConditions(
                ()->{
                    intake.setState(Intake.State.PAUSE);
                    outtake.setState(Outtake.State.PAUSE);
                    driveTrain.setTargetPosition(beforeTakeStack2Position);
                }
                ,
                ()->{
                    return driveTrain.inPosition(25 , 25 , 0.1);
                }
                ,
                new Node[]{takeStack2}
        );

        takeStack2.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeStack2Position);
                    intake.setState(Intake.State.INTAKE);
                    outtake.setState(Outtake.State.PAUSE);
                }
                ,
                ()->{
                    return driveTrain.inPosition(25 , 25 , 0.1);
                }
                ,
                new Node[]{beforeShootAfterCollecting}
        );

        beforeShootAfterCollecting.addConditions(
                ()->{
                    driveTrain.setTargetPosition(beforeShootAfterCollectingPosition);
                    outtake.setState(Outtake.State.SHOOT);
                    intake.setState(Intake.State.INTAKE);
                }
                ,
                ()->{
                    return driveTrain.inPosition(25, 25 , 0.1);
                }
                ,
                new Node[]{shoot}
        );

        beforeIntakeWhileOpenGate.addConditions(
                ()->{
                    driveTrain.setTargetPosition(beforeIntakeWhileOpenGatePosition);
                    intake.setState(Intake.State.PAUSE);
                    outtake.setState(Outtake.State.PAUSE);
                }
                ,
                ()->{
                    return driveTrain.inPosition();
                }
                ,
                new Node[]{intakeWhileOpenGate}
        );

        intakeWhileOpenGate.addConditions(
                ()->{
                    driveTrain.setTargetPosition(intakeWhileOpenGatePosition);
                    intake.setState(Intake.State.INTAKE);
                    outtake.setState(Outtake.State.PAUSE);

                    if(!driveTrain.inPosition(25, 25, 0.1))failSafeIntake.reset();
                }
                ,
                ()->{
                    return Sorter.state==Sorter.State.GoingReadyForTransfer || Sorter.state==Sorter.State.ReadyForTransfer || failSafeIntake.seconds()>4;
                }
                ,
                new Node[]{beforeShootAfterCollecting}
        );

    }
}
