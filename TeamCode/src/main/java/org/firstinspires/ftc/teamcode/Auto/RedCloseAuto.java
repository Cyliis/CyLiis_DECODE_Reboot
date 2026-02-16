package org.firstinspires.ftc.teamcode.Auto;

import android.media.AudioRecord;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Sorter;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Turret;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

public class RedCloseAuto {

    public static Pose2D shootPosition = new Pose2D(-1450, 460, -Math.PI/2);
    public static Pose2D[] beforeTakeStack2Position = {
            new Pose2D(-1910, 100, -Math.PI/2) ,
            new Pose2D(-2450, 100, -Math.PI/2)
    };

    public static Pose2D[] takeStack2Position = {
            new Pose2D(-1910, -760, -Math.PI/2),
            new Pose2D(-2450, -760, -Math.PI/2)
    };

    public static Pose2D beforeShootAfterCollectingPosition = new Pose2D(-1930, -600, -Math.PI/2+0.3);
    public static Pose2D beforeIntakeWhileOpenGatePosition = new Pose2D(-1790, -660, -Math.PI/2);
    public static Pose2D intakeWhileOpenGatePosition = new Pose2D(-1860, -710, -Math.PI/2-0.55);

    public Pose2D beforeTakeStack1Position = new Pose2D(-1270 , 450 , -Math.PI/2  );
    public Pose2D takeStack1Position = new Pose2D(-1270 , -620 , -Math.PI/2);

    Node beforeShootAfterCollecting, shoot,  beforeTakeStack2 ,takeStack2, beforeTakeStack1, takeStack1, beforeIntakeWhileOpenGate, intakeWhileOpenGate, openGate;
    public Node currentNode;

    MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;

    ElapsedTime failSafeIntake = new ElapsedTime();

    public void init(HardwareMap hardwareMap)
    {
        Odo.init(hardwareMap);
        Hardware.init(hardwareMap);
        driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        intake = new Intake();
        intake.setState(Intake.State.READY_FOR_TRANSFER);
        Sorter.state=Sorter.State.GoingReadyForTransfer;
        outtake = new Outtake(Turret.State.RED);

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

        currentNode=shoot;

        shoot.addConditions(
                ()->{
                    if(intake.state== Intake.State.INTAKE && !intake.sorter.isMoving())intake.setState(Intake.State.PAUSE);
                    if(Sorter.state != Sorter.State.BeforeReset && Sorter.state != Sorter.State.FastShooting && Sorter.state!=Sorter.State.ReadyForTransfer) Sorter.state = Sorter.State.GoingReadyForTransfer;
                    driveTrain.setTargetPosition(shootPosition);
                    outtake.setState(Outtake.State.SHOOT);
                    if(driveTrain.inPosition(25, 25, 0.13) && outtake.isReady() && Math.abs(Odo.odo.getVelY(DistanceUnit.MM))<=10 && Math.abs(Odo.odo.getVelX(DistanceUnit.MM))<=10)
                    {
                        intake.setState(Intake.State.SHOOTING);
                        if(Pitch.rapidShootTimer.seconds() > Pitch.rapidShootTime)Pitch.rapidShootTimer.reset();
                    }
                }
                ,
                ()-> {
                    if(Sorter.state == Sorter.State.BeforeReset)
                    {
                        return true;
                    }
                    return false;

                }
                ,
                new Node[]{beforeTakeStack2, beforeIntakeWhileOpenGate, beforeTakeStack1, beforeTakeStack2}
        );
        beforeTakeStack2.addConditions(
                ()->{
                    outtake.setState(Outtake.State.PAUSE);
                    driveTrain.setTargetPosition(beforeTakeStack2Position[Math.min(beforeTakeStack2.index , beforeTakeStack2Position.length-1) ] );
                }
                ,
                ()->{
                    return driveTrain.inPosition(35 , 35 , 0.1) && intake.state== Intake.State.PAUSE;
                }
                ,
                new Node[]{takeStack2}
        );

        takeStack2.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeStack2Position[Math.min(takeStack2.index , takeStack2Position.length-1) ] );
                    intake.setState(Intake.State.INTAKE);
                    outtake.setState(Outtake.State.PAUSE);
                }
                ,
                ()->{
                    return driveTrain.inPosition(35 , 35 , 0.1);
                }
                ,
                new Node[]{beforeShootAfterCollecting}
        );

        beforeShootAfterCollecting.addConditions(
                ()->{
                    driveTrain.setTargetPosition(beforeShootAfterCollectingPosition);
                    outtake.setState(Outtake.State.SHOOT);
                    if(Sorter.state==Sorter.State.ReadyForTransfer)
                        intake.setState(Intake.State.REVERSE);
                }
                ,
                ()->{
                    return driveTrain.inPosition(35, 35 , 0.1);
                }
                ,
                new Node[]{shoot}
        );

        beforeIntakeWhileOpenGate.addConditions(
                ()->{
                    driveTrain.setTargetPosition(beforeIntakeWhileOpenGatePosition);
                    outtake.setState(Outtake.State.PAUSE);
                }
                ,
                ()->{
                    failSafeIntake.reset();
                    return driveTrain.inPosition(60, 60, 0.15);
                }
                ,
                new Node[]{intakeWhileOpenGate}
        );

        intakeWhileOpenGate.addConditions(
                ()->{
                    driveTrain.setTargetPosition(intakeWhileOpenGatePosition);
                    intake.setState(Intake.State.INTAKE);
                    outtake.setState(Outtake.State.PAUSE);

                    if(!driveTrain.inPosition(80, 80, 0.3))failSafeIntake.reset();
                }
                ,
                ()->{
                    return Sorter.state==Sorter.State.GoingReadyForTransfer || Sorter.state==Sorter.State.ReadyForTransfer || failSafeIntake.seconds()>3.5;
                }
                ,
                new Node[]{beforeShootAfterCollecting}
        );

        beforeTakeStack1.addConditions(
                ()->{
                    driveTrain.setTargetPosition(beforeTakeStack1Position);
                    outtake.setState(Outtake.State.PAUSE);
                    failSafeIntake.reset();
                }
                ,
                ()->{
                    failSafeIntake.reset();
                    return driveTrain.inPosition(40 , 40 , 0.1) && intake.state== Intake.State.PAUSE;
                }
                ,
                new Node[]{takeStack1}
        );

        takeStack1.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeStack1Position);
                    intake.setState(Intake.State.INTAKE);
                    outtake.setState(Outtake.State.PAUSE);
                }
                ,
                ()->{
                    return driveTrain.inPosition(100 , 100 , 0.2) && failSafeIntake.seconds()>1.7;
                }
                ,
                new Node[]{shoot}
        );

    }

    public void run()
    {

        currentNode.run();

        Odo.update();
        driveTrain.update();
        intake.update();
        outtake.update();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];
    }

}
