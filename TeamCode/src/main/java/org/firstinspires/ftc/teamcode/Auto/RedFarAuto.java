package org.firstinspires.ftc.teamcode.Auto;

import android.media.AudioRecord;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

public class RedFarAuto {

    public static Pose2D shootPosition = new Pose2D(-3100, 200, -Math.PI/2);

    public static Pose2D takeStackPosition = new Pose2D(-3180, -700, -Math.PI/2);
    public Pose2D beforeTakeFromGate = new Pose2D(-3000 , -700 , -Math.PI/2-0.5);
    public Pose2D takeFRomGate = new Pose2D(-2500 , -700 , -Math.PI/2-1.2);

    Node shoot, takeStack, beforeTakeArtefacts, takeArtefacts;
    public Node currentNode;

    MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;

    ElapsedTime timer = new ElapsedTime();


    public void init(HardwareMap hardwareMap)
    {
        Odo.init(hardwareMap);
        Hardware.init(hardwareMap);
        driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        intake = new Intake();
        intake.setState(Intake.State.READY_FOR_TRANSFER);
        Sorter.state=Sorter.State.GoingReadyForTransfer;
        outtake = new Outtake(Turret.State.RED);

        shoot = new Node("shoot");
        takeStack = new Node("takeStack");
        beforeTakeArtefacts = new Node("beforeTakeArtefacts");
        takeArtefacts = new Node("takeArtefacts");

        timer.startTime();


        Odo.odo.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.MM , -3200 , 0 , AngleUnit.RADIANS , Math.PI));

        shoot.addConditions(
                ()->{
                    if(intake.state== Intake.State.INTAKE && !intake.sorter.isMoving())intake.setState(Intake.State.PAUSE);
                    if(Sorter.state != Sorter.State.BeforeReset && Sorter.state != Sorter.State.FastShooting && Sorter.state!=Sorter.State.ReadyForTransfer) Sorter.state = Sorter.State.GoingReadyForTransfer;
                    driveTrain.setTargetPosition(shootPosition);
                    outtake.setState(Outtake.State.SHOOT);
                    if(driveTrain.inPosition(25, 25, 0.1) && outtake.isReady() && Math.abs(Odo.odo.getVelY(DistanceUnit.MM))<=5 && Math.abs(Odo.odo.getVelX(DistanceUnit.MM))<=5)
                    {
                        intake.setState(Intake.State.SHOOTING);
                        if(Pitch.rapidShootTimer.seconds() > Pitch.rapidShootTime)Pitch.rapidShootTimer.reset();
                    }
                }
                ,
                ()->{
                    if(Sorter.state == Sorter.State.BeforeReset)
                    {
                        return true;
                    }
                    return false;
                }
                , new Node[]{takeStack , beforeTakeArtefacts}
        );

        takeStack.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeStackPosition);

                    if(Sorter.state==Sorter.State.GoingBall1)
                        intake.setState(Intake.State.INTAKE);
                    outtake.setState(Outtake.State.PAUSE);

                    if(!driveTrain.inPosition(25 , 25 , 0.1))timer.reset();
                }
                ,
                ()->{
                    return Sorter.state==Sorter.State.GoingReadyForTransfer || Sorter.state==Sorter.State.ReadyForTransfer || timer.seconds()>3.5;
                }
                , new Node[]{shoot}
        );

        beforeTakeArtefacts.addConditions(
                ()->{
                    driveTrain.setTargetPosition(beforeTakeFromGate);
                    if(Sorter.state==Sorter.State.GoingBall1)
                        intake.setState(Intake.State.INTAKE);

                    timer.reset();
                }
                ,
                ()->{
                    return driveTrain.inPosition(25 , 25 , 0.1);
                }
                , new Node[]{takeArtefacts}
        );

        takeArtefacts.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeFRomGate);
                    if(Sorter.state==Sorter.State.GoingBall1)
                        intake.setState(Intake.State.INTAKE);
                }
                ,
                ()->{
                    return Sorter.state==Sorter.State.GoingReadyForTransfer || Sorter.state==Sorter.State.ReadyForTransfer || driveTrain.inPosition(15 , 15 ,0.1) || timer.seconds()>5;
                }
                , new Node[]{shoot}
        );

        currentNode=shoot;
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
