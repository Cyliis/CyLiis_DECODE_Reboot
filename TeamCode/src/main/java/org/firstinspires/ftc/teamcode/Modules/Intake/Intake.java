package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Pitch;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Intake {

    public enum State{
        INTAKE, REVERSE, PAUSE, READY_FOR_TRANSFER, SHOOTING;
    }
    public State state;

    DigitalChannel bb;
    ActiveIntake intake;
    public Latch latch;
    public Sorter sorter;
    UpgradedColorSensor colorSensor;

    public Intake()
    {
        intake = new ActiveIntake();
        latch = new Latch(Latch.State.FREE);
        sorter = new Sorter(Sorter.State.GoingBall1);
        colorSensor = new UpgradedColorSensor();
        bb = Hardware.bb;
    }

    public void setState(State state)
    {
        this.state = state;
    }

    private void updateState()
    {
        switch (state)
        {
            case READY_FOR_TRANSFER:
                break;
            case PAUSE:
                intake.setState(ActiveIntake.State.PAUSE);
                break;
            case INTAKE:
                if(!sorter.isMoving() && !bb.getState())
                {
                    colorSensor.update();
                    switch (Sorter.state)
                    {
                        case Ball1:
                        case GoingBall1:

                            sorter.ball1 = colorSensor.state;

                            break;

                        case Ball2:
                        case GoingBall2:

                            sorter.ball2 = colorSensor.state;

                            break;

                        case Ball3:
                        case GoingBall3:

                            sorter.ball3 = colorSensor.state;


                            break;
                    }
                    sorter.goNext();
                }


                intake.setState(ActiveIntake.State.INTAKE);

                if(Sorter.state!= Sorter.State.ReadyForTransfer)
                latch.setState(Latch.State.GOING_FREE);
                else latch.setState(Latch.State.GOING_TRANSFER);

                break;
            case REVERSE:
                intake.setState(ActiveIntake.State.REVERSE);
                break;

            case SHOOTING:
                sorter.shoot();
                intake.setState(ActiveIntake.State.TRANSFER);
                if(Sorter.state== Sorter.State.BeforeReset && !sorter.isMoving())latch.setState(Latch.State.GOING_FREE);
                if(latch.state== Latch.State.FREE){state=State.PAUSE;Sorter.state= Sorter.State.GoingBall1;}
                break;


        }
    }

    public void update()
    {

        if(Sorter.state==Sorter.State.ReadyForTransfer)latch.setState(Latch.State.GOING_TRANSFER);
        updateState();

        intake.update();
        latch.update();
        sorter.update();
    }
}
