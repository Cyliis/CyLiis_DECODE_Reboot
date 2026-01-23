package org.firstinspires.ftc.teamcode.Modules.Outtake;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

public class Outtake {

    public enum State{
        PAUSE, SHOOT;
    }
    public State state=State.PAUSE;
    enum ShootState{
        CLOSE, FAR, BROKEN;
    }
    ShootState shootState=ShootState.CLOSE;


    Pitch pitch;
    public Shooter shooter;
    public Turret turret;



    public Outtake(Turret.State state)
    {
        turret = new Turret(state);
        shooter = new Shooter();
        pitch = new Pitch();
    }

    public boolean isReady()
    {
        return state==State.PAUSE || shooter.isVelocityOptimal();
    }

    public void setState(State state)
    {
        this.state=state;
    }

    private void updateStates()
    {
        switch (state)
        {
            case PAUSE:
                shooter.setState(Shooter.State.Pause);
                break;
            case SHOOT:

                switch (shootState)
                {
                    case BROKEN:
                        turret.state= Turret.State.Middle;
                        shooter.setState(Shooter.State.Close);
                        pitch.setState(Pitch.State.CLOSE);
                        break;
                    case FAR:
                        shooter.setState(Shooter.State.Far);
                        pitch.setState(Pitch.State.FAR);
                        break;
                    case CLOSE:
                        shooter.setState(Shooter.State.Close);
                        pitch.setState(Pitch.State.CLOSE);
                        break;
                }
            break;
        }
    }

    private void updatePositions()
    {
        if(shootState==ShootState.BROKEN)return;

        double targetX = turret.state.targetX;
        double targetY = turret.state.targetY;

        double distance = Math.sqrt( ( targetX- Odo.getX() ) * ( targetX- Odo.getX() ) + ( targetY- Odo.getY() ) * ( targetY- Odo.getY() ) );

        if(distance>1000)shootState=ShootState.FAR;
        else shootState=ShootState.CLOSE;
    }

    public void update()
    {
        updateStates();
        updatePositions();

        turret.update();
        shooter.update();
        pitch.update();
    }
}
