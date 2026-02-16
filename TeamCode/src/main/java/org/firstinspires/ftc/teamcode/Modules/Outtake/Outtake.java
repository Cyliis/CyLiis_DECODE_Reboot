package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Config
public class Outtake {

    public enum State{
        PAUSE, SHOOT;
    }
    public State state=State.PAUSE;
    enum ShootState{
        CLOSE, FAR, BROKEN;
    }
    ShootState shootState=ShootState.CLOSE;
    public static double[] pitches={
            0.05,//0
            0.1,//1
            0.1,//2
            0.15,//3
            0.17,//4
            0.2,//5
            0.21,//6
            0.255,//7
            0.28,//8
            0.3,//9
            0.315,//10
            0.33,//11
            0.335,//12
            0.32,//13 trebuie
            0.32,//14
            0.32,//15
            0.35,//16
            0.31,//17
            0.32,//18
            0.385,//19
            0.385,//20
            0.365,//21
            0.375,//22
            0.37,//23
            0.375,//24
            0.375,//25
            0.36,0.36,0.36,0.36,
            0.33,0.33,0.33,0.33,
            0.37,0.37,0.37,0.37,
            0.37,0.37,0.37,0.37,
            0.37,0.37,0.37,0.37,
            0.37,0.37,0.37,0.37,
            0.37,0.37,0.37,0.37
    };
    public static double[] targetVelocity={
            1300,//0
            1300,//1
            1300,//2
            1340,//3
            1350,//4
            1380,//5
            1420,//6
            1440,//7
            1540,//8
            1580,//9
            1580,//10
            1560,//11
            1580,//12
            1600,//13
            1640,//14
            1680,//15
            1680,//16
            1700,//17
            1740,//18
            1850,//19
            1860,//20
            1900,//21
            1940,//22
            1960,//23
            2000,//24
            2040,//25
            2040,2040,2040,2040,
            1980,1980,1980,1980,
            1900,1900,1900,1900,
            1900,1900,1900,1900,
            1900,1900,1900,1900,
            1900,1900,1900,1900,
            1900,1900,1900,1900
    };

    Pitch pitch;
    public Shooter shooter;
    public Turret turret;
    public double distance=0;
    Turret.State statee;
    public int position;

    public Outtake(Turret.State state)
    {
        statee=state;
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
                //turret.state= Turret.State.Middle;
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
                        turret.setState(statee);
                        shooter.setState(Shooter.State.Far);
                        pitch.setState(Pitch.State.FAR);
                        break;
                    case CLOSE:
                        turret.setState(statee);
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

        distance = Math.sqrt( ( targetX- Odo.getX() ) * ( targetX- Odo.getX() ) + ( targetY- Odo.getY() ) * ( targetY- Odo.getY() ) );

        if(distance<1000)distance=1000;
        position=(int)(distance/100)-10;

        if(position+1>=pitches.length){position=pitches.length-3;distance=(position+10)*100+100;}

        Pitch.State.CLOSE.position= pitches[position]*( Math.abs(100-Math.abs(distance-(position+10)*100)))/100 + pitches[position+1]*( Math.abs(100-Math.abs(distance-(position+11)*100)))/100;
        Shooter.State.Close.targetVelocity= targetVelocity[position]*( Math.abs(100-Math.abs(distance-(position+10)*100)))/100 + targetVelocity[position+1]*( Math.abs(100-Math.abs(distance-(position+11)*100)))/100;
        Shooter.State.Pause.targetVelocity=Shooter.State.Close.targetVelocity;
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
