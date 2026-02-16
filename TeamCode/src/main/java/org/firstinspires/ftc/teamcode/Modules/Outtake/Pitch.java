package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Pitch {

    public static double farPosition=0.25, closePosition=0.25;


    public static double K=-0.00002;
    public static double correction = 0;
    public static double rapidShootTime = 0.8;

    public static ElapsedTime rapidShootTimer = new ElapsedTime();

    public enum State{
        FAR(farPosition), CLOSE(closePosition);

        public double position;
        State(double position)
        {
            this.position=position;
        }
    }State state=State.CLOSE;

    Servo servo;

    public Pitch()
    {
        servo= Hardware.ssh5;
    }

    public void setState(State state)
    {
        this.state=state;
    }

    private void updatePositions()
    {
        //State.FAR.position=farPosition;
        //State.CLOSE.position=closePosition + (rapidShootTimer.seconds() > rapidShootTime + rapidShootDelay ? 0
        //        : rapidShootOffset * Math.max(rapidShootTimer.seconds() - rapidShootDelay, 0) / rapidShootTime);

        State.CLOSE.position=State.CLOSE.position+correction*K;
        servo.setPosition(state.position);
    }

    public void update()
    {
        updatePositions();
    }
}
