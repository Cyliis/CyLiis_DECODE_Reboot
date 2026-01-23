package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Pitch {

    public static double farPosition=0.87, closePosition=0.6;
    public enum State{
        FAR(farPosition), CLOSE(closePosition);

        double position;
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
        servo.setPosition(state.position);
    }

    private void updatePositions()
    {
        State.FAR.position=farPosition;
        State.CLOSE.position=closePosition;
    }

    public void update()
    {
        updatePositions();
    }
}
