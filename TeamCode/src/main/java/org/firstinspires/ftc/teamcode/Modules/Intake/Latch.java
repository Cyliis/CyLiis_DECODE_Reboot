package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
public class Latch {

    public static double freePosition=0.13, transferPosition=0.385;
    public static boolean reversed=false;

    public static double maxVel=20, acc=9, dec=9;

    BetterMotionProfile profile;
    Servo servo;

    public enum State {
        TRANSFER(transferPosition), GOING_TRANSFER(transferPosition, TRANSFER),
        FREE(freePosition), GOING_FREE(freePosition, FREE);

        double position;
        State nextState;
        State(double position , State nextState)
        {
            this.position=position;
            this.nextState=nextState;
        }
        State(double position)
        {
            this.position=position;
            nextState=this;
        }
    }
    State state;

    public Latch(State initialState)
    {
        state=initialState;
        servo= Hardware.ssh0;

        if(reversed)servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(state.position);

        profile=new BetterMotionProfile(maxVel, acc, dec);

        profile.setMotion(state.position, state.position, 0);

    }

    public void setState(State state)
    {
        if(state.nextState!=this.state)
        this.state=state;
    }

    public boolean isMoving()
    {
        return state==State.FREE || state==State.TRANSFER;
    }

    private void updateHardware()
    {
        if(profile.finalPosition != state.position)
            profile.setMotion(profile.getPosition(), state.position, profile.getVelocity());

        servo.setPosition(profile.getPosition());

        if(profile.getPosition() == state.position)
            state=state.nextState;
    }

    private void updatePositions()
    {
        State.FREE.position = freePosition;
        State.GOING_FREE.position = freePosition;
        State.TRANSFER.position = transferPosition;
        State.GOING_TRANSFER.position = transferPosition;
    }

    private void updateProfile()
    {
        profile.maxVelocity = maxVel;
        profile.acceleration = acc;
        profile.deceleration = dec;
    }

    public void update()
    {
        updatePositions();
        updateProfile();
        updateHardware();
        profile.update();
    }
}
