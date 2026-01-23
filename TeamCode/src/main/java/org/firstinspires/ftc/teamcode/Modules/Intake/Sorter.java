package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Math._33VtoRadians;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.opencv.core.Mat;

@Config
public class Sorter {


    public enum Case{
        PPG("PPG"), PGP("PGP"), GPP("GPP");

        String string;
        Case( String string)
        {
            this.string=string;
        }
    }
    Case aCase=Case.PPG;

    public enum Ball{
        GREEN("G"), PURPLE("P"), NOTHING("");

        String string;
        Ball( String string)
        {
            this.string=string;
        }
    }
    public Ball ball1 = Ball.NOTHING, ball2 = Ball.NOTHING, ball3 = Ball.NOTHING;


    public boolean sort=false;

    public static double ball1Position=0,ball2Position= 2*Math.PI/3, ball3Position= 4*Math.PI/3, readyTransferPosition=Math.PI-0.2 , shootingPosition=Math.PI-0.2 , sortingInitialPosition;
    public static double angle, zeroAngle=0.3;
    public static double error=0;

    DigitalChannel bb;
    CRServo motor;
    AnalogInput encoder;

    ElapsedTime timer;

    public static double KP=0.82, KI, KD=0.023;
    public static double KP_SPECIAL, KI_SPECIAL, KD_SPECIAL;
    PIDController controller = new PIDController(KP, KI, KD);

    public enum State{
        BeforeReset(shootingPosition),FastShooting(shootingPosition, BeforeReset), ReadyForTransfer(readyTransferPosition, FastShooting),GoingReadyForTransfer(readyTransferPosition, ReadyForTransfer),
        Ball3(ball3Position, GoingReadyForTransfer), GoingBall3(ball3Position, Ball3),
        Ball2(ball2Position,GoingBall3), GoingBall2(ball2Position, Ball2),
        Ball1(ball1Position, GoingBall2), GoingBall1(ball1Position, Ball1),

         SortedShooting(shootingPosition, BeforeReset);

        State nextState;
        public double position;
        State(double position)
        {
            nextState=this;
            this.position=position;
        }

        State(double position, State nextState)
        {
            this.nextState=nextState;
            this.position=position;
        }
    }
    public static State state;

    public Sorter(State initialState)
    {
        timer=new ElapsedTime();
        motor = Hardware.sch2;
        encoder = Hardware.analogInput;
        bb=Hardware.bb;
        state=initialState;

    }

    public void Sort()
    {
        sort=true;
    }
    public void Rapid()
    {
        sort=false;
    }

    public void goNext()
    {
        if(state==State.Ball1 || state==State.Ball2 || state==State.Ball3)state=state.nextState;
    }

    public void shoot()
    {
        if(state==State.ReadyForTransfer)state=state.nextState;
    }

    public boolean isMoving()
    {
        return Math.abs(error)>0.20;
    }

    private void updateHardware()
    {

        if(state==State.FastShooting)
        {
            motor.setPower(-1);
            return;
        }

        { error =  state.position - angle;
        if(java.lang.Math.abs(error)> java.lang.Math.PI )error = -java.lang.Math.signum (error) * ( 2 * java.lang.Math.PI - java.lang.Math.abs(error));

         double power=controller.calculate( error, 0 );
         motor.setPower(power);}
    }

    private void updatePID()
    {
        if(state!=State.SortedShooting)
        {
            controller.kp=KP;
            controller.ki=KI;
            controller.kd=KD;
        }
        else
        {
            controller.kp=KP_SPECIAL;
            controller.ki=KI_SPECIAL;
            controller.kd=KD_SPECIAL;
        }
    }

    private void updateStates()
    {
        switch (state)
        {
            case GoingBall1:
            case GoingBall2:
            case GoingBall3:
            case GoingReadyForTransfer:
            case BeforeReset:
                if(Math.abs(angle-state.position)<0.25)state=state.nextState;
            break;
            case FastShooting:
                if(timer.seconds()>6)timer.reset();
                if(timer.seconds()>0.5)state=state.nextState;
                break;
            case Ball1:
            case Ball2:
            case Ball3:
            case SortedShooting:
            case ReadyForTransfer:
                break;
        }
    }

    private void updateReadyTransferPosition()
    {
        if( ( ball1.string + ball2.string + ball3.string ).equals( aCase.string ) ) sortingInitialPosition = Math.PI-0.2;
        if( ( ball2.string + ball3.string + ball1.string ).equals( aCase.string ) ) sortingInitialPosition = (Math.PI+ Math.PI*2/3-0.2)%(Math.PI*2);
        if( ( ball1.string + ball3.string + ball2.string ).equals( aCase.string ) ) sortingInitialPosition = (Math.PI + Math.PI*4/3-0.2)%(Math.PI*2);

    }

    private void updateAngle()
    {
        angle=(_33VtoRadians.convert(encoder.getVoltage())+zeroAngle)%(Math.PI*2);
    }

    public void update()
    {
        State.ReadyForTransfer.position=readyTransferPosition;
        updatePID();
        updateReadyTransferPosition();
        updateAngle();
        updateStates();
        updateHardware();

    }
}
