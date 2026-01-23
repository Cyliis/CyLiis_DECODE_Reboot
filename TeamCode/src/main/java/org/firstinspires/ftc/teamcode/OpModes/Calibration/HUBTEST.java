package org.firstinspires.ftc.teamcode.OpModes.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
@TeleOp
public class HUBTEST extends LinearOpMode {

    enum HARDWARE{
        SERVO("ssh"), MOTOR("mch"), SPARKMINI("sch");

         String name;

        HARDWARE(String name)
        {
            this.name=name;
        }
    }
    public static HARDWARE hardware = HARDWARE.MOTOR;

    enum PORT{
        _0("0"), _1("1"), _2("2"), _3("3"), _4("4"), _5("5");

        String number;

        PORT(String number)
        {
            this.number=number;
        }
    }
    public static PORT port=PORT._0;

    public static double power=0,position=0;


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor;
        Servo servo;
        CRServo crServo;

        waitForStart();

        while(opModeIsActive())
        {
            String string = hardware.name+port.number;

            switch (hardware)
            {
                case SPARKMINI:
                    crServo=hardwareMap.get(CRServo.class, string);
                    crServo.setPower(power);
                    break;
                case SERVO:
                    servo=hardwareMap.get(Servo.class, string);
                    servo.setPosition(position);
                    break;
                case MOTOR:
                    motor=hardwareMap.get(DcMotorEx.class, string);
                    motor.setPower(power);
                    telemetry.addData("a" , motor.getVelocity());
                    break;
            }

            telemetry.update();

        }
    }



}
