package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "Blue Auto")
public class BlueAuto extends LinearOpMode {

    Hardware hw = Hardware.getInstance();
    double initalGyro;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        hw.init(hardwareMap);

        //turn(340, 0.1);
        driveWithPrecision(10,0.3);

    }

    public void setPower(double fl, double fr, double bl, double br){
        hw.fl.setPower(fl);
        hw.fr.setPower(fr);
        hw.br.setPower(br);
        hw.bl.setPower(bl);
    }

    public void strafe(double inches, double power){
        double radius = 1.85;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 537.7;
        double ticks = (inches /circumference) * ticksPerRotation;

        if(inches < 0){
            //left
            setPower(-power,power,power,-power);
            while(hw.fl.getCurrentPosition() > ticks){

            }
            hw.setPower(0,0,0,0);
        }else{
            //right
            setPower(power,-power,-power,power);
            while(hw.fl.getCurrentPosition() < ticks){

            }
            setPower(0,0,0,0);
        }

    }

    public void strafeWithPrecision(double inches, double maxPower){

    }

    public void driveWithPrecision(double inches, double maxPower){
        double radius = 1.85;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 537.7;
        double ticks = (inches /circumference) * ticksPerRotation;

        double cTicks = (hw.fl.getCurrentPosition() + hw.fr.getCurrentPosition() + hw.bl.getCurrentPosition() + hw.br.getCurrentPosition()) / 4.0;

        double error = ticks - cTicks;


        while(error > 0.1 || error < -0.1){

            cTicks = (hw.fl.getCurrentPosition() + hw.fr.getCurrentPosition() + hw.bl.getCurrentPosition() + hw.br.getCurrentPosition()) / 4.0;
            error = ticks - cTicks;
        }
        hw.setPower(0,0,0,0);

    }

    public void drive(double inches, double power){
        double radius = 1.85;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 537.7;
        double ticks = (inches /circumference) * ticksPerRotation;

        if(inches > 0){
            setPower(power,power,power,power);
            while(hw.fl.getCurrentPosition() > ticks){

            }
            setPower(0,0,0,0);
        }
    }

}
