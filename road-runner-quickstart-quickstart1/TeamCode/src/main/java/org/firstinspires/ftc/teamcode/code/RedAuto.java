package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "Red Auto")
public class RedAuto extends LinearOpMode {

    ParkingPipeline pipeline = new ParkingPipeline(telemetry, false);
    Hardware hw = Hardware.getInstance();

    double initalGyro;

    final double t_over_d_arm = (3629/90.0);
    final double v_over_d_claw = (1/300.0);

    @Override
    public void runOpMode() throws InterruptedException {

        hw.init(hardwareMap);

        waitForStart();


        FullArmController control = new FullArmController(0.1, this, hw.armMotor, hw.slideMotor);
        Thread controlThd = new Thread(control);
        controlThd.start();

        control.set_arm_target(-2000);

        sleep(2000);

        control.set_slide_target(-1000);

        sleep(2000);

        control.set_slide_target(0);

        sleep(2000);

        control.set_arm_target(0);

        sleep(10000);

        control.stop();


    }
    public void strafe(double inches, double power){
        double radius = 1.85;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 537.7;
        double ticks = (inches /circumference) * ticksPerRotation;

        if(inches < 0){
            //left
            hw.setPower(-power,power,power,-power);
            while(hw.fl.getCurrentPosition() > ticks){

            }
            hw.setPower(0,0,0,0);
        }else{
            //right
            hw.setPower(power,-power,-power,power);
            while(hw.fl.getCurrentPosition() < ticks){

            }
            hw.setPower(0,0,0,0);
        }

    }

    public void driveWithPrecision(double inches){
        double radius = 1.85;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 537.7;
        double ticks = (inches /circumference) * ticksPerRotation;

        double cTicks = (hw.fl.getCurrentPosition() + hw.fr.getCurrentPosition() + hw.bl.getCurrentPosition() + hw.br.getCurrentPosition()) / 4.0;

        double error = ticks - cTicks;

        while(error > 1){

            cTicks = (hw.fl.getCurrentPosition() + hw.fr.getCurrentPosition() + hw.bl.getCurrentPosition() + hw.br.getCurrentPosition()) / 4.0;
            error = ticks - cTicks;
        }

    }

    public void drive(double inches, double power){
        double radius = 1.85;
        double circumference = 2 * Math.PI * radius;
        double ticksPerRotation = 537.7;
        double ticks = (inches /circumference) * ticksPerRotation;

        if(inches > 0){
            hw.setPower(power,power,power,power);
            while(hw.fl.getCurrentPosition() > ticks){

            }
            hw.setPower(0,0,0,0);
        }
    }


}
