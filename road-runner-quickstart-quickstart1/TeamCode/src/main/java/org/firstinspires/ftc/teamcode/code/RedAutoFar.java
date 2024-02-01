package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous (name = "Red Auto Far")
public class RedAutoFar extends LinearOpMode {

    ParkingPipeline pipeline = new ParkingPipeline(telemetry, false);
    Hardware hw = Hardware.getInstance();

    double initalGyro;

    @Override
    public void runOpMode() throws InterruptedException {

        hw.init(hardwareMap);

        //camera initiation
        hw.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                hw.camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
                hw.camera.setPipeline(pipeline);

                telemetry.addData("Camera has opened successfully", "");
                telemetry.update();

            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera has Broken", "");
                telemetry.update();
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double c_0 = RobotConstants.startPos;
        final double open_full = 1.00;

        final double t_over_d_arm = (3629/90.0);
        final double v_over_d_claw = (1/300.0);

        FullArmController arm_controller = new FullArmController(0.02, this, hw.armMotor, hw.slideMotor);
        Thread arm_controller_thd = new Thread(arm_controller);

        waitForStart();

        arm_controller_thd.start();

        ParkingPipeline.position position = pipeline.getPos();

        position = ParkingPipeline.position.left; //DELETE THIS

        hw.intakeMotor.setPower(-0.2);
        hw.sideClawLeftServo.setPosition(0.76);
        hw.sideClawRightServo.setPosition(0.668);
        hw.sideClawPosServo.setPosition(0.170);

        double time;
        boolean temp = true;
        switch(position){
            case left:

                telemetry.addData("Team element on left", "");
                drive.followTrajectoryAsync(drive.trajectoryBuilder(new Pose2d())
                        .forward(24)
                        .build()
                );
                double stime = getRuntime();
                while(!Thread.currentThread().isInterrupted() && drive.isBusy()){
                    drive.update();
                    while(getRuntime() < stime + 5);
                    hw.sideClawLeftServo.setPosition(0.316);
                }
                break;

            case center:

                telemetry.addData("Team element on middle", "");

                break;

            case right:

                telemetry.addData("Team element on right", "");

                break;
        }

        arm_controller.stop();


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
