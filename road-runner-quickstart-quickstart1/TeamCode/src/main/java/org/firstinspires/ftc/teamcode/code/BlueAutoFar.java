package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "Blue Auto Far")
public class BlueAutoFar extends LinearOpMode {

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
                hw.camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
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

        final double t_to_d_arm = 211.7444444;
        final double d_to_v_claw = (1.0/300.0);
        double c_0 = RobotConstants.startPos;
        double c_30 = c_0 + (10 * (1/d_to_v_claw));
        final double open_full = 1.00;
        double c_60 = c_0 + (60 * (d_to_v_claw));

//        ArmThread thdArm = new ArmThread(hw.armMotor, -1500, 25);
//        Thread armPosThd = new Thread(thdArm);
//
//        ArmThreadStupid slide1 = new ArmThreadStupid(hw.slideMotor, 0, 0);
//        Thread slideThread1 = new Thread(slide1);
//
//        ArmThread thdArm2 = new ArmThread(hw.armMotor, 0, 15);
//        Thread armPosThd2 = new Thread(thdArm2);

        final double t_over_d_arm = (3629/90.0);
        final double v_over_d_claw = (1/300.0);

        FullArmController arm_controller = new FullArmController(0.02, this, hw.armMotor, hw.slideMotor);
        Thread arm_controller_thd = new Thread(arm_controller);

        waitForStart();

        arm_controller_thd.start();

        ParkingPipeline.position position = pipeline.getPos();

        //position = ParkingPipeline.position.left; //DELETE THIS

        hw.intakeMotor.setPower(-0.2);

        double time;
        boolean temp = true;
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .forward(8)
                .build()
        );
        switch(position){
            case right:

                telemetry.addData("Team element on left", "");
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(8,0,Math.toRadians(-90)))
                        .forward(8)
                        .build()
                );
                hw.intakeMotor.setPower(0.3);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(8,8,Math.toRadians(-90)))
                        .back(8)
                        .build()
                );
                break;

            case center:

                telemetry.addData("Team element on middle", "");
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(8,0,Math.toRadians(-90)))
                        .forward(8)
                        .build()
                );
                hw.intakeMotor.setPower(-0.3);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(16,0,Math.toRadians(-90)))
                        .back(8)
                        .build()
                );
                break;

            case left:

                telemetry.addData("Team element on right", "");
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(8,0,Math.toRadians(90)))
                        .forward(8)
                        .build()
                );
                hw.intakeMotor.setPower(0.3);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(8,-8,Math.toRadians(90)))
                        .back(8)
                        .build()
                );
                break;
        }

//        hw.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        hw.armMotor.setTargetPosition(-8353);
//        hw.armMotor.setPower(1);
//        hw.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        while(hw.armMotor.getCurrentPosition() > -8353 + 100){
//            telemetry.addData("arm pos", hw.armMotor.getCurrentPosition());
//            telemetry.update();
//        }
//        hw.clawPosServo.setPosition(c_30 + (hw.armMotor.getCurrentPosition() * (1/t_to_d_arm) * v_to_d_claw));
//        while(hw.slideMotor.getCurrentPosition() > -1463 + 50){
//            hw.slideMotor.setPower(-0.7);
//            telemetry.addData("arm slide", hw.slideMotor.getCurrentPosition());
//            telemetry.addData("arm pow", hw.slideMotor.getPower());
//            telemetry.update();
//        }
//        hw.slideMotor.setPower(0);
//
//        hw.clawOutServo.setPosition(open_full);

        arm_controller.stop();
        telemetry.update();


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
