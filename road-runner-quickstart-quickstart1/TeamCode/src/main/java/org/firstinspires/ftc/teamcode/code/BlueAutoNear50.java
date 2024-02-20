package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;
import java.util.Vector;

@Autonomous (name = "Blue 50 Near")
public class BlueAutoNear50 extends LinearOpMode {

    ParkingPipeline pipeline = new ParkingPipeline(telemetry, true);
    Hardware hw = Hardware.getInstance();

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

        double c_0 = RobotConstants.startPos;
        final double open_full = 1.00;
        final double close_full = 0.462;

        final double t_over_d_arm = (3629/90.0);
        final double v_over_d_claw = (1/300.0);

        FullArmController arm_controller = new FullArmController(0.01, this, hw.armMotor, hw.slideMotor);
        Thread arm_controller_thd = new Thread(arm_controller);


        waitForStart();

        arm_controller_thd.start();

        hw.sideClawLeftServo.setPosition(0.750); //0.750 close, 0.5 open
        hw.sideClawRightServo.setPosition(0.05); //0.05 close, 0.025 open
        hw.sideClawPosServo.setPosition(0.490); //0.170 close

        ParkingPipeline.position position = pipeline.getPos();

        hw.camera.stopStreaming();

        double time;
        boolean temp = true, temp2 = true, temp3 = true, temp4 = true, temp5 = true;
        switch(position){
            case left:

                telemetry.addData("Team element on left", "");

                arm_controller.set_min_arm_pos_for_extend(-1250);

                hw.sideClawPosServo.setPosition(0.170 );

                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                        .lineToSplineHeading(new Pose2d(18, 0, Math.toRadians(-90)))
                        .build()
                );
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(18, 0, Math.toRadians(-90)))
                        .back(12)
                        .build()
                );
                arm_controller.set_arm_target(-1260);
                arm_controller.set_slide_target(-1390);
                hw.sideClawLeftServo.setPosition(0.5);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(18, 12, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(18, 31, Math.toRadians(90)))
                        .build()
                );
                hw.clawPosServo.setPosition(RobotConstants.startPos + 110 * (v_over_d_claw) + (hw.armMotor.getCurrentPosition() * (1/t_over_d_arm)) * v_over_d_claw);
                sleep(1000);
                hw.clawOutServo.setPosition(open_full);
                sleep(1000);
                hw.clawPosServo.setPosition(RobotConstants.startPos);
                arm_controller.set_slide_target(0);
                arm_controller.set_arm_target(-1500);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(18, 31, Math.toRadians(90)))
                        .strafeRight(32)
                        .build()
                );
                arm_controller.set_arm_target(0);
                hw.sideClawPosServo.setPosition(0.490);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(50, 31, Math.toRadians(90)))
                        .forward(20)
                        .build()
                );


                break;

            case center:

                telemetry.addData("Team element on middle", "");

                arm_controller.set_min_arm_pos_for_extend(-1250);

                hw.sideClawPosServo.setPosition(0.170);

                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                        .lineToSplineHeading(new Pose2d(21, 0, Math.toRadians(-90)))
                        .build()
                );
                hw.sideClawLeftServo.setPosition(0.5);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(21, 0, Math.toRadians(-90)))
                        .back(12)
                        .build()
                );
                arm_controller.set_arm_target(-1260);
                arm_controller.set_slide_target(-1390);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(21, 12, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(24, 31, Math.toRadians(90)))
                        .build()
                );
                hw.clawPosServo.setPosition(RobotConstants.startPos + 110 * (v_over_d_claw) + (hw.armMotor.getCurrentPosition() * (1/t_over_d_arm)) * v_over_d_claw);
                sleep(1000);
                hw.clawOutServo.setPosition(open_full);
                sleep(1000);
                arm_controller.set_slide_target(0);
                arm_controller.set_arm_target(-1500);
                hw.clawPosServo.setPosition(RobotConstants.startPos);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(24, 31, Math.toRadians(90)))
                        .strafeRight(26)
                        .build()
                );
                arm_controller.set_arm_target(0);
                hw.sideClawPosServo.setPosition(0.490);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(50, 31, Math.toRadians(90)))
                        .forward(20)
                        .build()
                );


                break;

            case right:

                telemetry.addData("Team element on right", "");
                arm_controller.set_min_arm_pos_for_extend(-1250);

                hw.sideClawPosServo.setPosition(0.170);

                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                        .lineToSplineHeading(new Pose2d(18, 0, Math.toRadians(-90)))
                        .build()
                );
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(18, 0, Math.toRadians(-90)))
                        .forward(12)
                        .build()
                );
                hw.sideClawLeftServo.setPosition(0.5);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(18, -12, Math.toRadians(-90)))
                        .back(24)
                        .build()
                );
                arm_controller.set_arm_target(-1260);
                arm_controller.set_slide_target(-1390);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(18, 12, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(32, 31, Math.toRadians(90)))
                        .build()
                );
                hw.clawPosServo.setPosition(RobotConstants.startPos + 110 * (v_over_d_claw) + (hw.armMotor.getCurrentPosition() * (1/t_over_d_arm)) * v_over_d_claw);
                sleep(1000);
                hw.clawOutServo.setPosition(open_full);
                sleep(1000);
                hw.clawPosServo.setPosition(RobotConstants.startPos);
                arm_controller.set_slide_target(0);
                arm_controller.set_arm_target(-1500);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(32, 31, Math.toRadians(90)))
                        .strafeRight(19)
                        .build()
                );
                arm_controller.set_arm_target(0);
                hw.sideClawPosServo.setPosition(0.490);
                drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(50, 31, Math.toRadians(90)))
                        .forward(20)
                        .build()
                );
                break;
        }

        hw.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(2000);



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
