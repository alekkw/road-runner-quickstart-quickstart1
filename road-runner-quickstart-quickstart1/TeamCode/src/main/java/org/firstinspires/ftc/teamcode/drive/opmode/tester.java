package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "tester")
public class tester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                .forward(40)
                .build();
        Trajectory t3 = drive.trajectoryBuilder(new Pose2d(40,0,Math.toRadians(90)))
                .forward(20)
                .build();
        Trajectory t4 = drive.trajectoryBuilder(new Pose2d(40,20,Math.toRadians(180)))
                .forward(40)
                .build();
        Trajectory t5 = drive.trajectoryBuilder(new Pose2d(0,20,Math.toRadians(270)))
                .forward(20)
                .build();
        waitForStart();

        drive.followTrajectory(t1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(t3);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(t4);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(t5);
        drive.turn(Math.toRadians(90));

    }
}
