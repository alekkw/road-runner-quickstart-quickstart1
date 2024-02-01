package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Locale;

@TeleOp(name = "TeleOpDebugger")
public class TeleOpDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = Hardware.getInstance();
        hw.init(hardwareMap);



        waitForStart();


//        double ctime = getRuntime();
//        int slideTarget = -(int)((Math.random() * 1000) + 200);

//        PIDController slideController = new PIDController(0.05, this, hw.slideMotor,
//                new double[][] {{RobotConstants.slide_kp_up, RobotConstants.slide_ki_up, RobotConstants.slide_kd_up},
//                                {RobotConstants.slide_kp_down, RobotConstants.slide_ki_down, RobotConstants.slide_kd_down}},
//                (Double current_pos, Double target_pos) -> (current_pos > target_pos) ? 0 : 1);
//        slideController.setTarget(slideTarget);
//        slideController.reverse();

//        Thread slideThread = new Thread(slideController);
//        slideThread.start();
//
//        int armTarget = hw.armMotor.getCurrentPosition();
//
//        PIDController armController = new PIDController(0.05, this, hw.armMotor);
//        armController.setTarget(armTarget);
//        armController.reverse();
//
//        Thread armThread = new Thread(armController);
//        armThread.start();


        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry t = dash.getTelemetry();

        final double t_over_d_arm = (3629/90.0);
        //final double v_over_d_claw = (0.36/90);
        final double v_over_d_claw = (1/300.0);

        boolean has_slide_pause = false, has_arm_pause = false;
        final double lowSlide = hw.slideMotor.getCurrentPosition();
        final double highSlide = -1600;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while(opModeIsActive()){

//            slideController.setCoeffs( new double[][] {{RobotConstants.slide_kp_up, RobotConstants.slide_ki_up, RobotConstants.slide_kd_up},
//                                                        {RobotConstants.slide_kp_down, RobotConstants.slide_ki_down, RobotConstants.slide_kd_down}});
//            armController.setCoeffs(RobotConstants.arm_kp, RobotConstants.arm_ki, RobotConstants.arm_kd);


            if(gamepad1.a){
                hw.fl.setPower(1);
            }else{
                hw.fl.setPower(0);
            }
            if(gamepad1.b){
                hw.fr.setPower(1);
            }else{
                hw.fr.setPower(0);
            }
            if(gamepad1.x){
                hw.bl.setPower(1);
            }else{
                hw.bl.setPower(0);
            }
            if(gamepad1.y){
                hw.br.setPower(1);
            }else{
                hw.br.setPower(0);
            }


            if(gamepad2.right_bumper){
                hw.intakeMotor.setPower(-1);
                hw.propelMotor.setPower(1);
            }else{
                hw.intakeMotor.setPower(0);
                hw.propelMotor.setPower(0);
            }

            hw.armMotor.setPower(gamepad2.left_stick_y);
            hw.slideMotor.setPower(gamepad2.right_stick_y);



            if(hw.armMotor.getCurrentPosition() < (-50 * t_over_d_arm)){
                hw.clawPosServo.setPosition(RobotConstants.startPos + (60 * v_over_d_claw) + (hw.armMotor.getCurrentPosition() * (1 / t_over_d_arm) * v_over_d_claw));
            }else{
                hw.clawPosServo.setPosition(RobotConstants.startPos + (hw.armMotor.getCurrentPosition() * (1 / t_over_d_arm) * v_over_d_claw));
            }

            NormalizedRGBA colors = hw.leftColor.getNormalizedColors();
            telemetry.addLine(String.format(Locale.ENGLISH, "Red: %f\tGreen: %f\tBlue: %f", colors.red, colors.green, colors.blue));
            telemetry.addData("Distance: ", ((DistanceSensor)hw.leftColor).getDistance(DistanceUnit.CM));

            double x1 = hw.propelMotor.getCurrentPosition() / 2000.0 * (2 * Math.PI * 24/25.4), x2 = -hw.intakeMotor.getCurrentPosition() / 2000.0 * (2 * Math.PI * 24/25.4);
            telemetry.addData("X Wheel 1 (in): ", x1);
            telemetry.addData("X Wheel 2 (in): ", x2);
            telemetry.addData("Theoretical Angle (my way): ", ((x1 - x2) / 2.0) / (Math.PI * RobotConstants.effectiveTrackWidth));
            telemetry.addData("Theoretical Angle (chatGPT): ", Math.atan((x1 - x2) / RobotConstants.effectiveTrackWidth));
            telemetry.addData("Arm Motor Power: ", hw.armMotor.getPower());
            telemetry.addData("Slide Motor Power: ", hw.slideMotor.getPower());
            telemetry.addData("Theoretical Angle (RoadRunner): ", drive.getPoseEstimate().getHeading());
            drive.update();

/*            if(gamepad2.b){
                hw.armMotor.setPower(1);
            }else if(gamepad2.y){
                hw.armMotor.setPower(-1);
            }else{
                hw.armMotor.setPower(0);
            }
            if(gamepad2.x){
                hw.slideMotor.setPower(1);
            }else{
                hw.slideMotor.setPower(0);
            }*/


//            if(getRuntime() > ctime + 4){
//                slideTarget = -(int)((Math.random() * 1000) + 200);
//                slideController.setTarget(slideTarget);
//                ctime = getRuntime();
//            }
//
//            hw.armMotor.setPower(armController.getControl());
//            hw.slideMotor.setPower(slideController.getControl());
//
//            telemetry.addData("Arm Pos: ", hw.armMotor.getCurrentPosition());
//            telemetry.addData("Servo Pos: ", hw.clawPosServo.getPosition());
//            telemetry.update();
//
//            t.addData("Current Position: ", hw.slideMotor.getCurrentPosition());
//            t.addData("Target Position: ", slideTarget);
//            t.addData("Error: ", hw.slideMotor.getCurrentPosition() - slideTarget);
            t.update();
            telemetry.update();



        }

//        slideController.stop();
//        armController.stop();

    }
}
