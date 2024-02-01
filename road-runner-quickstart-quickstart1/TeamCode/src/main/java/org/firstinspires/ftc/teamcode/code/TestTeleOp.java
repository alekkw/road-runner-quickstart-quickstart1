package org.firstinspires.ftc.teamcode.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TEST TELE OP")

public class TestTeleOp extends LinearOpMode {

    Hardware hw = Hardware.getInstance();

    final double t_over_d_arm = (3629/90.0);
    final double v_over_d_claw = (1/300.0);

    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap);

        final double open_servo_right = 0.762;
        final double open_servo_left = 0.190;
        final double open_full = 1.00;
        final double close_full = 0.462;

        double c_0 = RobotConstants.startPos;


        final double drone_release = 0;

        final double lowSlide = hw.slideMotor.getCurrentPosition();
        final double highSlide = -1600;

        double cSlidePos = hw.slideMotor.getCurrentPosition();

        double prior_x_pos = 0;
        double prior_y_pos = 0;
        double prior_heading = 0;
        double prior_time = 0;
       StandardTrackingWheelLocalizer odometry = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<Integer>(), new ArrayList<Integer>());

        FullArmController arm_controller = new FullArmController(0.01, this, hw.armMotor, hw.slideMotor);
        Thread arm_controller_thread = new Thread(arm_controller);
        arm_controller.pause_arm();
        arm_controller.pause_slide();
        boolean has_slide_pause = false, has_arm_pause = false;

        Interpolator claw_pos_interpolator = new Interpolator(
                Interpolator.generateBreakpoints(0, c_0,
                        new double[]{25, c_0 + (3*(v_over_d_claw))},
                        new double[]{30, c_0 + (5*(v_over_d_claw))},
                        new double[]{43, c_0 + (30*(v_over_d_claw))},
                        new double[]{45, c_0 + (80*(v_over_d_claw))}
                )
        );

        boolean game2_dpad_up = false;
        boolean game2_dpad_down = false;
        int queue_arm_pos = 0;

        int queue_arm_pos_table[] = {
                0,
                -1514,
                -1603,
                -1808
        };
        int queue_slide_pos_table[] = {
                0,
                -879,
                -1123,
                -1327
        };

        boolean queue_arm = true;
        boolean queue_arm_switch = true;

        boolean is_using_queue = false;

        boolean game1_dpad_left = false;
        boolean game1_dpad_right = false;
        int queue_move_pos = 0;

        int queue_move_pos_table[][] = {
                new int[] {0,0,0},
                new int[] {10, 10, 0}
        };

        boolean queue_move = true;
        boolean queue_move_switch = true;
        boolean is_using_queue_move = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final double queue_move_START_X = 0, queue_move_START_Y = 0, queue_move_START_HEAD = 0;

        double inital_angle_offset = Math.toRadians(90); //radians

        waitForStart();

        arm_controller_thread.start();

        while(opModeIsActive()){


             c_0 = RobotConstants.startPos;

            //get current velocity/position
            double x_vel = (odometry.getXPos() - prior_x_pos) / (getRuntime() - prior_time);
            double y_vel = (odometry.getYPos() - prior_y_pos) / (getRuntime() - prior_time);
            prior_x_pos = odometry.getXPos();
            prior_y_pos = odometry.getYPos();
            prior_heading = 0;
            prior_time = getRuntime();
//            telemetry.addData("X Pos/Y Pos:", prior_x_pos + "/" + prior_y_pos);
//
//            telemetry.addData("X Pos / Y Pos:", drive.getLocalizer().getPoseEstimate().getX() + " / " + drive.getLocalizer().getPoseEstimate().getY());

            //movement
                //driver oriented driving
            double robot_angle = -drive.getPoseEstimate().getHeading();
            drive.update();
            double n = robot_angle + inital_angle_offset;
            double a = gamepad1.left_stick_x;
            double b = gamepad1.left_stick_y;
            double pi_4 = Math.PI/4;
            double pi_2 = pi_4 * 2;
            double true_x_gamepad = a * Math.cos (n - pi_2) + b * Math.sin(n - pi_2);
            double true_y_gamepad = -a * Math.sin(n - pi_2) + b * Math.cos(n - pi_2);

            telemetry.addData("AHHHHHHHHH 1: ", true_x_gamepad);
            telemetry.addData("AHHHHHHHHH 2: ", true_y_gamepad);
            telemetry.addData("AHHHHHHHHH 3: ", n);

//            double pi_4 = Math.PI/4;
//            double pi_2 = pi_4 * 2;
//            double true_x_gamepad = gamepad1.left_stick_x;
//            double true_y_gamepad = gamepad1.left_stick_y;

                //driving calculations
            //movement
//            double r = Math.hypot(true_x_gamepad, -true_y_gamepad);
//            double robotAngle = Math.atan2(-true_y_gamepad, true_x_gamepad) - Math.PI / 4;
//            double rightX = -gamepad1.right_stick_x;
//            double slow = 1 - gamepad1.right_trigger;
//            final double v1 = (r * Math.cos(robotAngle) - rightX) * (Math.max(slow, 0.25));
//            final double v2 = (r * Math.sin(robotAngle) + rightX) * (Math.max(slow, 0.25));
//            final double v3 = (r * Math.sin(robotAngle) - rightX) * (Math.max(slow, 0.25));
//            final double v4 = (r * Math.cos(robotAngle) + rightX) * (Math.max(slow, 0.25));
            double v1 = true_x_gamepad + true_y_gamepad - gamepad1.right_stick_x;
            double v2 = true_x_gamepad - true_y_gamepad + gamepad1.right_stick_x;
            double v3 = true_x_gamepad - true_y_gamepad - gamepad1.right_stick_x;
            double v4 = true_x_gamepad + true_y_gamepad + gamepad1.right_stick_x;

            //fl, fr, bl, br
            setPower(v1,v2,v3,v4);

            telemetry.addLine(String.format(Locale.ENGLISH, "FR POW: %f\nFL POW: %f\nBR POW: %f\nBL POW: %f", hw.fr.getPower(), hw.fl.getPower(), hw.br.getPower(), hw.bl.getPower()));


            //intake
            hw.intakeMotor.setPower(-gamepad2.right_trigger * 0.45 + gamepad2.left_trigger * 0.45);
            hw.propelMotor.setPower(gamepad2.right_trigger * 0.2 - gamepad2.left_trigger * 0.2);





            if(!is_using_queue){
                //slide
                if(gamepad2.right_stick_y != 0){
                    has_slide_pause = false;
                    if(hw.slideMotor.getCurrentPosition() < highSlide + 10 && gamepad2.right_stick_y < 0){
                        //  telemetry.addData("Case 1: ", highSlide + 10);
                        arm_controller.set_slide_target(highSlide + 10);
                        arm_controller.unpause_slide();
                    }else if(hw.slideMotor.getCurrentPosition() > lowSlide - 10 && gamepad2.right_stick_y > 0){
                        //  telemetry.addData("Case 2: ", lowSlide - 10);
                        arm_controller.pause_slide();
                        hw.slideMotor.setPower(0);
                    }else{
                        // telemetry.addData("Case 3:","");
                        arm_controller.pause_slide();
                        hw.slideMotor.setPower(gamepad2.right_stick_y);
                    }
                }else{
                    if(!has_slide_pause){
                        arm_controller.set_slide_target((hw.slideMotor.getCurrentPosition() < highSlide + 10) ? highSlide + 10 : hw.slideMotor.getCurrentPosition());
                        arm_controller.unpause_slide();
                        has_slide_pause = true;
                    }
                }

                //arm
                if(gamepad2.left_stick_y != 0){
                    arm_controller.pause_arm();
                    hw.armMotor.setPower(gamepad2.left_stick_y);
                    has_arm_pause = false;
                }else{
                    if(!has_arm_pause){
                        arm_controller.set_arm_target(hw.armMotor.getCurrentPosition());
                        arm_controller.unpause_arm();
                        has_arm_pause = true;
                    }
                }
            }

            //queueing system

                //arm queue
            if(gamepad2.dpad_up){
                if(!game2_dpad_up)
                    queue_arm_pos++;
                game2_dpad_up = true;
            }else{
               game2_dpad_up = false;
            }
            if(gamepad2.dpad_down){
                if(!game2_dpad_down)
                    queue_arm_pos--;
                game2_dpad_down = true;
            }else{
                game2_dpad_down = false;
            }

                    //arm activator
            if(gamepad2.left_bumper){
                if(queue_arm){
                    if(queue_arm_switch){
                        //turn on arm
                        arm_controller.set_arm_target(queue_arm_pos_table[queue_arm_pos]);
                        arm_controller.set_slide_target(queue_slide_pos_table[queue_arm_pos]);
                        arm_controller.unpause_arm();
                        arm_controller.unpause_slide();
                        queue_arm_switch = false;
                        is_using_queue = true;
                        has_arm_pause = false;
                        has_slide_pause = false;
                    }else{
                        //turn off
                        arm_controller.pause_arm();
                        arm_controller.pause_slide();
                        is_using_queue = false;
                        queue_arm_switch = true;
                        queue_arm_pos = 0;
                    }
                    queue_arm = false;
                }
            }else{
                queue_arm = true;
            }

            if(is_using_queue && arm_controller.isAtTargets()){
                is_using_queue = false;
                queue_arm_switch = true;
                queue_arm_pos = 0;
            }

                //move queue
            if(gamepad1.dpad_left){
                if(!game1_dpad_left){
                    queue_move_pos++;
                    game1_dpad_left = true;
                }
            }else{
                game1_dpad_left = false;
            }
            if(gamepad1.dpad_right){
                if(!game1_dpad_right){
                    queue_move_pos--;
                    game1_dpad_right = true;
                }
            }else{
                game1_dpad_right = false;
            }

                    //activator
            if(gamepad1.left_bumper){
                if(queue_move){
                    if(queue_move_switch){
                        //turn on move
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(new Pose2d(prior_x_pos, prior_y_pos, prior_heading))
                                .splineTo(new Vector2d(queue_move_pos_table[queue_move_pos][0],queue_move_pos_table[queue_move_pos][1]), queue_move_pos_table[queue_move_pos][2])
                                .build()
                        );
                        queue_move_switch = false;
                        is_using_queue_move = true;
                    }else{
                        //turn off
                        drive.stop();
                        is_using_queue_move = false;
                        queue_move_switch = true;
                        queue_move_pos = 0;
                    }
                    queue_move = false;
                }
            }else{
                queue_move = true;
            }

            while (is_using_queue_move && !Thread.currentThread().isInterrupted() && drive.isBusy()){
                drive.update();
            }

            //door servo
            if(gamepad2.a){
                hw.clawOutServo.setPosition(close_full);
            }
            if(gamepad2.y){
                hw.clawOutServo.setPosition(open_full);
            }
            if(gamepad2.x){
                hw.clawOutServo.setPosition(open_servo_left);
            }
            if(gamepad2.b){
                hw.clawOutServo.setPosition(open_servo_right);
            }

            //claw position servo
            claw_pos(
                    (hw.slideMotor.getCurrentPosition() > -200) ?
                            claw_pos_interpolator.calculate(-arm_degree()) + //negative degree because motor is reversed (should probably fix this cuz counterintuative)
                            arm_degree() * (v_over_d_claw) //correction term
                            :
                            c_0 + 80 * (v_over_d_claw) +
                            arm_degree() * (v_over_d_claw) //correction term
            );
//            telemetry.addData("arm degree: ", arm_degree());
//            telemetry.addData("interpolator: ", claw_pos_interpolator.calculate(-arm_degree()));
//            telemetry.addData("Claw Pos: ", hw.clawPosServo.getPosition());


            if(gamepad1.y)
                hw.droneServo.setPosition(0);

            if(gamepad2.right_bumper){
                hw.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hw.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hw.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_controller.set_arm_target(0);
                arm_controller.set_slide_target(0);
            }



//            telemetry.addData("Claw servo pos", hw.clawPosServo.getPosition());
            telemetry.addData("Slide Position", hw.slideMotor.getCurrentPosition());
            telemetry.addData("Arm pos", hw.armMotor.getCurrentPosition());
            telemetry.addData( "Queue Arm Pos: ", queue_arm_pos);
            telemetry.addData("Queue is done: ", arm_controller.isAtTargets());
//
            telemetry.update();
        }
        arm_controller.stop();

        hw.slideMotor.setPower(0);



    }

    public double arm_degree(){return hw.armMotor.getCurrentPosition() * (1/t_over_d_arm);}

    public void claw_pos(double voltage){
        hw.clawPosServo.setPosition(voltage);
    }

    public void setPower(double fl, double fr, double bl, double br){
        hw.fl.setPower(fl);
        hw.fr.setPower(fr);
        hw.br.setPower(br);
        hw.bl.setPower(bl);
    }

}
