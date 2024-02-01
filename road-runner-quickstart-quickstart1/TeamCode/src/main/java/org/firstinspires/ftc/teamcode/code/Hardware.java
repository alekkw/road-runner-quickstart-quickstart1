package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Hardware {

    //drivetrain
    DcMotor fr, fl, br, bl;

    //arm stuff
    DcMotorEx armMotor, slideMotor;
    Servo clawPosServo, clawOutServo;

    //intake
    DcMotor intakeMotor, propelMotor;

    //camera
    OpenCvCamera camera;

    //drone
    Servo droneServo;

    //pixels
    NormalizedColorSensor leftColor, rightColor;

    //side claw
    Servo sideClawPosServo, sideClawLeftServo, sideClawRightServo;

    private static Hardware hardware;

    public static Hardware getInstance(){
        if(hardware == null){
            hardware = new Hardware();
        }
        return hardware;
    }

    public void init(HardwareMap hwMap){
        try{
            fr = hwMap.get(DcMotor.class, "fr");
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //fr.setDirection(DcMotor.Direction.REVERSE);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setPower(0);
        }catch(Exception e){
            fr = null;
        }
        try{
            fl = hwMap.get(DcMotor.class, "fl");
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fl.setDirection(DcMotor.Direction.REVERSE);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fl.setPower(0);
        }catch(Exception e){
            fl = null;
        }
        try{
            br = hwMap.get(DcMotor.class, "br");
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //br.setDirection(DcMotor.Direction.REVERSE);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            br.setPower(0);
        }catch(Exception e){
            br = null;
        }
        try{
            bl = hwMap.get(DcMotor.class, "bl");
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setDirection(DcMotor.Direction.REVERSE);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bl.setPower(0);
        }catch(Exception e){
            bl = null;
        }
        try{
            intakeMotor = hwMap.get(DcMotor.class, "intake");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setPower(0);
        }catch(Exception e){
            intakeMotor = null;
        }
        try{
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam");
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        }catch(Exception e){
            camera = null;
        }
        try{
            propelMotor = hwMap.get(DcMotor.class, "propel");
            propelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotor.Direction.REVERSE);
            propelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            propelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            propelMotor.setPower(0);
        }catch(Exception e){
            propelMotor = null;
        }
        try{
            armMotor = hwMap.get(DcMotorEx.class, "arm");
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotor.Direction.REVERSE);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setPower(0);
        }catch(Exception e){
            armMotor = null;
        }
        try{
            slideMotor = hwMap.get(DcMotorEx.class, "slide");
            PIDFCoefficients coeffs = new PIDFCoefficients(1,0,0,0);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //bl.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor.setPower(0);
        }catch(Exception e){
            slideMotor = null;
        }
        try{
            clawPosServo = hwMap.servo.get("pos");
        }catch(Exception e){
            clawPosServo = null;
        }
        try{
            clawOutServo = hwMap.servo.get("out");
        }catch(Exception e){
            clawOutServo = null;
        }
        try{
            droneServo = hwMap.servo.get("drone");
        }catch(Exception e){
            droneServo = null;
        }
        try{
            leftColor = hwMap.get(NormalizedColorSensor.class, "left_color");
        }catch(Exception e){
            leftColor = null;
        }
        try{
            sideClawPosServo = hwMap.servo.get("sidePos");
        }catch(Exception e){
            sideClawPosServo = null;
        }
        try{
            sideClawLeftServo = hwMap.servo.get("sideLeft");
        }catch(Exception e){
            sideClawLeftServo = null;
        }
        try{
            sideClawRightServo = hwMap.servo.get("sideRight");
        }catch(Exception e){
            sideClawRightServo = null;
        }
    }

    public void setPower(double fl, double fr, double bl, double br){
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
    }
}
