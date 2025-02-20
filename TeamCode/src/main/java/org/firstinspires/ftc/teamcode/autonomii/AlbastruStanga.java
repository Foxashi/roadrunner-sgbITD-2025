package org.firstinspires.ftc.teamcode.autonomii;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
//@TeleOp


public class AlbastruStanga extends LinearOpMode {

    DcMotor fdr, sdr, fst, sst; // sasiu

    DcMotor gdr, gst; // glisiera

    Servo bratSt, bratDr, cleste;



    public void runOpMode() throws InterruptedException {


        fdr = hardwareMap.dcMotor.get("fdr");
        fdr.setDirection(DcMotorSimple.Direction.REVERSE);
        fdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sdr = hardwareMap.dcMotor.get("sdr");
        sdr.setDirection(DcMotorSimple.Direction.REVERSE);
        sdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fst = hardwareMap.dcMotor.get("fst");
        fst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sst = hardwareMap.dcMotor.get("sst");
        sst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gdr = hardwareMap.dcMotor.get("gdr");
        gdr.setDirection(DcMotorSimple.Direction.REVERSE);
        gdr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gst = hardwareMap.dcMotor.get("gst");
        gst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bratSt = hardwareMap.servo.get("bratSt");
        bratSt.setPosition(0);

        bratDr = hardwareMap.servo.get("bratDr");
        bratDr.setDirection(Servo.Direction.REVERSE);
        bratDr.setPosition(0);

        cleste = hardwareMap.servo.get("cleste");
        cleste.setPosition(0.4);

        telemetry.addLine("Gata de pornire");

        waitForStart();



    }


    /****************************************************************************************
     ______   _    _   _   _    _____   _______   _____   _____
     |  ____| | |  | | | \ | |  / ____| |__   __| |_   _| |_   _|
     | |__    | |  | | |  \| | | |         | |      | |     | |
     |  __|   | |  | | | . ` | | |         | |      | |     | |
     | |      | |__| | | |\  | | |____     | |     _| |_   _| |_
     |_|       \____/  |_| \_|  \_____|    |_|    |_____| |_____|
     ****************************************************************************************/

    public void fata(double power, int time) {
        fdr.setPower(power);
        sdr.setPower(power);
        fst.setPower(power);
        sst.setPower(power);
        sleep(time);
        opresteMotoare();
    }
    public void spate(double power, int time) {
        fdr.setPower(-power);
        sdr.setPower(-power);
        fst.setPower(-power);
        sst.setPower(-power);
        sleep(time);
        opresteMotoare();
    }

    public void dreapta(double power, int time){
        fdr.setPower(-power);
        fst.setPower(power);
        sdr.setPower(power);
        sst.setPower(-power);
        sleep(time);
        opresteMotoare();
    }
    public void stanga(double power, int time){
        fdr.setPower(power);
        fst.setPower(-power);
        sdr.setPower(-power);
        sst.setPower(power);
        sleep(time);
        opresteMotoare();
    }
    public void rotirestanga(double power, int time){
        fdr.setPower(power);
        fst.setPower(-power);
        sdr.setPower(power);
        sst.setPower(-power);
        sleep(time);
        opresteMotoare();
    }
    public void rotiredreapta(double power, int time){
        fdr.setPower(-power);
        fst.setPower(power);
        sdr.setPower(-power);
        sst.setPower(power);
        sleep(time);
        opresteMotoare();
    }

    public void opresteMotoare() {
        fdr.setPower(0);
        fst.setPower(0);
        sdr.setPower(0);
        sst.setPower(0);
    }

    public void brat_jos() {
        bratSt.setPosition(1);
        bratDr.setPosition(1);
    }

    public void brat_diagonal() {
        bratSt.setPosition(0.40);
        bratDr.setPosition(0.40);
    }
    public void nivel1() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(1200);
        gst.setTargetPosition(1200);
        gdr.setPower(0.5);
        gst.setPower(0.5);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() && gst.isBusy()) {
            sleep(1);
        }
        gdr.setPower(0.01);
        gst.setPower(0.01);
    }

    public void nivel2() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(3300);
        gst.setTargetPosition(3300);
        gdr.setPower(0.5);
        gst.setPower(0.5);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() && gst.isBusy()) {
            sleep(1);
        }
        gdr.setPower(0.01);
        gst.setPower(0.01);
    }

    public void tractiune() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(1360);
        gst.setTargetPosition(1360);
        gdr.setPower(0.5);
        gst.setPower(0.5);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() && gst.isBusy()) {
            sleep(1);
        }
        gdr.setPower(0.01);
        gst.setPower(0.01);
    }


    public void coborare() {
        gdr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gdr.setTargetPosition(0);
        gst.setTargetPosition(0);
        gdr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gst.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(gdr.isBusy() || gst.isBusy()) {
            gdr.setPower(0.4);
            gst.setPower(0.4);
        }
    }
}