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
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {

            fata(0.5, 820);
            stanga(0.5, 600);
            nivel1();
            sleep(1000);
            brat_Sus();
            sleep(1000);
            punePeBara();
            sleep(1000);
            deschideCleste();
            sleep(2000);
            spate(0.5, 500);
            coborare();
            sleep(1000);
            brat_spate();
            sleep(1000);
            dreapta(0.5, 2400);


            telemetry.addLine("GataAutonomie");
            telemetry.update();
        }

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

    public void brat_spate() {
        bratSt.setPosition(0);
        bratDr.setPosition(0);
    }

    public void punePeBara() {
        bratSt.setPosition(0.50);
        bratDr.setPosition(0.50);
    }

    public void brat_Sus() {
        bratSt.setPosition(0.37);
        bratDr.setPosition(0.37);
    }

    public void deschideCleste() {
        cleste.setPosition(1);
    }

    public void inchideCleste() {
        cleste.setPosition(0.4);
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