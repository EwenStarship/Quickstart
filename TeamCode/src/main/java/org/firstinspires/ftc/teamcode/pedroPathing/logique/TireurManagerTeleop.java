
package org.firstinspires.ftc.teamcode.pedroPathing.logique;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;

public class TireurManagerTeleop {

    // --- Modules contrôlés ---
    private final Shooter shooter;
    private boolean tirEnCours = false;
    private final SpinTurret tourelle;
    private final AngleShooter ServoAngleShoot;
    private final ServoTireur servoTireur;
    private final Indexeur indexeur;
    private final Intake intake;
    private double angleCibleTourelle = 0;
    private boolean tirAutoActif = false;

    private final AfficheurRight afficheurRight;

    // --- Machine à états ---
    public enum TirState {
        IDLE,
        SHOOTER_SPINUP,
        TURRET_POSITION,
        ANGLE_POSITION,
        SERVO_PUSH,
        SERVO_RETRACT,
        INDEX_ADVANCE,
        WAIT_AFTER_INDEX,
        AVANCE1TIR,
        AFTERWAIT_INDEX
    }

    private TirState state = TirState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private int tirsEffectues = 0;

    private int shotsRemaining = 0;

    private double Min_shooterRPM = 4000;
    private double toleranceVelocityMax;
    private double toleranceVelocityMin;
    private double TargetFlyWheelRPM = 4700;

    // --- Rampe pour le shooter (sans modifier PIDF) ---
    private double rampedShooterRPM = 0;
    private final ElapsedTime dtShooter = new ElapsedTime();

    // Slew rate en RPM/s (20k RPM/s = super rapide)
    private double maxRpmSlewPerSecond = 20000;

    // Kick anti-collage
    private boolean kickActiveShooter = false;
    private final ElapsedTime kickTimerShooter = new ElapsedTime();
    private static final double KICK_MIN_RPM = 900;
    private static final double KICK_TIME_MS = 80;

    private double shootermaxspintime = 2;

    // --- Cibles dynamiques ---
    private double angleCibleShooter = 0;
    private double vitesseCibleShooter = 0;

    public TireurManagerTeleop(Shooter shooter,
                               SpinTurret tourelle,
                               AngleShooter ServoAngleShoot,
                               ServoTireur servoTireur,
                               Indexeur indexeur, Intake intake, AfficheurRight afficheurRight) {

        this.shooter = shooter;
        this.tourelle = tourelle;
        this.ServoAngleShoot = ServoAngleShoot;
        this.servoTireur = servoTireur;
        this.indexeur = indexeur;
        this.intake = intake;
        this.afficheurRight = afficheurRight;

        dtShooter.reset();
        kickActiveShooter = false;
        rampedShooterRPM = 0;
    }

    public void update() {

        boolean tirActif = (state != TirState.IDLE);
        if (tirActif) {
            //afficheurRight.setClignoteVert();
        } else {
            //afficheurRight.setIdle();
        }

        afficheurRight.update();

        switch (state) {

            case IDLE: {
                // IMPORTANT : ramener doucement à 0 via la rampe
                shooter.setShooterTargetRPM(0);
                break;
            }

            case AVANCE1TIR: {
                double rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);

                if (!indexeur.isHomingDone()) {
                    indexeur.lancerHoming();
                    return;
                }
                if (indexeur.isHomingDone()) {
                    indexeur.avancerPourTir();
                    timer.reset();
                    state = TirState.WAIT_AFTER_INDEX;
                }
                break;
            }

            case ANGLE_POSITION: {
                double rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);

                ServoAngleShoot.setAngle(angleCibleShooter);
                if (ServoAngleShoot.isAtAngle(angleCibleShooter)) {
                    timer.reset();
                    state = TirState.SHOOTER_SPINUP;
                }
                break;
            }

            case SHOOTER_SPINUP: {
                double rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);

                if (vitesseCibleShooter < 4000) {
                    toleranceVelocityMin = 0.95 * vitesseCibleShooter;
                    toleranceVelocityMax = 1.08 * vitesseCibleShooter;
                }
                else if (vitesseCibleShooter < 4400) {
                    toleranceVelocityMin = 0.94 * vitesseCibleShooter;
                    toleranceVelocityMax = 1.06 * vitesseCibleShooter;
                }
                else {
                    toleranceVelocityMin = 0.93 * vitesseCibleShooter;
                    toleranceVelocityMax = 1.05 * vitesseCibleShooter;
                }
                if ((shooter.getShooterVelocityRPM() > toleranceVelocityMin)
                        && (shooter.getShooterVelocityRPM() < toleranceVelocityMax)
                        && !indexeur.isindexeurBusy()) {
                    timer.reset();
                    state = TirState.SERVO_PUSH;
                }
                break;
            }

            case SERVO_PUSH: {
                if (!indexeur.isHomingDone()) {
                    indexeur.lancerHoming();
                    return;
                }
                if (indexeur.isHomingDone()) {
                    servoTireur.push();
                    if (timer.milliseconds() > 380) {
                        timer.reset();
                        indexeur.decrementerBalle();
                        state = TirState.SERVO_RETRACT;
                    }
                }
                break;
            }

            case SERVO_RETRACT: {
                servoTireur.retract();
                if (timer.milliseconds() > 250) {
                    timer.reset();
                    if (shotsRemaining > 0) {
                        shotsRemaining--;
                    }
                    tirsEffectues++;
                    state = TirState.INDEX_ADVANCE;
                }
                break;
            }

            case INDEX_ADVANCE: {
                if (shotsRemaining > 0) {
                    shooter.setShooterTargetRPM(0);
                    intake.repriseApresTir();
                    state = TirState.IDLE;
                } else {
                    indexeur.avancerPourTir();
                    timer.reset();
                    state = TirState.WAIT_AFTER_INDEX;
                }
                break;
            }

            case WAIT_AFTER_INDEX: {
                if (timer.milliseconds() > 120) {
                    state = TirState.AFTERWAIT_INDEX;
                }
                break;
            }

            case AFTERWAIT_INDEX: {
                if (indexeur.isRotationTerminee()) {
                    if (shotsRemaining > 0) {
                        state = TirState.SERVO_PUSH;
                    } else {
                        shooter.setShooterTargetRPM(0);
                        intake.repriseApresTir();
                        tirEnCours = false;
                        state = TirState.IDLE;
                    }
                }
                //if (indexeur.isRotationTerminee()) {
                //    timer.reset();
                //    state = TirState.ANGLE_POSITION;
                //}
                break;
            }

            case TURRET_POSITION: {
                double rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);

                tourelle.allerVersAngle(angleCibleTourelle);
                if (tourelle.isAtAngle(angleCibleTourelle)) {
                    timer.reset();
                    tirAutoActif = false;
                    state = TirState.ANGLE_POSITION;
                }
                break;
            }
        }
    }

    // --- Lancer un tir manuel 3 tirs ---
    public void startTirManuel3Tirs(double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 3;
        tirsEffectues = 0;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;

        double rpmCmd = rampShooterRPM(vitesseCibleShooter);
        shooter.setShooterTargetRPM(rpmCmd);

        timer.reset();
        state = TirState.ANGLE_POSITION;
    }

    public void startTirManuel1tir(double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 1;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;
        tirsEffectues = 0;

        double rpmCmd = rampShooterRPM(vitesseCibleShooter);
        shooter.setShooterTargetRPM(rpmCmd);

        timer.reset();
        state = TirState.AVANCE1TIR;
    }

    public void startTirAuto(double angleTourelle, double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirAutoActif = true;
        tirEnCours = true;
        shotsRemaining = 3;
        this.angleCibleTourelle = angleTourelle;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;

        tirsEffectues = 0;

        timer.reset();

        double rpmCmd = rampShooterRPM(vitesseCibleShooter);
        shooter.setShooterTargetRPM(rpmCmd);
        state = TirState.TURRET_POSITION;
    }

    public TirState getState() {
        return state;
    }

    public boolean isBusy() {
        return state != TirState.IDLE;
    }

    public boolean isTirEnCours() { return tirEnCours; }

    public void setState(TirState state) { this.state = state; }

    public boolean isTirAutoActif() { return tirAutoActif; }

    // --- Rampe rapide + kick (sans toucher au PIDF) ---
    private double rampShooterRPM(double targetRPM) {

        double dt = dtShooter.seconds();
        dtShooter.reset();

        double currentRPM = shooter.getShooterVelocityRPM();

        // ==============================
        // 🚀 PAS DE RAMPE AU-DESSUS DE 4000
        // ==============================
        if (targetRPM >= 4000) {
            rampedShooterRPM = targetRPM;   // reset rampe pour éviter dérive
            kickActiveShooter = false;
            return targetRPM;
        }

        // ==============================
        // 🛑 STOP DIRECT SI ON COUPE
        // ==============================
        if (targetRPM <= 0) {
            rampedShooterRPM = 0;
            kickActiveShooter = false;
            return 0;
        }

        // ==============================
        // ⚡ KICK de démarrage bas régime
        // ==============================
        if (currentRPM < KICK_MIN_RPM && rampedShooterRPM < KICK_MIN_RPM * 0.8) {
            if (!kickActiveShooter) {
                kickActiveShooter = true;
                kickTimerShooter.reset();
            }
        }

        double commanded = targetRPM;

        if (kickActiveShooter) {
            commanded = Math.max(targetRPM, KICK_MIN_RPM);
            if (kickTimerShooter.milliseconds() > KICK_TIME_MS || currentRPM >= KICK_MIN_RPM * 0.9) {
                kickActiveShooter = false;
            }
        }
        // ==============================
        // 🧠 RAMPE UNIQUEMENT BASSE VITESSE
        // ==============================
        double slewUp   = 3500;   // montée douce pour 3800
        double slewDown = 12000;  // descente rapide

        double maxUp   = slewUp * dt;
        double maxDown = slewDown * dt;

        double delta = commanded - rampedShooterRPM;

        if (delta >  maxUp)   delta =  maxUp;
        if (delta < -maxDown) delta = -maxDown;

        rampedShooterRPM += delta;

        return rampedShooterRPM;
    }
    public void shotsRemaining(int nb) {shotsRemaining = nb;}

    public void cancelTir() {
        shotsRemaining = 0;
        tirEnCours = false;
        servoTireur.retract();
        shooter.setShooterTargetRPM(0);
        intake.repriseApresTir();
        state = TirState.IDLE;
    }

}
