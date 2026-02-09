package org.firstinspires.ftc.teamcode.pedroPathing.logique;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;

public class TireurManager {

    // --- Modules contrôlés ---
    private final Shooter shooter;
    private static final int TOL_STRICT_TICKS = 50;
    boolean tirDeclenche = false;
    private boolean tirEnCours = false;
    private final SpinTurret tourelle;
    private final AngleShooter ServoAngleShoot;
    private final ServoTireur servoTireur;
    private final Indexeur indexeur;
    private final Intake intake;

    private double toleranceVelocityMax;
    private double toleranceVelocityMin;

    private final AfficheurRight afficheurRight;

    // --- Machine à états ---
    public enum TirState {
        IDLE,
        SHOOTER_SPINUP,
        SPINUP_CONTROL,
        TURRET_POSITION,
        ANGLE_POSITION,
        SERVO_PUSH,
        SERVO_RETRACT,
        INDEX_ADVANCE,
        AFTERWAIT_INDEX,
        WAIT_AFTER_INDEX
    }

    private TirState state = TirState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private int tirsEffectues = 0;

    private int shotsRemaining = 0;

    private double Min_shooterRPM = 4000;
    private double TargetFlyWheelRPM = 4700;

    private double shootermaxspintime = 2;

    // --- Cibles dynamiques ---
    private double angleCibleTourelle = 0;
    private boolean kickActiveShooter = false;
    private final ElapsedTime kickTimerShooter = new ElapsedTime();
    private static final double KICK_MIN_RPM = 900;
    private static final double KICK_TIME_MS = 80;
    private double angleCibleShooter = 0;
    private double vitesseCibleShooter = 0;
    private double rampedShooterRPM = 0;
    private boolean prespinActif = false;
    private double prespinRPM = 0;

    private final ElapsedTime dtShooter = new ElapsedTime();

    public TireurManager(Shooter shooter,
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

    private void checkIndexeurCoherence() {
        if (!indexeur.isHomingDone()) return;      // pas de check tant que non calé
        if (indexeur.isindexeurBusy()) return;     // laisse finir la rotation

        int err = indexeur.getPositionErreurTicks();
        if (err > TOL_STRICT_TICKS) {
            indexeur.microCorrigerVersCible(0.25); // correction douce vers cible planifiée
        }
    }

    public void update() {

        boolean tirActif = (state != TirState.IDLE);
        if (tirActif) {
            afficheurRight.setClignoteVert();
        } else {
            afficheurRight.setIdle();
        }

        afficheurRight.update();

        switch (state) {

            case IDLE:
                if (prespinActif) {
                    shooter.setShooterTargetRPM(rampShooterRPM(prespinRPM));
                } else {
                    shooter.setShooterTargetRPM(0);
                }
                break;

            // --- 1) Shooter spin-up ---
            case SHOOTER_SPINUP:
                intake.arretPourTir();
                if (shotsRemaining > 0) {
                    double rpmCmd = rampShooterRPM(vitesseCibleShooter);
                    shooter.setShooterTargetRPM(rpmCmd);
                    intake.disableRamassage();
                    //tourelle.allerVersAngle(angleCibleTourelle);
                    if (!indexeur.isHomingDone()) {
                        indexeur.lancerHoming();
                        return;
                    }
                    if (indexeur.isHomingDone()) {
                        state = TirState.ANGLE_POSITION;
                        timer.reset();
                    }
                } else {
                    state = TirState.IDLE;
                    shooter.setShooterTargetRPM(0);
                }
                break;

            case ANGLE_POSITION:
                double rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);
                ServoAngleShoot.setAngle(angleCibleShooter);

                // 1) Indexeur pas prêt → correction → rester dans ANGLE_POSITION
                if (!indexeur.isHomingDone()) {
                    indexeur.lancerHoming();
                    return;
                }
                if (!indexeur.indexeurPretPourTir()) {
                    indexeur.microCorrigerVersCible(0.20);
                    return; // rester tant que l'indexeur n'est pas calé
                }

                // 2) Volet pas encore calé → attendre ici
                if (!ServoAngleShoot.isAtAngle(angleCibleShooter)) {
                    return;
                }

                // 3) Les deux sont prêts → on peut avancer
                timer.reset();
                state = TirState.SPINUP_CONTROL;
                break;

            // --- 3) Contrôle de la montée en régime ---
            case SPINUP_CONTROL:
                rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);

                if (vitesseCibleShooter < 4000) {
                    toleranceVelocityMin = 0.94 * vitesseCibleShooter;
                    toleranceVelocityMax = 1.08 * vitesseCibleShooter;
                }
                else if (vitesseCibleShooter < 4400) {
                    toleranceVelocityMin = 0.93 * vitesseCibleShooter;
                    toleranceVelocityMax = 1.07 * vitesseCibleShooter;
                }
                else {
                    toleranceVelocityMin = 0.93 * vitesseCibleShooter;
                    toleranceVelocityMax = 1.05 * vitesseCibleShooter;
                }

                if ((shooter.getShooterVelocityRPM() > toleranceVelocityMin)
                        && (shooter.getShooterVelocityRPM() < toleranceVelocityMax)
                        && !indexeur.isindexeurBusy()) {
                    timer.reset();
                    state = TirState.TURRET_POSITION;
                }

                // NOTE : timeout laissé inchangé (comportement d'origine)
                if (timer.milliseconds() > 2500) {
                    timer.reset();
                    state = TirState.SERVO_RETRACT;
                    indexeur.decrementerBalle();
                }
                break;

            // --- 2) Positionnement tourelle ---
            case TURRET_POSITION:
                rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);
                tourelle.allerVersAngle(angleCibleTourelle);

                if (tourelle.isAtAngle(angleCibleTourelle)) {
                    timer.reset();                // reset à l'entrée de SERVO_PUSH
                    state = TirState.SERVO_PUSH;
                }
                break;

            // --- 4) Pousser la balle ---
            case SERVO_PUSH: {
                rpmCmd = rampShooterRPM(vitesseCibleShooter);
                shooter.setShooterTargetRPM(rpmCmd);

                servoTireur.push();

                // Attendre 200 ms avant de décrémenter et passer à RETRACT
                if (timer.milliseconds() > 200) {
                    indexeur.decrementerBalle(); // décrémentation unique, au bon moment
                    timer.reset();               // reset à l'entrée de SERVO_RETRACT
                    state = TirState.SERVO_RETRACT;
                }
                break;
            }

            // --- 5) Rétracter le servo ---
            case SERVO_RETRACT:
                servoTireur.retract();
                if (timer.milliseconds() > 150) {
                    timer.reset();
                    shotsRemaining--;
                    tirsEffectues++;
                    state = TirState.INDEX_ADVANCE;
                }
                break;

            // --- 6) Avancer l'indexeur ou finir ---
            case INDEX_ADVANCE:
                if (shotsRemaining == 0) {
                    shooter.setShooterTargetRPM(0);
                    intake.repriseApresTir();
                    timer.reset();
                    state = TirState.IDLE;

                } else {
                    indexeur.avancerPourTir();
                    timer.reset();
                    state = TirState.WAIT_AFTER_INDEX;
                }
                break;

            // --- 7) Petite pause avant tir suivant ---
            case WAIT_AFTER_INDEX:
                if (timer.milliseconds() > 10) {
                    state = TirState.AFTERWAIT_INDEX;
                }
                break;

            case AFTERWAIT_INDEX: {
                // Cohérence stricte (comme TeleOp)
                checkIndexeurCoherence();
                if (indexeur.isRotationTerminee()) {
                    if (shotsRemaining > 0) {
                        // Fenêtre stricte : ne pas perdre un tir
                        if (!indexeur.indexeurPretPourTir()) {
                            indexeur.microCorrigerVersCible(0.20);
                            return; // rester ici jusqu'au calage
                        }
                        timer.reset();           // reset à l'entrée de SERVO_PUSH
                        state = TirState.SERVO_PUSH;
                    } else {
                        shooter.setShooterTargetRPM(0);
                        intake.repriseApresTir();
                        timer.reset();
                        state = TirState.IDLE;
                    }
                }
                break;
            }
        }
    }

    // --- Lancer un tir automatique ---
    public void startTirAuto(double angleTourelle, double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 3;
        indexeur.setBalles(3);
        prespinActif = false;
        this.angleCibleTourelle = angleTourelle;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;

        tirsEffectues = 0;

        shooter.setShooterTargetRPM(vitesseCibleShooter);  // Démarre immédiatement
        state = TirState.SHOOTER_SPINUP;
    }

    public TirState getState() {
        return state;
    }

    public boolean isBusy() {
        return state != TirState.IDLE;
    }

    public boolean isTirEnCours() {
        return tirEnCours;
    }

    private double rampShooterRPM(double targetRPM) {

        double dt = dtShooter.seconds();
        dtShooter.reset();

        double currentRPM = shooter.getShooterVelocityRPM();

        // PAS DE RAMPE AU-DESSUS DE 4800 (commentaire aligné avec la condition)
        if (targetRPM >= 4800) {
            rampedShooterRPM = targetRPM;   // reset rampe pour éviter dérive
            kickActiveShooter = false;
            return targetRPM;
        }

        // STOP DIRECT SI ON COUPE
        if (targetRPM <= 0) {
            rampedShooterRPM = 0;
            kickActiveShooter = false;
            return 0;
        }

        // KICK de démarrage bas régime
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

        // RAMPE UNIQUEMENT BASSE VITESSE
        double slewUp = 3500;    // montée douce pour ~3800
        double slewDown = 12000; // descente rapide

        double maxUp = slewUp * dt;
        double maxDown = slewDown * dt;

        double delta = commanded - rampedShooterRPM;

        if (delta >  maxUp)  delta =  maxUp;
        if (delta < -maxDown) delta = -maxDown;

        rampedShooterRPM += delta;

        return rampedShooterRPM;
    }

    public void prespinShooter(double rpm) {
        prespinRPM = rpm;
        prespinActif = true;
    }

    public void stopPrespin() {
        prespinActif = false;
    }

    public void shotsRemaining(int n) { shotsRemaining = n; }
}



