package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TeleOpDecode;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.TeleOpDecodeBleuCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManager;
import org.firstinspires.ftc.teamcode.pedroPathing.navigation.GlobalPoseStorage;


@Autonomous (name="BLEU--GOAL", group="Competition")
public class DecodeBlueAutoGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //private ElapsedTime pathTimer = new ElapsedTime();
    //private ElapsedTime opModeTimer = new ElapsedTime();
    private Shooter shooter;
    private SpinTurret tourelle;
    private AngleShooter ServoAngleShoot;
    private ServoTireur servoTireur;
    private Indexeur indexeur;
    private Intake intake;

    private AfficheurLeft afficheurLeft;
    private AfficheurRight afficheurRight;

    private TireurManager tireurManager;

    private boolean shotsTriggered = false;
    private boolean turretZeroDone = false;
    private long imuReadySince = 0;

    public enum PathState {
        //Start Position -End Position
        //drive > Movement
        //Shoot >Attempts to Score the Artifact
        DRIVE_STARTPOSITIONTOSHOOT,
        align_RANGEE1Blue,
        align_rangee2blue,
        align_rangee3blue,
        intakeballeRangee1,
        intakerange2,
        intakerangee3,
        PremierTir,
        DrivedeuxiemeShoot,
        deuxiemetir,
        DriveTroisiemeTir,
        troisiemetir,
        Drive2Gate,
        Tourne1,
        atgate
    }
    PathState pathState;

    private final Pose startPose = new Pose(21,125, Math.toRadians(144));
    private final Pose firstshootPose = new Pose(58,84,Math.toRadians(135));

    private final Pose Tourne1= new Pose (62,89, Math.toRadians(180));
    private final Pose drivetoligne1= new Pose (45.50, 91.00, Math.toRadians(180));

    private final Pose avalerballeRangee1 = new Pose (20.0, 91.80, Math.toRadians(180));

    private final Pose Shoot2 = new Pose (51.80, 84.70, Math.toRadians(180));

    private final Pose Shoot3 = new Pose (51.80, 84.70, Math.toRadians(180));

    private final Pose drivetoligne2= new Pose (44.40, 73.00, Math.toRadians(180));

    private final Pose avalerballeRangee2= new Pose (13.5, 66.00, Math.toRadians(180));

    private final Pose Gate= new Pose (14, 70, Math.toRadians(-90));

    private PathChain driveStartofirstShootPos,driveTourne1, driveShoot2pickup1Pos, driveAvalerpremiereLigne, DrivedeuxiemeShoot,drivetorangee2, drivetavalerdeuxiemeligne,driveAvaler2emeLignetotroisemeShoot,DrivetoGate;

    public void buildPaths() {
        //put the coordinate from start to shooting
        driveStartofirstShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstshootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstshootPose.getHeading())
                .build();

        // 1er tir a Virage 1
        driveTourne1 = follower.pathBuilder()

                .addPath(new BezierLine(firstshootPose,Tourne1))
                .setLinearHeadingInterpolation(firstshootPose.getHeading(), Tourne1.getHeading())
                .build();


        //du premiershoot à la rangée numéro 1
        driveShoot2pickup1Pos = follower.pathBuilder()
                .addPath(new BezierLine(firstshootPose, drivetoligne1))
                .setLinearHeadingInterpolation(Tourne1.getHeading(), drivetoligne1.getHeading())
                .build();

        //de la l'alignement pickup1 à la derniere balle rangée 1
        driveAvalerpremiereLigne = follower.pathBuilder()
                .addPath(new BezierLine(drivetoligne1,avalerballeRangee1))
                .setLinearHeadingInterpolation(drivetoligne1.getHeading(), avalerballeRangee1.getHeading())
                .setVelocityConstraint(0.23)
                .build();
        //Aller à la zone de Tir apres avoir avaler les balles de la rangée 1
        DrivedeuxiemeShoot = follower.pathBuilder()
                .addPath(new BezierLine(avalerballeRangee1,Shoot2))
                .setLinearHeadingInterpolation(avalerballeRangee1.getHeading(), Shoot2.getHeading())
                .build();
        //Aller s'aligner à la deuxieme rangée de balle
        drivetorangee2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, drivetoligne2))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), drivetoligne2.getHeading())
                .build();

        //Aller avaler les balles de la rangee 2
        drivetavalerdeuxiemeligne = follower.pathBuilder()
                .addPath(new BezierLine(drivetoligne2, avalerballeRangee2))
                .setLinearHeadingInterpolation(drivetoligne2.getHeading(), avalerballeRangee2.getHeading())
                .setVelocityConstraint(0.23)
                .build();

        //Aller à la zone de Tir apres avoir avaler les balles de la rangée 2
        driveAvaler2emeLignetotroisemeShoot = follower.pathBuilder()
                .addPath(new BezierLine(avalerballeRangee2,Shoot3))
                .setLinearHeadingInterpolation(avalerballeRangee2.getHeading(), Shoot3.getHeading())
                .build();


        //Aller de la zone du troisieme Tir à la troisieme rangée
        DrivetoGate= follower.pathBuilder()
                .addPath(new BezierLine(Shoot2,Gate))
                .setLinearHeadingInterpolation(Shoot2.getHeading(),Gate.getHeading())
                .build();


    }
    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOSITIONTOSHOOT:
                follower.followPath(driveStartofirstShootPos,0.70, true); //true will hold the positon
                setPathState(PathState.PremierTir); // Reset Timer + make new staet
                tireurManager.prespinShooter(4000);
                break;



            case PremierTir: // Premier tir en cours
                //intake.update();

                //indexeur.update();
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>1.0) {
                    // avons nous deja demandé des tirs :

                    if (!shotsTriggered){
                        tireurManager.stopPrespin();
                        tireurManager.startTirAuto(// Lancer tir automatique
                                -5,   // angle tourelle (exemple)
                                0.45,  // angle shooter
                                4000   // RPM
                        );
                        shotsTriggered = true;}
                    else if (shotsTriggered && !tireurManager.isBusy()){
                            setPathState(PathState.align_RANGEE1Blue);
                            shotsTriggered = false;
                        }

                }
                break;


            case Tourne1: // on tourne avant de rammaser les balles
                intake.update();
                indexeur.update();
                if (!follower.isBusy()) {
                    //follower.turn(45,false);
                    //follower.turnDegrees(45,true);
                    //follower.followPath(driveTourne1 ,0.8, false);
                    telemetry.addLine("Done with Shooting 1, tourner vers premiere rangée");
                    // transition to next state
                     // chemin d'alignement de la premiere rangée
                    setPathState(PathState.align_RANGEE1Blue); // on va a l'étape suivante
                }
                break;

            case align_RANGEE1Blue: // On va s'aliner avec la rangée 1
                // check is follow done is path
                intake.update();
                indexeur.update();
                //if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5) {
                if (!follower.isBusy()) {
                    telemetry.addLine("Done with Shooting 1, deplacement vers premiere rangée");
                    // transition to next state
                    follower.followPath(driveShoot2pickup1Pos ,0.8, false); // chemin d'alignement de la premiere rangée
                    setPathState(PathState.intakeballeRangee1); // on va a l'étape suivante
                }
                break;

            case intakeballeRangee1:
                // mise à jour des systèmes
                intake.update();
                indexeur.update();

                if (!follower.isBusy()) {// attendre que le path soit fini
                    follower.followPath(driveAvalerpremiereLigne,0.40,false); // on avance doucement pour avaler les balles
                    setPathState(PathState.DrivedeuxiemeShoot);
                    }
                break;

            case DrivedeuxiemeShoot:
                ;
                if (!follower.isBusy()) { // Attendre que l'on est fini d'avoir pris toutes les balles
                    follower.followPath(DrivedeuxiemeShoot,0.60,true);
                    // Le robot est arrivé en position de tir :
                    setPathState(PathState.deuxiemetir);
                    tireurManager.prespinShooter(4000);
                }
                break;

            case deuxiemetir:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>1.5) {
                    if (!shotsTriggered) { // deuxieme période de tir
                        tireurManager.stopPrespin();
                        tireurManager.startTirAuto(// Lancer tir automatique
                                -8,   // angle tourelle (exemple)
                                0.48,  // angle shooter
                                4080   // RP
                                // M
                        );
                        shotsTriggered = true;
                    } else if (shotsTriggered && !tireurManager.isBusy()) {
                        setPathState(PathState.align_rangee2blue);
                        shotsTriggered = false;
                    }
                }
                break;

            case align_rangee2blue: // alignement avec la deuxieme zo
                // ne de balle (centrale)
                intake.update(); // mise à jour de nos systemes (constate que toutes les balles sont parties)
                indexeur.update();
                //if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>5) {
                if (!follower.isBusy()) {
                    follower.followPath(drivetorangee2,0.7, false);
                // TO DO demarer intake , tourner indexeur des dectetion balles)
                telemetry.addLine("alignement ramassage ligne 2");
                // transition to next state
                setPathState(PathState.intakerange2);
                }
                break;

            case intakerange2:
                intake.update(); // mise à jour de nos systemes (constate que toutes les balles sont parties)
                indexeur.update();
                if (!follower.isBusy()) {
                    follower.followPath(drivetavalerdeuxiemeligne, 0.38 , false);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("ramassage 2 terminé");
                    // transition to next state
                    setPathState(PathState.DriveTroisiemeTir);
                    }
                break;


            case DriveTroisiemeTir:
                intake.update(); // mise à jour de nos systemes (constate que toutes les balles sont parties)
                indexeur.update();
                if (!follower.isBusy()) {
                    follower.followPath(driveAvaler2emeLignetotroisemeShoot,0.60, true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("Position 3 de tir");
                    // transition to next state
                    setPathState(PathState.troisiemetir);
                    tireurManager.prespinShooter(4000);
                }
                break;

            case troisiemetir:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>1.5) {
                    TeleOpDecode.startingPose = follower.getPose();
                    // le robot est arrivé sur la troisieme position de tir :

                    if (!shotsTriggered){
                        tireurManager.stopPrespin();
                        tireurManager.startTirAuto(// Lancer tir automatique
                                -12,   // angle tourelle (exemple)
                                0.48,  // angle shooter
                                4080   // RPM
                        );
                        shotsTriggered = true;}
                    else if (shotsTriggered && !tireurManager.isBusy()){;
                            setPathState(PathState.atgate);
                            shotsTriggered = false;
                        }



                }
                break;
            case Drive2Gate: // pas utiliser sur la partie Goal
                intake.update(); // mise à jour de nos systemes (constate les balles sont tirées )
                indexeur.update();
                // shoot logique 3eme Tir
                if (!follower.isBusy()) {
                    follower.followPath(DrivetoGate,1,true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("Auto Termine & A cote de la porte ");
                    // transition to next state
                    setPathState(PathState.atgate);
                }
                break;

                
            case atgate:
                tourelle.allerVersAngle(35);;

                telemetry.addLine("C'est fini, position enregitrée");
                break;
        }


    }

    public void setPathState (PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;

    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSITIONTOSHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

        tourelle = new SpinTurret();
        tourelle.init(hardwareMap);

        // --- Initialisation hardware ---
        shooter = new Shooter();
        shooter.init(hardwareMap);


        ServoAngleShoot = new AngleShooter();
        ServoAngleShoot.init(hardwareMap);

        indexeur = new Indexeur();
        indexeur.init(hardwareMap);

        afficheurLeft = new AfficheurLeft();
        afficheurLeft.init(hardwareMap);

        afficheurRight = new AfficheurRight();
        afficheurRight.init(hardwareMap);

        intake = new Intake(indexeur, afficheurLeft);
        intake.init(hardwareMap);
        indexeur.setIntake(intake);

        servoTireur = new ServoTireur(indexeur);
        servoTireur.init(hardwareMap);

        // --- TireurManager ---
        tireurManager = new TireurManager(shooter, tourelle, ServoAngleShoot, servoTireur, indexeur, intake, afficheurRight);

        indexeur.setBalles(3);
        indexeur.resetCompartiments();


    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);

    }
    @Override
    public void loop (){
        follower.update();
        statePathUpdate();
        intake.update();
        indexeur.update();
        tireurManager.update();

        telemetry.addData("path state", pathState.toString());
        //telemetry.addData("x",follower.getPose().getX());
        //telemetry.addData("y",follower.getPose().getY());
        //telemetry.addData("heading",follower.getPose().getHeading());
        //telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        //telemetry.addData("angle Tourelle actuel", tourelle.lectureangletourelle());
        //telemetry.addData("RPM", intake.getRPM());
        //telemetry.addData("DistanceBalle", intake.getCapteurDistance());
        //telemetry.addData("Lum Indexeur", intake.getLumIndexeur());
        //telemetry.addData("Score", intake.getScore());
        //telemetry.addData("État Indexeur", indexeur.getEtat());
        //telemetry.addData("Pale detectée", indexeur.detectionpale());
        //telemetry.addData("Nombre de balles", indexeur.getBalles());
        //for (int i = 0; i < 3; i++) { telemetry.addData("Compartiment " + i, indexeur.getCouleurCompartiment(i)); }
        //telemetry.addData("État tireur manager", tireurManager.getState());
        //telemetry.addData("État indexeur", indexeur.getEtat());
        //telemetry.addData("Etat de l'intake", intake.getEtat());
        telemetry.addData("Shooter RPM", shooter.getShooterVelocityRPM());
        //telemetry.addData("Servo pos", servoTireur.getPosition());
        //telemetry.addData("Index rotation finie", indexeur.isRotationTerminee());

        telemetry.update();
    }

    public void stop() {

        TeleOpDecodeBleuCamera.startingPose = follower.getPose();
        GlobalPoseStorage.turretAngleDeg = tourelle.lectureangletourelle();

    }

}
