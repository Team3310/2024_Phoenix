package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.generated.TunerConstants;

public class Targeting{
    //Cartesian Coordinates of Targets 
    //Origin is CENTER OF FIELD
    //X, Y, Z (Inches)
    private static final double[] redSpeaker =      { 7.9674974, 1.4422628, 2.0432395};
    private static final double[] blueSpeaker =     {-7.9387446, 1.4422628, 2.0432395}; 
    private static final double[] redAmp =          {-6.4147446, 4.0985948, 0.8891270};
    private static final double[] blueAmp =         { 6.4445134, 4.0985948, 0.8891270};
    private static final double[] centerOfField =   { 0.0000000, 0.0000000, 0.0000000};

    public enum Target{
        BLUESPEAKER(blueSpeaker),
        BLUAMP(blueAmp),
        REDSPEAKER(redSpeaker),
        REDAMP(redAmp),
        NONE(centerOfField);

        private final double[] pos;

        private Target(double[] toTargetPos){
            this.pos = toTargetPos;
        }

        public double[] getPos(){
            return pos;
        }
    }
    
    //Targeting Constructor, Limelight 
    public Targeting(String limelightHostname, boolean isOdometry){
        numberOfTargetingObjects++;
        this.limelightHostname = "-" + limelightHostname;
        this.limelight = new Limelight(limelightHostname);
        this.isOdometry = isOdometry;
        TargetingObjectList.add(this);
    }

    //Targeting Constructor, Odometry
    public Targeting(boolean isOdometry){
        numberOfTargetingObjects++;
        this.isOdometry = isOdometry;
        this.limelight = new Limelight();
        TargetingObjectList.add(this);
        if (!isOdometry){
            System.err.println("TARGETING ERROR: ODOMETRY CONSTRUCTOR USED WITH isOdometry SET TO FALSE");
        }
    }

    //STATIC 
    private static int numberOfTargetingObjects = 0;
    private static List<Targeting> TargetingObjectList = new ArrayList<Targeting>();
    private static final int movingAveragePeriod = 5;
    private static List<Double> movingAverageAzList = new ArrayList<Double>();
    private static List<Double> movingAverageElList = new ArrayList<Double>();
    private static double targetAz_averageAzEl;
    private static double targetEl_averageAzEl;
    public static double[] averageBotPos = new double[6];
    public static double[] movingAverage_averageBotPos = new double[6];
    private static List<double[]> movingAverage_averageBotPosList = new ArrayList();
    private static double targetAz_averageBotPos;
    private static double targetEl_averageBotPos;
    private static double[] targetPos = centerOfField;
    private static Target target = Target.NONE;
    private static boolean latestCalc_ABP_UpdateSuccesfull = false;
    
    public static void setTarget(Target toTarget){
        target = toTarget;
        targetPos = target.getPos();
    }

    public static Target getTarget(){
        return target;
    }

    public static double getTargetAz_averageAzEl(){
        return targetAz_averageAzEl;
    }

    public static double getTargetEl_averageAzEl(){
        return targetEl_averageAzEl;
    }

    public static double getTargetAz_averageBotPos(){
        return targetAz_averageBotPos;
    }

    public static double getTargetEl_averageBotPos(){
        return targetEl_averageBotPos;
    }

    public static void updateAll_averageBotPos(){
        List<double[]> botPosList = new ArrayList();
        for (Targeting TargetingObject : TargetingObjectList){
            TargetingObject.updateBotPos();

            
            if(TargetingObject.latestBotPosUpdateSuccesfull){
                botPosList.add(TargetingObject.botPos);
            }
        }
        if(botPosList.size() != 0){
            double[] averageXYZ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            
            for(double[] aBotPos : botPosList){
                for(int i = 0; i < 6; i++){
                    averageXYZ[i] += aBotPos[i]; 
                }
            }
            for(int i = 0; i < 6; i++){
                averageXYZ[i] /= botPosList.size(); 
            }
            averageBotPos = averageXYZ;

            if(movingAverage_averageBotPosList.size()>movingAveragePeriod){
                movingAverage_averageBotPosList.remove(0);
            }
            movingAverage_averageBotPosList.add(averageBotPos);
            
            if(movingAverage_averageBotPosList.size() != 0){
                double temp_movingAverage_averageBotPos[] = new double[6];
                for(double[] anAverageBotPos : movingAverage_averageBotPosList){
                    for(int i = 0; i < 6; i++){
                        temp_movingAverage_averageBotPos[i] += anAverageBotPos[i];
                    }
                }
                for(int i = 0; i < 6; i++){
                    temp_movingAverage_averageBotPos[i] /= movingAverage_averageBotPosList.size();
                }
                movingAverage_averageBotPos = temp_movingAverage_averageBotPos;

                Targeting.updateTargetAzEl_averageBotPos();
            
            }            
        }
    }

    private static void updateTargetAzEl_averageBotPos(){
        //Compute the X, Y, and Z distances between coordinates
        double delta_X = targetPos[0] - averageBotPos[0];
        double delta_Y = targetPos[1] - averageBotPos[1];
        double delta_Z = targetPos[2] - averageBotPos[2];

        //Azimuth Trig
        if (delta_Y > 0){
            targetAz_averageBotPos = Math.atan(delta_X / delta_Y);              
        }else if ((delta_X > 0) && (delta_Y <= 0)){
            targetAz_averageBotPos = -(Math.atan(delta_Y / delta_X)) + (Math.PI / 2); 
        }else if ((delta_X < 0) && (delta_Y <= 0)){
            targetAz_averageBotPos = -(Math.atan(delta_Y / delta_X)) - (Math.PI / 2); 
        }else if ((delta_X == 0) && (delta_Y < 0)){                      
            targetAz_averageBotPos = Math.PI;  
        }else if ((delta_X == 0) && (delta_Y == 0)){
            latestCalc_ABP_UpdateSuccesfull = false;
        }else{
            latestCalc_ABP_UpdateSuccesfull = false;
        }

        //Elevation Trig
        double distance_XY = Math.hypot(delta_X, delta_Y);
        if (distance_XY != 0){ 
            targetEl_averageBotPos = Math.atan(delta_Z / distance_XY);
        }else {
            latestCalc_ABP_UpdateSuccesfull = false;   
        }
    }
    
    public static void updateAll_averageElAz(){
        for (Targeting TargetingObject : TargetingObjectList){
            TargetingObject.update();
        }

        double averageAz = 0;
        double averageEl = 0;
        int count = 0;

        //Get an Average Az/El across all TargetingObjects
        for (Targeting TargetingObject : TargetingObjectList){
            if (TargetingObject.latestCalcUpdateSuccesfull){
                averageAz += TargetingObject.getAz();
                averageEl += TargetingObject.getEl();
                count++;
            }
        }
        //Only continue, if at least one Targeting Object succesfully updated
        if(count != 0){
            averageAz /= count;
            averageEl /= count;

            if(movingAverageAzList.size()>movingAveragePeriod){
                movingAverageAzList.remove(0);
            }
            if(movingAverageElList.size()>movingAveragePeriod){
                movingAverageElList.remove(0);
            }

            movingAverageAzList.add(averageAz);
            movingAverageElList.add(averageEl);

            double sumAz = 0;
            double sumEl = 0;
            for (double az : movingAverageAzList){
                sumAz += az;
            }
            for (double el : movingAverageAzList){
                sumEl += el;
            }
            targetAz_averageAzEl = sumAz / movingAverageAzList.size();
            targetEl_averageAzEl = sumEl / movingAverageElList.size();
        }
    }

    //NON-STATIC
    private double[] botPos = centerOfField;
    private double targetAz = 0;
    private double targetEl = 0;
    private boolean latestCalcUpdateSuccesfull = false;
    private boolean latestBotPosUpdateSuccesfull = false;
    private String limelightHostname;
    private boolean isOdometry = false;
    private final Limelight limelight;

    public void updateBotPos(){
        if(isOdometry){
            botPos[0] = TunerConstants.DriveTrain.getPose().getX();
            botPos[1] = TunerConstants.DriveTrain.getPose().getY();
            botPos[2] = 0.0;
        }else{ 
            try {
                // botPos = NetworkTableInstance.getDefault().getTable("limelight" + this.limelightHostname).getEntry("botpose").getDoubleArray(new double[6]);
                botPos = limelight.getBotPose();
                if (botPos[0] != 0){
                    latestBotPosUpdateSuccesfull = true;
                }else{
                    latestBotPosUpdateSuccesfull = false;
                }

                if (botPos.length < 3){
                    System.err.println(botPos.length);
                    System.err.println("Targeting Error: " + limelightHostname);
                    System.err.println("Targeting Error: updateBotPos failed, Limelight gave BAD data, zeroed botPos");
                    botPos = new double[]{0, 0, 0, 0, 0, 0};
                }
            
            } catch (Exception e) {
                System.err.println("Targeting Error: " + limelightHostname);
                System.err.println("Targeting Error: updateBotPos failed to get Limelight Data");
                botPos = new double[]{0, 0, 0, 0, 0, 0};
            }
        }
    }

    public void updateTargetAzEl(){
        //AZIMUTH
        //-----------------------------------------------------------------------------------------------------------------------
        //0 degrees is in the fields Y Positive Direction
        //pi/2 (90 degrees) is in the fields X Positive Direction
        //pi (180 degrees) is in the fields Y Negative Direction
        //-pi/2 (-90 degrees) is in the fields X Negative Direction
        //Range is from pi to ~-pi (180 to -179.999)

        //ELEVATION
        //-----------------------------------------------------------------------------------------------------------------------
        //0 degrees is along the XY plane
        //90 degrees is in the fields Z Positive Direction
        //-90 degrees is in the fields Z Negative Direction
        //Range  is from pi/2 to -pi/2 (180 to -180)
        
        //Compute the X, Y, and Z distances between coordinates
        double delta_X = targetPos[0] - this.botPos[0];
        double delta_Y = targetPos[1] - this.botPos[1];
        double delta_Z = targetPos[2] - this.botPos[2];

        // SmartDashboard.putString("delta x,y,z", delta_X+","+delta_Y+","+delta_Z);
        // SmartDashboard.putString("bot x,y,z", botPos[0]+","+botPos[1]+","+botPos[2]);
        // SmartDashboard.putString("target x,y,z", targetPos[0]+","+targetPos[1]+","+targetPos[2]);

        //If Limelight cannot see any targets... don't calculate, and change latestUpdateSuccesfull to FALSE
        latestCalcUpdateSuccesfull = true;
        if (!this.hasTarget()){
            // System.err.println("No Target");
            this.latestCalcUpdateSuccesfull = false;
            return;
        }

        //Azimuth Trig
        if (delta_Y > 0){                                                //(pi/2 to -pi/2), (90 deg to -90 deg))
            this.targetAz = Math.atan(delta_X / delta_Y);              
        }else if ((delta_X > 0) && (delta_Y <= 0)){                      //[pi/2 to pi)
            this.targetAz = -(Math.atan(delta_Y / delta_X)) + (Math.PI / 2); 
        }else if ((delta_X < 0) && (delta_Y <= 0)){                      //[-pi/2 to -pi)
            this.targetAz = -(Math.atan(delta_Y / delta_X)) - (Math.PI / 2); 
        }else if ((delta_X == 0) && (delta_Y < 0)){                        //Edge case: pointing along Y-Axis, Y-: pi (180 deg)                         
            this.targetAz = Math.PI;  
        }else if ((delta_X == 0) && (delta_Y == 0)){                     //This should never occur in a realistic scenario, will not update AZ.
            this.latestCalcUpdateSuccesfull = false;
        }else{                                                           //No cases ran, should never occur
            this.latestCalcUpdateSuccesfull = false;
        }

        // SmartDashboard.putNumber("target azi in method", targetAz);

        //Elevation Trig
        double distance_XY = Math.hypot(delta_X, delta_Y);
        if (distance_XY != 0){ 
            this.targetEl = Math.atan(delta_Z / distance_XY);
        }else {
            this.latestCalcUpdateSuccesfull = false;   
        }
    }

    public boolean hasTarget(){
        if(isOdometry){
            return true;
        }else{
            return limelight.hasTarget();
        }
    }

    public double getAz(){
        return this.targetAz;
    }
    
    public double getEl(){
        return this.targetEl;
    }

    public void update(){
        updateBotPos();
        updateTargetAzEl();   
    }
}

    
