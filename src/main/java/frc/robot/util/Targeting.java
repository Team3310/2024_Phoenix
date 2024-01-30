package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Targeting{
    
    //Targeting Constructor, Limelight 
    public Targeting(String limelightHostname, boolean isOdometry){
        numberOfTargetingObjects++;
        this.limelightHostname = "-" + limelightHostname;
        this.isOdometry = isOdometry;
        TargetingObjectList.add(this);
    }

    //Targeting Constructor, Odometry
    public Targeting(boolean isOdometry){
        numberOfTargetingObjects++;
        this.isOdometry = isOdometry;
        TargetingObjectList.add(this);
        if (!isOdometry){
            System.err.println("TARGETING ERROR: ODOMETRY CONSTRUCTOR USED WITH isOdometry SET TO FALSE");
        }
    }

    //STATIC 
    private static int numberOfTargetingObjects = 0;
    private static ArrayList<Targeting> TargetingObjectList = new ArrayList<Targeting>();
    private static final int movingAveragePeriod = 5;
    private static ArrayList<Double> movingAverageAzList = new ArrayList<Double>();
    private static ArrayList<Double> movingAverageElList = new ArrayList<Double>();
    private static double movingAverageAz;
    private static double movingAverageEl;
    private static double[] targetPos = new double[3]; //Translation (X, Y, Z)
    private static Target target = Target.None;
    public static enum Target{
        BluSpeaker,
        BluAmp,
        RedSpeaker,
        RedAmp,
        None
    }
    
    //Cartesian Coordinates of Targets 
    //Origin is CENTER OF FIELD
    //X, Y, Z (Inches)
    private static final double[] redSpeaker =   { 7.9674974, 1.4422628, 2.0432395};
    private static final double[] bluSpeaker =   {-7.9387446, 1.4422628, 2.0432395}; 
    private static final double[] redAmp =       {-6.4147446, 4.0985948, 0.8891270};
    private static final double[] bluAmp =       { 6.4445134, 4.0985948, 0.8891270};
    private static final double[] centerOfField ={ 0.0000000, 0.0000000, 0.0000000};

    //Set targetPos and set target, must be ran first, target initialized to None
    public static void setTarget(Target toTarget){
        switch(toTarget){
                case RedSpeaker:  targetPos = redSpeaker; 
                                    break;
                case BluSpeaker:  targetPos = bluSpeaker;
                                    break;
                case RedAmp:  targetPos = redAmp;
                                break;
                case BluAmp:  targetPos = bluAmp;
                                break;
                default:    targetPos = centerOfField;
                            break;
            }
        target = toTarget;    
    }

    //Get ENUM of Target
    public static Target getTarget(){
        return target;
    }

    public static double getMovingAverageAz(){
        return movingAverageAz;
    }

    public static double getMovingAverageEl(){
        return movingAverageEl;
    }

    //Run .update() for all Targeting objects
    public static void updateAll(){
        for (Targeting TargetingObject : TargetingObjectList){
            TargetingObject.update();
        }

        double averageAz = 0;
        double averageEl = 0;
        int count = 0;

        //Get an Average Az/El across all TargetingObjects
        for (Targeting TargetingObject : TargetingObjectList){
            if (TargetingObject.latestUpdateSuccesfull){
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
            movingAverageAz = sumAz / movingAverageAzList.size();
            movingAverageEl = sumEl / movingAverageElList.size();
        }
    }






    //NON-STATIC
    private double[] botPos = new double[6]; //Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
    private double targetAz = 0; //Az (Radians)
    private double targetEl = 0; //El (Radians)
    private boolean latestUpdateSuccesfull = false;
    private String limelightHostname = "";
    private boolean isOdometry = false;

    //Update botPos array for Targeting Object
    public void updateBotPos(){
        //Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        
        //Update botPos via Odometry
        if(isOdometry){
            botPos[0] = TunerConstants.DriveTrain.getPose().getX();
            botPos[1] = TunerConstants.DriveTrain.getPose().getY();
            botPos[2] = 0.0;
        }else{ 
            try {
                //Update botPos via Limelight
                botPos = NetworkTableInstance.getDefault().getTable("limelight" + limelightHostname).getEntry("botpose").getDoubleArray(new double[6]);
            } catch (Exception e) {
                System.err.println("Targeting Error: updateBotPos failed to get Limelight Data");
            }
        }
    }

    //Update targetAz and targetEl for Targeting Object
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
        double delta_X = targetPos[0] - botPos[0];
        double delta_Y = targetPos[1] - botPos[1];
        double delta_Z = targetPos[2] - botPos[2];

        //If Limelight cannot see any targets... don't calculate, and change latestUpdateSuccesfull to FALSE
        latestUpdateSuccesfull = true;
        if (!this.hasTarget()){
            latestUpdateSuccesfull = false;
            return;
        }

        //Azimuth Trig
        if (delta_Y > 0){                                                //(pi/2 to -pi/2), (90 deg to -90 deg))
            targetAz = Math.atan(delta_X / delta_Y);              
        }else if ((delta_X > 0) && (delta_Y <= 0)){                      //[pi/2 to pi)
            targetAz = -(Math.atan(delta_Y / delta_X)) + (Math.PI / 2); 
        }else if ((delta_X < 0) && (delta_Y <= 0)){                      //[-pi/2 to -pi)
            targetAz = -(Math.atan(delta_Y / delta_X)) - (Math.PI / 2); 
        }else if ((delta_X == 0) && (delta_Y < 0)){                        //Edge case: pointing along Y-Axis, Y-: pi (180 deg)                         
            targetAz = Math.PI;  
        }else if ((delta_X == 0) && (delta_Y == 0)){                     //This should never occur in a realistic scenario, will not update AZ.
            latestUpdateSuccesfull = false;
        }else{                                                           //No cases ran, should never occur
            latestUpdateSuccesfull = false;
        }

        //Elevation Trig
        double distance_XY = Math.hypot(delta_X, delta_Y);
        if (distance_XY != 0){ 
            targetEl = Math.atan(delta_Z / distance_XY);
        }else {
            latestUpdateSuccesfull = false;   
        }
    }

    //Returns if Limelight has view of an AprilTag, if Targeting Object is an Odometry object, will always return True
    public boolean hasTarget(){
        if(isOdometry){
            return true;
        }else{
            return NetworkTableInstance.getDefault().getTable("limelight" + limelightHostname).getEntry("tv").getBoolean(false);
        }
    }

    //Get Targeting Object targetAz
    public double getAz(){
        //Flipped sign?
        return -targetAz;
    }
    //Get Targeting Object targetEl
    public double getEl(){
        return targetEl;
    }

    //Update Targeting Object
    public void update(){
        updateBotPos();
        updateTargetAzEl();   
    }
 
}

    
