package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//CHANGE!
public class Targeting{
    //Cartesian Coordinates of Targets 
    //Origin is CENTER OF FIELD
    //X, Y, Z (Inches)
    private final double[] redSpeaker =   { 7.9674974, 1.4422628, 2.0432395};
    private final double[] bluSpeaker =   {-7.9387446, 1.4422628, 2.0432395}; 
    private final double[] redAmp =       {-6.4147446, 4.0985948, 0.8891270};
    private final double[] bluAmp =       { 6.4445134, 4.0985948, 0.8891270};
    private final double[] centerOfField ={ 0.0000000, 0.0000000, 0.0000000};
    private double[] botPos = new double[6]; //Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
    private double[] targetPos = new double[3]; //Translation (X, Y, Z)
    private double targetAz = 0; //Az (Radians)
    private double targetEl = 0; //El (Radians)
    private Target target = Target.None;

    public void updateBotPos(){
        //update the botPos array by calling the limelight
        //Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
        try {
            botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
        } catch (Exception e) {
            System.err.println("Targeting Error: updateBotPos failed to get Limelight Data");
            //botPos = new double[]{0, 0, 0, 0, 0, 0};
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
        double delta_X = targetPos[0] - botPos[0];
        double delta_Y = targetPos[1] - botPos[1];
        double delta_Z = targetPos[2] - botPos[2];

        //Azimuth Trig
        if (delta_Y > 0){                                                //(pi/2 to -pi/2), (90 deg to -90 deg))
            targetAz = Math.atan(delta_X / delta_Y);
            SmartDashboard.putBoolean("update az failed", false);                   
        }else if ((delta_X > 0) && (delta_Y <= 0)){   
            SmartDashboard.putBoolean("update az failed", false);                   //[pi/2 to pi)
            targetAz = -(Math.atan(delta_Y / delta_X)) + (Math.PI / 2);
        }else if ((delta_X < 0) && (delta_Y <= 0)){                      //[-pi/2 to -pi)
            targetAz = -(Math.atan(delta_Y / delta_X)) - (Math.PI / 2);
            SmartDashboard.putBoolean("update az failed", false);
        }else if ((delta_X == 0) && (delta_Y < 0)){                        //Edge case: pointing along Y-Axis, Y-: pi (180 deg)                         
            targetAz = Math.PI;
            SmartDashboard.putBoolean("update az failed", false);
        }else if ((delta_X == 0) && (delta_Y == 0)){                     //This should never occur in a realistic scenario, will not update AZ.
            System.err.println("Targeting Error: delta_X, delta_Y BOTH ZERO, impossible target");
            SmartDashboard.putBoolean("update az failed", true);
        }else{                                                           //No cases ran, should never occur
            System.err.println("Targeting Error: Az Trig, no calculation completed");
            SmartDashboard.putBoolean("update az failed", true);
        }

        //Elevation Trig
        double distance_XY = Math.hypot(delta_X, delta_Y);
        if (distance_XY != 0){ 
            targetEl = Math.atan(delta_Z / distance_XY);
        }else {
            targetEl = 0;
        }
    }

    public void setTarget(Target target){
        switch(target){
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
        this.target = target;    
    }

    public double getAz(){
        return -targetAz;
    }

    public double getEl(){
        return targetEl;
    }

    public Target getTarget(){
        return target;
    }

    public void update(){
            updateBotPos();
            updateTargetAzEl();   
        }

        public enum Target{
            BluSpeaker,
            BluAmp,
            RedSpeaker,
            RedAmp,
            None
        }
    }

    
