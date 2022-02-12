/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Ashirbad Pradhan, Ana Lucia
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;
int               index = 0;
float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

//define ball location
float             g_x=5.0;
float             g_y=8.0; 
float             mags;
float             sgn_x;
float             sgn_y;
float             f_x;
float             f_y;

/* define maze blocks */
FBox              l1;

/* define start and stop button */
FCircle           c1;
FCircle           c2;

/* define game ball */
FBox              g1;
FBox              g2;
FCircle           g3;
FCircle           g4;
FCircle           g5;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1200, 600);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  /* Game Box */
  g1                  = new FBox(12, 5);
  g1.setSensor(true);
  g1.setStatic(true);
  //g1.setStrokeWeight(2);
  g1.setPosition(23, 8);
  g1.setDensity(100);
  g1.setFill(0,230,255,80);
  g1.setName("Widget");
  world.add(g1);
  
  
  /* Game Ball */
  g2                  = new FBox(6, 5);
  g2.setPosition(12, 8);
  g2.setDensity(100);
  g2.setFill(230,230,0);
  g2.setName("Widget");
  g2.setSensor(true);
  g2.setStatic(true);
  world.add(g2);
  

    /* Game Ball */
  g3                  = new FCircle(7);
  g3.setPosition(g_x, g_y);
  g3.setDensity(100);
  g3.setFill(0,170,255);
  g3.setName("Widget");
  g3.setSensor(true);
  g3.setStatic(true);
  g3.addForce(4,4, 3,0);
  world.add(g3);
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  

 
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
 
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
  
      float shake = random(-4, 4);
      float x_dist;
      float y_dist;
      float k = 7;
    if (s.h_avatar.isTouchingBody(g1)){
      s.h_avatar.setDamping(800);         
      fEE.set(-s.getVirtualCouplingForceX()/100000-2, sin((-s.getVirtualCouplingForceX()/100000)*TWO_PI/1.5)*1.8);
    }else if(s.h_avatar.isTouchingBody(g2)){
      fEE.set(-s.getVirtualCouplingForceX()/100000 + 3*shake, s.getVirtualCouplingForceY()/100000 + 0.03*shake);
    } else if (s.h_avatar.isTouchingBody(g3)){   
      x_dist = s.getToolPositionX()-g_x;
      y_dist = s.getToolPositionY()-g_y;
      sgn_x=x_dist/abs(x_dist);
      sgn_y=y_dist/abs(y_dist);
      f_x= k*1/(x_dist+3.5*sgn_x);
      f_y= k*1/(y_dist+3.5*sgn_y)*-1; 
      if(abs(y_dist) < 0.5){
        f_y = 0;     
        f_x=1.16*f_x;
      } 
      if (abs(x_dist) < 0.5){
        f_x = 0;   
        f_y=1.16*f_y;
      }
      if(abs(x_dist) < 0.5 && abs(y_dist) < 0.5){
        f_x = 0.03*f_x; f_y =0.03*f_y;
      }
      println(f_x, f_y);        
      fEE.set(-s.getVirtualCouplingForceX()/100000 + f_x, s.getVirtualCouplingForceY()/100000 + f_y);
    }else{
      fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());  
      fEE.div(100000);
    }
     //dynes to newtons      
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();    
    world.step(1.0f/1000.0f);  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
