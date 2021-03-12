/**
 **********************************************************************************************************************
 * @file       sketch_4_Wall_Physics.pde
 * @author     Preeti Vyas
 * @version    V4.1.0
 * @date       10-March-2021
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
import controlP5.*;

/* end library imports *************************************************************************************************/



/* scheduler definition ************************************************************************************************/
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/

ControlP5 cp5;


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
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
PVector           fEE                                = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 30.0;  
float             worldHeight                         = 15.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

int damp = 0;
int low = 250;
int medium = 400;
int high = 650;
int perturb = 900;
int sticky = 995;

/* Initialization of wall */
FBox              a1, b1, b2, b3;
Boolean novisual = true;

/* text font */
PFont             f;

Slider damper;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup() {
  /* put setup code here, run once: */

  /* screen size definition */
  size(1200, 680);

  f                   = createFont("Verdana", 20, true);

  /* GUI setup */
  smooth();
  cp5 = new ControlP5(this);

  PFont p = createFont("Verdana", 17); 
  ControlFont font = new ControlFont(p);

  // change the original colors
  cp5.setColorForeground(0xffaa0000);
  cp5.setColorBackground(color(255, 100, 100));
  cp5.setFont(font);
  cp5.setColorActive(0xffff0000);

  //slider setup for quick testing
  //damper = cp5.addSlider("damp")
  //  .setPosition(10, 605)
  //  .setSize(200, 30)
  //  .setRange(0, 1000) // values can range from big to small as well
  //  .setValue(0)
  //  ;


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
  haplyBoard          = new Board(this, "COM4", 0);
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

  /* bounding wall */

  b1                  = new FBox(10, 1);
  b1.setPosition(worldWidth/2, worldHeight/2-2);
  //b1.setFill(40, 62, 102);
  b1.setDensity(100);
  b1.setSensor(true);
  b1.setNoStroke()  ;
  b1.setStatic(true);
  world.add(b1);

  b2                  = new FBox(1, 5);
  b2.setPosition(worldWidth/3, worldHeight/2-4);
  //b2.setFill(40, 62, 102);
  b2.setDensity(100);
  b2.setSensor(true);
  b2.setNoStroke()  ;
  b2.setStatic(true);
  world.add(b2)     ;

  b3                  = new FBox(1, 5);
  b3.setPosition(2*worldWidth/3, worldHeight/2-4);
  //b3.setFill(40, 62, 102);
  b3.setDensity(100);
  b3.setSensor(true);
  b3.setNoStroke()  ;
  b3.setStatic(true);
  world.add(b3)     ;

  if (novisual == true) {
    background(255);
    b1.setFill(255, 255, 255);
    b2.setFill(255, 255, 255);
    b3.setFill(255, 255, 255);
  } else {
    background(255);
    b1.setFill(40, 62, 102);
    b2.setFill(40, 62, 102);
    b3.setFill(40, 62, 102);
  }

  //buttons
  cp5.addButton("One")
    .setValue(0)
    .setPosition(350, 605)
    .setSize(150, 50)
    ;
  cp5.addButton("Two")
    .setValue(0)
    .setPosition(550, 605)
    .setSize(150, 50)
    ;
  cp5.addButton("Three")
    .setValue(0)
    .setPosition(750, 605)
    .setSize(150, 50)
    ;

  damp = 0;

  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4);  
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); //

  /* world conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.5);
  world.setEdgesFriction(1);

  world.draw();


  /* setup framerate speed */
  frameRate(baseFrameRate);


  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/


public void One(int theValue) {
  damp = 0;
  resetWallSensor(false);
}
public void Two(int theValue) {
  damp = perturb;
  resetWallSensor(true);
}
public void Three(int theValue) {
  damp = sticky;
  resetWallSensor(true);
}

void resetWallSensor(boolean value){
  b1.setSensor(value);
  b2.setSensor(value);
  b3.setSensor(value);
}

/* draw section ********************************************************************************************************/
void draw() {
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if (renderingForce == false) {
    background(255);
    world.draw();
  }

  fill(255,100,100);
      textFont(f, 22);
      textAlign(CENTER);
      text("Move the Haply to fully closed position.", width/2, 450);
      textAlign(CENTER);
      text("Click the button using your mouse, explore the surface around you!", width/2, 480);
      textFont(f, 28);
      textAlign(CENTER);
      text("How do you feel?", width/2, 520);
}
/* end draw section ****************************************************************************************************/

//50, 300, 650

/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable {

  public void run() {
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */

    renderingForce = true;

    if (haplyBoard.data_available()) {
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));
    }

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 


    s.updateCouplingForce();
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons

    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();


    if (s.h_avatar.isTouchingBody(b1)) {
      s.h_avatar.setDamping(damp);
    } else if (s.h_avatar.isTouchingBody(b2)) {
      s.h_avatar.setDamping(damp);
    } else if (s.h_avatar.isTouchingBody(b3)) {
      s.h_avatar.setDamping(damp);
    } else {
      s.h_avatar.setDamping(10);
    }

    world.step(1.0f/1000.0f);
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/

/* end helper functions section ****************************************************************************************/
