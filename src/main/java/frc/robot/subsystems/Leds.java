package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDs;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.utils.RobotStatus;


public class Leds extends SubsystemBase {
  /* Creates a new leds */

  private static RobotStatus state = RobotStatus.DEFAULT;
  private static RobotStatus prevState = RobotStatus.DEFAULT;
  public static int LEDstate = 0;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;

  private int temp_Blue = 0;
  private int temp_Green = 0;
  private int temp_Red = 0;
  private int blueStreakLED = 0;
  private int numLoops = 0;
  private int index = 0;
  private int index1 = 1; //this varible should never be less than 1

  public Leds() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(200); // 200 for new bot
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {


    
  }

  public void ledsOff() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
    m_led.setData(m_ledBuffer);
  }

  public void rainbow() {
    // For every pixel
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;

    m_led.setData(m_ledBuffer);
  }

  public void pantherStreak() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, LEDs.purple_Red, LEDs.purple_Green, LEDs.purple_Blue);
    }
    for (int i = index; i < m_ledBuffer.getLength(); i += 2) {
      m_ledBuffer.setRGB(i, LEDs.yellow_Red, LEDs.yellow_Green, LEDs.yellow_Blue);
    }

    if (numLoops % 10 == 0) {
      index++;
      index %= 2;
      numLoops = 0;
    }

    numLoops++;

    m_led.setData(m_ledBuffer);
  }

  public void purpleStreak() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, LEDs.purple_Red, LEDs.purple_Green, LEDs.purple_Blue);
    }
    for (int i = index; i < m_ledBuffer.getLength(); i += 2) {
      m_ledBuffer.setRGB(i, LEDs.purple_Red / 4, LEDs.purple_Green / 4, LEDs.purple_Blue / 4);
    }

    if (numLoops % 10 == 0) {
      index++;
      index %= 2;
      numLoops = 0;
    }

    numLoops++;

    m_led.setData(m_ledBuffer);
  }

  public void purpleFlash(){
    for(int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, temp_Red, temp_Green, temp_Blue);
    }

    if (numLoops < 5){ //sets speed of flash
      temp_Red = LEDs.purple_Red;
      temp_Blue = LEDs.purple_Blue;
      temp_Green = LEDs.purple_Green;
    }
    else{
      temp_Red = 0;
      temp_Blue = 0;
      temp_Green = 0;
    }

    numLoops %= 10;

  }

  public void yellowFlash(){
    for(int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, temp_Red, temp_Green, temp_Blue);
    }

    if (numLoops < 5){ //sets speed of flash
      temp_Red = LEDs.yellow_Red;
      temp_Blue = LEDs.yellow_Blue;
      temp_Green = LEDs.yellow_Green;
    }
    else{
      temp_Red = 0;
      temp_Blue = 0;
      temp_Green = 0;
    }

    numLoops %= 10;

  }


  public void yellowStreak() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i,LEDs.yellow_Red, LEDs.yellow_Green, LEDs.yellow_Blue);
    }
    for (int i = index; i < m_ledBuffer.getLength(); i += 2) {
      m_ledBuffer.setRGB(i, LEDs.yellow_Red / 4, LEDs.yellow_Green / 4 , LEDs.yellow_Blue / 4);
    }

    if (numLoops % 10 == 0) {
      index++;
      index %= 2;
      numLoops = 0;
    }

    numLoops++;

    m_led.setData(m_ledBuffer);
  }

  public void red() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, LEDs.Red, 0, 0);//red
    }

    m_led.setData(m_ledBuffer);
  }

  public void blue() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, LEDs.Blue);//blue
    }

    m_led.setData(m_ledBuffer);
  }

  public void green() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for green
      m_ledBuffer.setRGB(i, 0, LEDs.Green, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  public void yellow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for yellow
      m_ledBuffer.setRGB(i, LEDs.yellow_Red, LEDs.yellow_Green, LEDs.yellow_Blue);
    }

    m_led.setData(m_ledBuffer);
  }

  public void orange() {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for orange
      m_ledBuffer.setRGB(i, 255, 165, 0);
    }

    m_led.setData(m_ledBuffer);
  }


  public void purple() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for purple
      m_ledBuffer.setRGB(i, LEDs.purple_Red, LEDs.purple_Blue, LEDs.purple_Green);
    }

    m_led.setData(m_ledBuffer);
  }

  public void white() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for white
      m_ledBuffer.setRGB(i, LEDs.Red, LEDs.Green, LEDs.Blue);
    }

    m_led.setData(m_ledBuffer);
  }

  public void bluePulse() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for blue
      m_ledBuffer.setRGB(i, 0, 0, temp_Blue);
    }

    // increase brightness
    temp_Blue += 5;

    // Check bounds
    temp_Blue %= 255;

    m_led.setData(m_ledBuffer);
  }

  public void greenPulse() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for blue
      m_ledBuffer.setRGB(i, 0, temp_Green, 0);
    }

    // increase brightness
    temp_Green += 5;

    // Check bounds
    temp_Green %= 255;

    m_led.setData(m_ledBuffer);
  }

  public void orangePulse() {

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for orange
      m_ledBuffer.setRGB(i, temp_Red, temp_Green, 0);
    }

    // increase brightness
    temp_Red += 5;
    temp_Green += 1;

    // Check bounds
    temp_Red %= 255;
    temp_Green %= 51;

    m_led.setData(m_ledBuffer);
  }

  public void blueStreak() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for blue
      m_ledBuffer.setRGB(i, 0, 0, LEDs.Blue);
    }

    // turns one led off
    m_ledBuffer.setRGB(blueStreakLED, 0, 0, 0);

    // increase brightness
    if (numLoops % 3 == 0) {
      blueStreakLED += 1;

      // Check bounds
      blueStreakLED %= m_ledBuffer.getLength();
    }

    m_led.setData(m_ledBuffer);

    numLoops += 1;

  }

public void purpleStreak10() {
    for (int i = index1; i < index1 + 10; i++) {
      m_ledBuffer.setRGB(i, LEDs.purple_Red, LEDs.purple_Blue, LEDs.purple_Green);
    }

    m_ledBuffer.setRGB(index1 - 1, 0,0,0);

    // increase brightness
    if (numLoops % 7 == 0) {
      index1++;
      numLoops = 0;

      // Check bounds
      index1 = index1 % (m_ledBuffer.getLength() - 20) + 1;
    }

    m_led.setData(m_ledBuffer);

    numLoops++;

  }

  public void PurpleWhiteStrobe(){
    if (numLoops == 0 || numLoops == 2){
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0); //black
      }
    }
    else if (numLoops == 1){
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, LEDs.Red, LEDs.Green, LEDs.Blue);
      }
    }
    else{
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, LEDs.purple_Red, LEDs.purple_Blue, LEDs.purple_Green);
      }
    }

    numLoops++;
    numLoops %= 4;
  }

  public void PinkYellowStreak(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 245, 34,195); //pink
      if (i % 2 == index){
        m_ledBuffer.setRGB(i, 239, 239, 32); //yellow
      }
      
    }
  /*  for (int i = index; i < m_ledBuffer.getLength(); i += 2) {
      m_ledBuffer.setRGB(i, 239, 239, 32);
    }
*/
    if (numLoops % 10 == 0) {
      index++;
      index = index % 2;
      numLoops = 0;
    }

    numLoops++;

    m_led.setData(m_ledBuffer);


  }


  public void snakeStreak()
  {
      for (int i = index; i < m_ledBuffer.getLength() - 4; i++) { //set color pattern of the leds  
          m_ledBuffer.setRGB(i, 148, 0, 211); //purple        

      /*add one every loop, if it reaches 4, make it go back to 0
      This switches the postions of the leds for each color, makes it looks like colors are snaking around bot*/
          if (numLoops % 10 == 0) {
              index++;
              index = index % 2;
              numLoops = 0;
            }

    }
  }

  public void setRobotStatus(RobotStatus newState){
    this.prevState = this.state;
    this.state = newState;
  }

public RobotStatus getRobotStatus(){
  return this.state;
}

  public RobotStatus getPrevRobotStatus(){
    return this.prevState;
  }

  public void ledState(){
      
    switch(this.state){
      
      case TARGET_LOCK: white(); break;
      case LAUNCH: blue(); break;
      case NOTE_STORED: orange(); break;
      case INTAKE: if(RobotContainer.feeder.isNoteDetected()){orangePulse();break;} greenPulse(); break;
      case 1: purpleFlash(); break;
      case 2: yellowFlash(); break;
      case 3: purpleStreak10(); break;
      case 4: yellowStreak(); break;
      case 5: red(); break;
      case ROBOT_CENTRIC: rainbow(); break;
      case DEFAULT: pantherStreak(); break;
  }


} 

