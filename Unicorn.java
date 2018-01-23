package Unicorn;
import robocode.*;
import java.awt.*;
import robocode.*;
import robocode.Robot;
import robocode.ScannedRobotEvent;
import robocode.HitByBulletEvent;
import robocode.RateControlRobot;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import java.awt.Color;
/**
 * Unicorn - a robot by (your name here)
 */
public class Unicorn extends AdvancedRobot
{
  double previousEnergy = 100;
  int movementDirection = 1;
  int gunDirection = 1;
  int gunPower = 3;		
 
/**
	 * run: Unicorn's default behavior
	 */
	public void run() {
	setBodyColor(Color.MAGENTA);
	setGunColor(Color.YELLOW);
	setRadarColor(Color.CYAN);
	setBulletColor(Color.MAGENTA);
	setScanColor(Color.MAGENTA);
    
	setTurnGunRight(Double.POSITIVE_INFINITY);
	
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
	 // обработчик сканирования других роботов
      setTurnRight(e.getBearing()+90- 30*movementDirection);
        
     //если энергия уменьшается
    double changeInEnergy =  previousEnergy-e.getEnergy();
    if (changeInEnergy>0 &&   changeInEnergy<=3) {
         //меняем направление на противоположное
         movementDirection = -movementDirection;
         setAhead((e.getDistance()/4+30)*movementDirection);
     }
 
	
    gunDirection = -gunDirection; //направление удара

    setTurnGunLeft(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
	setTurnLeft(45);
	setTurnGunLeft(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
	setTurnLeft(45);
	setTurnGunLeft(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
    setTurnLeft(45);
	setTurnGunLeft(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
    setTurnLeft(45);
	ahead(100);
	
	
    setAhead((100)*movementDirection);
	previousEnergy = e.getEnergy();
  
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// если попала пуля
		back(50);
		setTurnLeft(45);
		ahead(50);
		}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
			// встреча со стенкой
        back(20);		
        setTurnLeft(90);
		ahead(100);
			
	}
	
	public void onHitRobot(HitRobotEvent e) {
	    //встреча с роботом
		setTurnLeft(70);
		back(50);
		setTurnLeft(e.getBearing()+10);
		fire(gunPower);
		setTurnLeft(90);
		ahead(200);
	
	}	
}
