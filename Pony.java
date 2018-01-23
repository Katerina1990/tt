package Pony;
import java.awt.*;
import robocode.*;
import robocode.Robot;
import robocode.ScannedRobotEvent;
import robocode.HitByBulletEvent;
import robocode.RateControlRobot;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import java.awt.Color;

public class Pony extends AdvancedRobot
{
  double previousEnergy = 100;
  int movementDirection = 1;
  int gunDirection = 1;
  int gunPower = 2;	
/**
	 * run: Pony's default behavior
	 */
	public void run() {
	 //устанавливаем все цвета
	setBodyColor(Color.WHITE);
	setGunColor(Color.RED);
	setRadarColor(Color.BLUE);
	setBulletColor(Color.WHITE);
	setScanColor(Color.RED);
    
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
    // если подошли близко
	
	if (e.getDistance()<200) {
		gunPower=3;
	} else {
		gunPower=2;
	}

    
    gunDirection = -gunDirection; //направление удара
	setTurnGunRight(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
	setTurnGunLeft(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
	setTurnGunRight(Double.POSITIVE_INFINITY*gunDirection);
	fire ( gunPower ) ;
    setTurnRight(45);
	ahead(40);
	
    setAhead((100)*movementDirection);
	previousEnergy = e.getEnergy();
  
}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// если попала пуля
		setTurnRight(85);
		ahead(175);
		setTurnLeft(45);
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// встреча со стенкой
		setTurnRight(180);
		ahead(300);
		setTurnLeft(90);
	}	
	
	public void onHitRobot(HitRobotEvent e) {
	    //встреча с роботом
		movementDirection=-movementDirection;
		ahead(100);
        setTurnRight(145);
		ahead(500);
		//setTurnLeft(130);
	}
	
	public void onBulletHit(BulletHitEvent e) {
	    // поражен противник
		fire(gunPower);
	}
}
