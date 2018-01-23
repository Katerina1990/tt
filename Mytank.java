package TT;
import robocode.*; 
import robocode.AdvancedRobot;
import robocode.DeathEvent;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.Random;
import java.util.Map;
import java.util.*;

import static java.lang.Math.signum;
import static java.lang.Math.toRadians;

public class Mytank  extends AdvancedRobot {

   private static final double RADIANS_5 = toRadians(5);

   private boolean isAlive = true;

   private double enemyX = -1;  //позиция врага по x
   private double energe = -1;  //энергия 
   private double enemyY = -1;  //позиция врага по y
   private double velocity = -1;  //скорость
   private double distance = -1;  //расстояние
		
    final double FIREPOWER = 5; // максимальная сила удара, на которуюспособен робот
	final double HALF_ROBOT_SIZE = 18; // Robot size is 36x36 units, so the half size is 18 units
	Map<String, RobotData> enemyMap;  // карта, которая хранит значение строка - робот
	double scanDir = 1;   //директория сканирования 
	
    // последний просканированный робот
	RobotData oldestScanned; 

	// робот, на которого направлена атака. Если null - значит сейчас такого нет
	RobotData target;

	// последнее время изменения роботом направления движения
	long lastDirectionShift;

	// текущее направление движения, 1 - вперед, -1 - назад
	int direction = 1;

	public void BorderGuard() {
		// инициализируется специальная карта HashMap которая используется для последовательного доступа к элементам.
		// работает по типу стека 
        // робот всегда направляет радар в сторону последнего сканированного робота.
		enemyMap = new HashMap<String, RobotData>();
	}




/**
 * Mytank 
 */

	/**
	 * run: описывает поведение танка по умолчанию
	 */
	public void run() { 
    setTurnRadarRightRadians(Double.POSITIVE_INFINITY); // пока противник не найден бесконечно крутим радар в право
   
       while (isAlive) { // в принципе это не обязательно и можно оставить true

           if (enemyX > -1) { // если противник обнаружен
			
               final double radarTurn = getRadarTurn();
               setTurnRadarRightRadians(radarTurn);
			 setTurnRadarRightRadians(radarTurn);

               final double bodyTurn = getBodyTurn();
              setTurnRightRadians(bodyTurn);

               if (getDistanceRemaining() == 0) {
                    final double distance = getDistance();
                   setAhead(distance);
                }
                    final double gunTurn = getGunTurn();
              setTurnGunRightRadians(gunTurn);
              setFire(2);

            }

     
           
          execute();
        }
		
	//вызываем метод инициализации
		initialize();  
		// постоянный цикл действий робота
	while (true) {
		
			// вызываем сканирование вражеских роботов
			handleRadar();
			// вызывет метод для управление оружием и стрельбы
			handleGun();
			// вызываем метод для перемещения робота по полю
			moveRobot();

			// Сканирование других роботов
			scan();
		}
	 
	}
	
/*@Override
	public void onScannedRobot(ScannedRobotEvent scannedRobotEvent) {
		// Check that the scanned robot is not a sentry robot
		if (!scannedRobotEvent.isSentryRobot()) {
			// The scanned robot is not a sentry robot...

			// Update the enemy map
			updateEnemyMap(scannedRobotEvent);

			// Update the scan direction
			updateScanDirection(scannedRobotEvent);

			// Update enemy target positions
			updateEnemyTargetPositions();
		}
	}
*/	
	@Override
	//обработка когда робот умер
	public void onRobotDeath(RobotDeathEvent robotDeathEvent) {
		// получем имя убитого робота
		final String deadRobotName = robotDeathEvent.getName();
		// удаляем данные убитого робота из списка карты
		enemyMap.remove(deadRobotName);
		// удаляем данные о последнем сканированном роботе, если это был убитый робот
		if (oldestScanned != null && oldestScanned.name.equals(deadRobotName)) {
			oldestScanned = null;
		}
		if (target != null && target.name.equals(deadRobotName)) {
			target = null;
		}
	}
	
@Override
	public void onPaint(Graphics2D g) {
	
		g.setStroke(new BasicStroke(2f));
		Color color1 = new Color(0x00, 0xFF, 0x00, 0x40); // Green with 25% alpha blending
		Color color2 = new Color(0xFF, 0xFF, 0x00, 0x40); // Yellow with 25% alhpa blending

		// Paint a two circles for each robot in the enemy map. One circle where the robot was
		// scanned the last time, and another circle where our robot must point the gun in order to
		// hit it (target coordinate). In addition, a line is drawn between these circles.
		for (RobotData robot : enemyMap.values()) {
			// Paint the two circles and a line
			fillCircle(g, robot.scannedX, robot.scannedY, color1); // scanned coordinate
			fillCircle(g, robot.targetX, robot.targetY, color2); // target coordinate
			g.setColor(color1);
			g.drawLine((int) robot.scannedX, (int) robot.scannedY, (int) robot.targetX, (int) robot.targetY);
		}

		// Paint a two circles for the target robot. One circle where the robot was
		// scanned the last time, and another circle where our robot must point the gun in order to
		// hit it (target coordinate). In addition, a line is drawn between these circles.
		if (target != null) {
			// Prepare colors for painting the scanned coordinate and target coordinate
			color1 = new Color(0xFF, 0x7F, 0x00, 0x40); // Orange with 25% alpha blending
			color2 = new Color(0xFF, 0x00, 0x00, 0x80); // Red with 50% alpha blending

			// Paint the two circles and a line
			fillCircle(g, target.scannedX, target.scannedY, color1); // scanned coordinate
			fillCircle(g, target.targetX, target.targetY, color2); // target coordinate
			g.setColor(color1);
			g.drawLine((int) target.scannedX, (int) target.scannedY, (int) target.targetX, (int) target.targetY);
		}
	}

	/**
	 * инициализация робота перед новым раундом сражения.
	 */
	private void initialize() {
		// устанавливем что робот, пушка и радар вращаются независимо друг от друга
		setAdjustRadarForGunTurn(true);
		setAdjustGunForRobotTurn(true);

		// Sустанвливаем цвет робота
		setBodyColor(Color.GREEN); // Chocolate Brown
		setGunColor(Color.WHITE); // Aqua Marine
		setRadarColor(new Color(0xD2, 0x69, 0x1E)); // Orange Chocolate
		setBulletColor(new Color(0xFF, 0xD3, 0x9B)); // Burly wood
		setScanColor(new Color(0xCA, 0xFF, 0x70)); // Olive Green
	}
	//вращение радара
	private void handleRadar() {
		// радар устанавливается до предела вправо, если направление сканирования положительное, и влево - если отрицательное
		// onScannedRobot(ScannedRobotEvent) отвечает за определение направления сканирования
		setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
	}
    //вращение пушки		
    private void handleGun() {
		// перезарядка оружия
		updateTarget();
		// обновление направления
		updateGunDirection();
		// выстрел
		fireGunWhenReady();
	}

	/**
	 * перемещение робота по полю
	 */
	private void moveRobot() {
     	// робот стремится двигаться по границе по горизонтали или вертикали
		// когда робот приблизился к цели он должен все время менять направление движения и не стоять на месте
		int newDirection = direction;
		// стараемся подойти как можно ближе к цели
		if (target != null) {
			// вычисляем расстояние до границ, так чтоб наш робот находился внутри
			int borderRange = getSentryBorderSize() - 20;
			// флаги для перемещения по горизонтали и вертикали
			boolean horizontal = false;
			boolean vertical = false;
			double newHeading = getHeadingRadians();

			//проверка, находится ли робот на верхней или нижней границе, если да  - то движемся по горизонтали
			if (getY() < borderRange || getY() > getBattleFieldHeight() - borderRange) {
				horizontal = true;
			}
			// если на правой или левой границе - движемся по вертикале
			if (getX() < borderRange || getX() > getBattleFieldWidth() - borderRange) {
				vertical = true;
			}

			// если в углу  - то нужно выбрать направление движения
			if (horizontal && vertical) {
				// если по горизонтали до цели меньше чем по вертикали, выбираем вертикальное движение
				if (Math.abs(target.targetX - getX()) <= Math.abs(target.targetY - getY())) {
					horizontal = false; // Do not move horizontally => move vertically
				}
			}
			if (horizontal) {
				newHeading -= Math.PI / 2;
			}
			// Робот поворачивает влево на вычисленное количество радиан
			setTurnLeftRadians(Utils.normalRelativeAngle(newHeading));

			// если осталось меньше 1 градуса до конца поворота
			if (Math.abs(getTurnRemaining()) < 1 || Math.abs(getVelocity()) < 0.01) {
				double delta; // расстояние для перемещения
				if (horizontal) {
					delta = target.targetX - getX();
				} else {
					delta = target.targetY - getY();
				}
				setAhead(delta);

				// установить новое направление движения
				newDirection = delta > 0 ? 1 : -1;

				// если прошло больше 10 поворотов с нашего последнего изменения направления
				if (getTime() - lastDirectionShift > 10) {
					// тогда изменяем направление на противоположное, если скорость меньше 1
					if (Math.abs(getVelocity()) < 1) {
						newDirection = direction * -1;
					}
					// если сменили направление
					if (newDirection != direction) {
						direction = newDirection;
						lastDirectionShift = getTime();
					}
				}
			}
		}
		// устанвливаем 100 единиц по направлению движения
		setAhead(100 * direction);
	}

	/**
	 * обновление карты противников на основе сканирования роботов
	 */
	private void updateEnemyMap(ScannedRobotEvent scannedRobotEvent) {
		// получаем имя робота
		final String scannedRobotName = scannedRobotEvent.getName();
		// получаем данные о роботе, если есть запись на карте 
		RobotData scannedRobot = enemyMap.get(scannedRobotName);
		// если нет информации о роботе на карте
		if (scannedRobot == null) {
			// создаем новые данные про отсканированного робота
			scannedRobot = new RobotData(scannedRobotEvent);
			// добавляем данные о роботе на карту
			enemyMap.put(scannedRobotName, scannedRobot);
		} else {
			// если данные уже есть - обновляем данные новыми сканированными
			scannedRobot.update(scannedRobotEvent);
		}
	}

	/**
	 * обновление напраления радара на основе сканирования роботов.
	 * 
	 */
	private void updateScanDirection(ScannedRobotEvent scannedRobotEvent) {
		// получаем имя робота
		final String scannedRobotName = scannedRobotEvent.getName();
		// Измением направление сканирования тогда и только тогда, когда у нас нет записи для самого старого отсканированного
        // робота или сканированный робот - самый старый сканированный робот. 
		//и карта противников сканированын данные для всех роботов
		if ((oldestScanned == null || scannedRobotName.equals(oldestScanned.name)) && enemyMap.size() == getOthers()) {
			// получаем самую старую запись из списка
			RobotData oldestScannedRobot = enemyMap.values().iterator().next();
			// получаем текущую позицию для самого сторого сканированного робота
			double x = oldestScannedRobot.scannedX;
			double y = oldestScannedRobot.scannedY;
			// получаем заголовок робота
			double ourHeading = getRadarHeadingRadians();
			// вычислем азимут (направление) на самого старого сканированного робота
			// это есть разница между направление на нашего робота и других роботов
     		// угол может быть положит и отрицат.
			double bearing = bearingTo(ourHeading, x, y);
			// обновляем направление сканирования.
			// если азимут положит радар направляется вправо, иначе влево.
			scanDir = bearing;
		}
	}

	/**
	 * обновляет целевые позиции для атаки
	 * 
	 * алгоиртм тутhttp://robowiki.net/wiki/Linear_Targeting
	 */
	private void updateEnemyTargetPositions() {
		// проходим через всех роботов на карте
		for (RobotData enemy : enemyMap.values()) {

			double bV = Rules.getBulletSpeed(FIREPOWER);
			double eX = enemy.scannedX;
			double eY = enemy.scannedY;
			double eV = enemy.scannedVelocity;
			double eH = enemy.scannedHeading;

			//константы для вычисления квадратичных коэффициенов
			double A = (eX - getX()) / bV;
			double B = (eY - getY()) / bV;
			double C = eV / bV * Math.sin(eH);
			double D = eV / bV * Math.cos(eH);

			// квадратичные коэффициенты a*(1/t)^2 + b*(1/t) + c = 0
			double a = A * A + B * B;
			double b = 2 * (A * C + B * D);
			double c = (C * C + D * D - 1);

			// вычисляем дискриминант.
			double discrim = b * b - 4 * a * c;
			if (discrim >= 0) {
				// 2 решения по времени
				double t1 = 2 * a / (-b - Math.sqrt(discrim));
				double t2 = 2 * a / (-b + Math.sqrt(discrim));

				// Выбераем минимальное положительное время или ближайшее к 0, если время отрицательное
				double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);

				// вычисляем целевую позицию для атаки, куда будет указывать наша пушка в момент времни t.
				double targetX = eX + eV * t * Math.sin(eH);
				double targetY = eY + eV * t * Math.cos(eH);

				// ограничиваем цели на стенах
				double minX = HALF_ROBOT_SIZE;
				double minY = HALF_ROBOT_SIZE;
				double maxX = getBattleFieldWidth() - HALF_ROBOT_SIZE;
				double maxY = getBattleFieldHeight() - HALF_ROBOT_SIZE;

				enemy.targetX = limit(targetX, minX, maxX);
				enemy.targetY = limit(targetY, minY, maxY);
			}
		}
	}

	/**
	 * обновление целей
	 */
	private void updateTarget() {
		
		target = null;
		// создает список возможных целей
		ArrayList<RobotData> targets = new ArrayList<RobotData>(enemyMap.values());

		// те кто находятся за пределами атаки удаляются
		Iterator<RobotData> it = targets.iterator();
		while (it.hasNext()) {
			RobotData robot = it.next();
			if (isOutsideAttackRange(robot.targetX, robot.targetY)) {
				it.remove();
			}
		}

		// выбираем до какого робота меньше расстояние, он становится целью
		double minDist = Double.POSITIVE_INFINITY;
		for (RobotData robot : targets) {
			double dist = distanceTo(robot.targetX, robot.targetY);
			if (dist < minDist) {
				minDist = dist;
				target = robot;
			}
		}

		// если еще нет целевого робота, то берем первый из списка
		if (target == null && targets.size() > 0) {
			target = targets.get(0);
		}
	}

	/**
	 * Метод, который обновляет направление пистолета, чтобы указать на текущую цель.
	 */
	private void updateGunDirection() {
		// обновляет направление, только есть есть цель
		if (target != null) {
			// вычисляем азимут между пушкой и целью
			double targetBearing = bearingTo(getGunHeadingRadians(), target.targetX, target.targetY);
			// осуществляем поворот
			setTurnGunRightRadians(targetBearing); 
		}
	}

	/**
	 * метод, стреляет когд есть оружие
	 */
	private void fireGunWhenReady() {
		// стреляем когда есть цель
		if (target != null) {
			// стреляем когда пушка указывает на цель

			// вычисляем расстояние до цели
			double dist = distanceTo(target.targetX, target.targetY);
			// Угол, который «покрывает» целевого робота от его центра до его края
			double angle = Math.atan(HALF_ROBOT_SIZE / dist);
			// если оставшийся угол (поворот) для перемещения пистолета меньше, чем расчетная угол
			if (Math.abs(getGunTurnRemaining()) < angle) {
				// тогда открываем огонь
				setFire(FIREPOWER);
			}
		}
	}

	/**
	 * проверка находится ли точка вне диапазона атаки
	 */
	private boolean isOutsideAttackRange(double x, double y) {
		double minBorderX = getSentryBorderSize();
		double minBorderY = getSentryBorderSize();
		double maxBorderX = getBattleFieldWidth() - getSentryBorderSize();
		double maxBorderY = getBattleFieldHeight() - getSentryBorderSize();
		return (x > minBorderX) && (y > minBorderY) && (x < maxBorderX) && (y < maxBorderY);
	}


	private double limit(double value, double min, double max) {
		return Math.min(max, Math.max(min, value));
	}

	private double distanceTo(double x, double y) {
		return Math.hypot(x - getX(), y - getY());
	}


	private double angleTo(double x, double y) {
		return Math.atan2(x - getX(), y - getY());
	}

	/**
	 * вычисление азимута
	 */
	private double bearingTo(double heading, double x, double y) {
		return Utils.normalRelativeAngle(angleTo(x, y) - heading);
	}

	/**
	 * Mетод, который рисует заполненный круг в заданной координате (x, y) и заданном цвете.
	 * 
	 * 
	 */
	private void fillCircle(Graphics2D gfx, double x, double y, Color color) {
	
		gfx.setColor(color);
		gfx.fillOval((int) x - 20, (int) y - 20, 40, 40);
	}

//класс - данные робота
	class RobotData {
		final String name; // имя
		double scannedX; // x координата сканированного робота на основе последнего обновления
		double scannedY; // y координата сканированного робота на основе последнего обновления
		double scannedVelocity; // скорость сканированного робота из последнего обновления
		double scannedHeading; // направление сканированного робота из последнего обновления
		double targetX; // предикат x координаты для прицеливания  пистолета при стрельбе по роботу
		double targetY; // предикат у координаты для прицеливания  пистолета при стрельбе по роботу

		/**
		 * создает новую запись данных робота на основе новых данных сканирования для сканированного робота.
		  */
		RobotData(ScannedRobotEvent event) {
			name = event.getName();
			update(event);
			targetX = scannedX;
			targetY = scannedY;
		}

		/**
		 * обновляет отсканированные данные на основе новых данных сканирования сканируемого робота.
		*/
		void update(ScannedRobotEvent event) {
			// получаем новую позицию
			Point2D.Double pos = getPosition(event);
			// сохраняем ее
			scannedX = pos.x;
			scannedY = pos.y;
			// сохраняем скорость и направление
			scannedVelocity = event.getVelocity();
			scannedHeading = event.getHeadingRadians();
		}

			Point2D.Double getPosition(ScannedRobotEvent event) {
				double distance = event.getDistance();
				double angle = getHeadingRadians() + event.getBearingRadians();

			double x = getX() + Math.sin(angle) * distance;
			double y = getY() + Math.cos(angle) * distance;
			return new Point2D.Double(x, y);
		}
	}









       private double getRadarTurn() {
        // роботу жизненно необходимо постоянно видеть противника
        // считаем абсолютный угол до противника:
        final double alphaToEnemy = angleTo(getX(), getY(), enemyX, enemyY);
        // считаем направление, на который надо повернуть радар, чтобы противник остался в фокусе (Utils, это встренный в Robocode класс):
        final double sign = (alphaToEnemy != getRadarHeadingRadians())
                ? signum(Utils.normalRelativeAngle(alphaToEnemy - getRadarHeadingRadians()))
                : 1;

        // добавляем 5 градусов поворта для надёжности и получаем результирующий угол
        return Utils.normalRelativeAngle(alphaToEnemy - getRadarHeadingRadians() + RADIANS_5 * sign);
        // В принципе, прямо здесь можно вызвать setTurnRadarRightRadians, но я противник функций с сайд эффектами и стараюсь
        // минимизировать их количество
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        /** ScannedRobotEvent не содержит в себе явно положения противника, однако, его легко вычислить, зная направление
         * своего корпуса, беаринг (по сути угол относительный чего-то, в данном случае относительно корпуса) и расстояние до противника
         */

        // абсолютный угол до противника
       final double alphaToEnemy = getHeadingRadians() + event.getBearingRadians();

        // а далее элементарная геометрия
        enemyX = getX() + Math.sin(alphaToEnemy) * event.getDistance();
        enemyY = getY() + Math.cos(alphaToEnemy) * event.getDistance();
	    energe = event.getEnergy();
		velocity = event.getVelocity();	
        distance = event.getDistance();
		
    }

    @Override
    /*+*/public void onDeath(DeathEvent event) {
        isAlive = false;
    }
 /*
    @Override
   +public void onPaint(Graphics2D g) {
        // убеждаемся, что вычислили позицию противника верно
        // для того чтобы увидеть что мы рисуем, необходимо во время битвы на правой понели кликнуть по имени робота
        // и в появившемся окне нажать кнопку Paint

        if (enemyX > -1) {
            g.setColor(Color.WHITE);
            g.drawRect((int) (enemyX - getWidth() / 2), (int) (enemyY - getHeight() / 2), (int) getWidth(), (int) getHeight());
        }
    }
*/
    /**
     * В Robocode немного извращённые углы - 0 смотрит на север и далее по часовой стрелке:
     * 90 - восток, 180 - юг, 270 - запад, 360 - север.
     * <p/>
     * Из-за этого приходится писать собственный метод вычисления угла между двумя точками.
     
     */
    /*+*/
     private static double angleTo(double baseX, double baseY, double x, double y) {
        double theta = Math.asin((y - baseY) / Point2D.distance(x, y, baseX, baseY)) - Math.PI / 2;
        if (x >= baseX && theta < 0) {
            theta = -theta;
        }
        return (theta %= Math.PI * 2) >= 0 ? theta : (theta + Math.PI * 2);
    }
private double getDistance() {
	

        // вычесление дистанции движения элементарно
	Random	r = new Random();
		
        return 200 - 400 * r.nextDouble();
    }

    private double getBodyTurn() {
        // а вот вычисление угла поворота посложее
        final double alphaToMe = angleTo(enemyX, enemyY, getX(), getY());

        // определяем угловое направление относительно противника (по часовой стрелке, либо против) ...
        final double lateralDirection = signum((getVelocity() != 0 ? getVelocity() : 1) * Math.sin(Utils.normalRelativeAngle(getHeadingRadians() - alphaToMe)));
        // получаем желаемое направление движения
        final double desiredHeading = Utils.normalAbsoluteAngle(alphaToMe + Math.PI / 2 * lateralDirection);
        // нормализуем направление по скорости
        final double normalHeading = getVelocity() >= 0 ? getHeadingRadians() : Utils.normalAbsoluteAngle(getHeadingRadians() + Math.PI);
        // и возвращаем угол поворта
        return Utils.normalRelativeAngle(desiredHeading - normalHeading);
    }
private double getGunTurn() {
       // вычисления тривиальны: считаем на какой угол надо повернуть пушку, чтобы она смотрела прямо на противника:
       return Utils.normalRelativeAngle(angleTo(getX(), getY(), enemyX, enemyY) - getGunHeadingRadians());
   }



}
