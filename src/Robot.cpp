#include <WPILib.h>
#include <CANTalon.h>
#include <SpeedController.h>
#include <Joystick.h>
#include <GenericHID.h>
#include <DriverStation.h>
#include <navXSensor.h>
#include <DoubleSolenoid.h>
#include <Solenoid.h>


/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public frc::SampleRobot {
	frc::RobotDrive myRobot { lf, lr, rf, rr };  // robot drive system

private:
	CANTalon shooter     {4};  // CAN Talon
	frc::Talon lf        {0};// initialize the motor as a
	frc::Talon lr        {1}; // initialize the motor as a
	frc::Talon rf        {2}; // initialize the motor as a
	frc::Talon rr        {3}; // initialize the motor as a
	frc::Talon feeder    {4}; // initialize the motor as a
	frc::Talon climber   {5}; // initialize the motor as a
	frc::Talon climberB  {6}; // initialize the motor as a
	frc::Talon azimuth   {7}; // initialize the motor as a
	frc::Talon intake    {8}; // initialize the motor as a
	frc::Servo convertor {9}; // initialize the motor as a
	frc::XboxController stick { 0 };  // Xbox controller
    frc::Talon *blender;          /* E.g., PWM out to motor controller  */
	/* DoubleSolenoid corresponds to a double solenoid.
	 * Use double solenoid with forward channel of 1 and reverse of 2
	 */
	frc::DoubleSolenoid m_doubleSolenoid1 { 0, 1 };
	frc::DoubleSolenoid m_doubleSolenoid2 { 2, 3 };

	// Update every 5milliseconds/0.005 seconds.
	static constexpr double kUpdatePeriod = 0.005;

	// Numbers of the buttons to use for triggering the solenoids.
//	static constexpr int kSolenoidButton = 1;
	static constexpr int kDoubleSolenoid2Forward = 2;
	static constexpr int kDoubleSolenoid2Reverse = 3;
	static constexpr int kDoubleSolenoid1Forward = 0;
	static constexpr int kDoubleSolenoid1Reverse = 1;


    enum PinType { DigitalIO, PWM, AnalogIn, AnalogOut };
	char message[128];
    static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
	static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
	static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
	static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
	static const int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
	static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;

public:
	Robot() {
		myRobot.SetExpiration(0.1);
		blender = new Talon(        GetChannelFromPin( PinType::PWM,       0 ));
	}

	/* GetChannelFromPin( PinType, int ) - converts from a navX MXP */
	/* Pin type and number to the corresponding RoboRIO Channel     */
	/* Number, which is used by the WPI Library functions.          */

	int GetChannelFromPin( PinType type, int io_pin_number ) {
	    int roborio_channel = 0;
	    if ( io_pin_number < 0 ) {
	        throw std::runtime_error("Error:  navX MXP I/O Pin #");
	    }
	    switch ( type ) {
	    case DigitalIO:
	        if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
	            throw std::runtime_error("Error:  Invalid navX MXP Digital I/O Pin #");
	        }
	        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS +
	                          (io_pin_number > 3 ? 4 : 0);
	        break;
	    case PWM:
	        if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
	            throw std::runtime_error("Error:  Invalid navX MXP Digital I/O Pin #");
	        }
	        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
	        break;
	    case AnalogIn:
	        if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
	            throw new std::runtime_error("Error:  Invalid navX MXP Analog Input Pin #");
	        }
	        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
	        break;
	    case AnalogOut:
	        if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {
	            throw new std::runtime_error("Error:  Invalid navX MXP Analog Output Pin #");
	        }
	        roborio_channel = io_pin_number;
	        break;
	    }
	    return roborio_channel;
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() {

		double climberVal = 0.0;
		double feederVal = 0.0;
		int povVal = 0;

		while (IsOperatorControl() && IsEnabled()) {
			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			feeder.Set(stick.GetX(frc::GenericHID::kLeftHand));

			feederVal = stick.GetX(frc::GenericHID::kLeftHand);
			if ((feederVal > 0.05) || (feederVal < -0.05))
			{
			  feeder.Set(feederVal);
			} else {
				feeder.Set(0.0);
			}

			climberVal = stick.GetY(frc::GenericHID::kLeftHand);
			if ((climberVal > 0.05) || (climberVal < -0.05))
			{
			  climber.Set(climberVal);
			  climberB.Set(-climberVal);
			} else {
				climber.Set(0.0);
				climberB.Set(0.0);
			}

			if (stick.GetTriggerAxis(frc::GenericHID::kLeftHand) > 0.02) {
				blender->Set(stick.GetTriggerAxis(frc::GenericHID::kLeftHand));
			} else if (stick.GetTriggerAxis(frc::GenericHID::kRightHand) > 0.02) {
				blender->Set(-(stick.GetTriggerAxis(frc::GenericHID::kRightHand)));
			} else {
				blender->Set(0.0);
			}

			povVal = stick.GetPOV();
			switch (povVal) {
			case -1:
				m_doubleSolenoid1.Set(frc::DoubleSolenoid::kOff);
				m_doubleSolenoid2.Set(frc::DoubleSolenoid::kOff);
				break;
			case 0:
				m_doubleSolenoid1.Set(frc::DoubleSolenoid::kForward);
				break;
			case 180:
				m_doubleSolenoid1.Set(frc::DoubleSolenoid::kReverse);
				break;
			case 90:
				m_doubleSolenoid2.Set(frc::DoubleSolenoid::kForward);
				break;
			case 270:
				m_doubleSolenoid2.Set(frc::DoubleSolenoid::kReverse);
				break;
			default:
				m_doubleSolenoid1.Set(frc::DoubleSolenoid::kOff);
				m_doubleSolenoid2.Set(frc::DoubleSolenoid::kOff);
				break;
			}

			sprintf(message, "GetPOV = %d\n", povVal);
			frc::DriverStation::ReportError(message);
//			sprintf(message, "GetTriggerAxis(left) = %f\n", stick.GetTriggerAxis(frc::GenericHID::kLeftHand));
//			frc::DriverStation::ReportError(message);

			if (stick.GetAButton())
			{
				intake.Set(1.0);
			} else if (stick.GetBButton())
			{
				intake.Set(-1.0);
			} else
			{
				intake.Set(0.0);
			}

			if (stick.GetXButton())
			{
				azimuth.Set(1.0);
			} else if (stick.GetYButton())
			{
				azimuth.Set(-1.0);
			} else
			{
				azimuth.Set(0.0);
			}

			if (stick.GetStartButton())
			{
				shooter.Set(1.0);
			} else if (stick.GetBackButton())
			{
				shooter.Set(-1.0);
			} else
			{
				shooter.Set(0.0);
			}

			frc::Wait(0.005);			// wait for a motor update time
		}
	}
};

START_ROBOT_CLASS(Robot)
