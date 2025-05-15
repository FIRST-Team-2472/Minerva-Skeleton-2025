package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase {

	Compressor compressor = new Compressor(PneumaticsConstants.kCompressorid, PneumaticsModuleType.REVPH);

	public boolean bigLeftExtended;
	public DoubleSolenoid doubleSolenoidBigLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PneumaticsConstants.kBigLeftPneumaticInflateChannel,
			PneumaticsConstants.kBigLeftPneumaticDeflateChannel);

	public boolean smallLeftExtended;
	public DoubleSolenoid doubleSolenoidSmallLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PneumaticsConstants.kSmallLeftPneumaticInflateChannel,
			PneumaticsConstants.kSmallLeftPneumaticDeflateChannel);

	public boolean bigRightExtended;
	public DoubleSolenoid doubleSolenoidBigRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PneumaticsConstants.kBigRightPneumaticInflateChannel,
			PneumaticsConstants.kBigRightPneumaticDeflateChannel);

	public boolean smallRightExtended;
	public DoubleSolenoid doubleSolenoidSmallRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			PneumaticsConstants.kSmallRightPneumaticInflateChannel,
			PneumaticsConstants.kSmallRightPneumaticDeflateChannel);


	public PneumaticsSubsystem() {
		doubleSolenoidBigLeft.set(DoubleSolenoid.Value.kReverse);
		doubleSolenoidBigRight.set(DoubleSolenoid.Value.kReverse);
		doubleSolenoidSmallLeft.set(DoubleSolenoid.Value.kReverse);
		doubleSolenoidSmallRight.set(DoubleSolenoid.Value.kReverse);

		bigLeftExtended = false;
		smallLeftExtended = false;
		bigRightExtended = false;
		smallLeftExtended = false;
	}

	/**
	 * Toggles whither the smaller cylinders are extended or not.
	 */
	public void toggleSmallpneumatics() {
		if (!bigLeftExtended) {
			toggleSmallLeftPneumatic();
		}
		if (!bigRightExtended) {
			toggleSmallRightPneumatic();
		}
	}

	public void toggleBigpneumatics() {
		if (smallLeftExtended) {
			toggleBigLeftPneumatic();
		}
		if (smallRightExtended) {
			toggleBigRightPneumatic();
		}
	}

	public void toggleBigLeftPneumatic() {
		doubleSolenoidBigLeft.toggle();
		bigLeftExtended = bigLeftExtended ? false : true;
	}

	public void toggleSmallLeftPneumatic() {
		doubleSolenoidSmallLeft.toggle();
		smallLeftExtended = smallLeftExtended ? false : true;
	}

	public void toggleBigRightPneumatic() {
		doubleSolenoidBigRight.toggle();
		bigRightExtended = bigRightExtended ? false : true;
	}

	public void toggleSmallRightPneumatic() {
		doubleSolenoidSmallRight.toggle();
		smallRightExtended = smallRightExtended ? false : true;
	}

	public boolean getBigLeftPneumatic() {
		return bigLeftExtended;
	}

	public boolean getSmallLeftPneumatic() {
		return smallLeftExtended;
	}

	public boolean getBigRightPneumatic() {
		return bigRightExtended;
	}

	public boolean getSmallRightPneumatic() {
		return smallRightExtended;
	}
}
