
package ros.app;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class FRIServerPosition extends RoboticsAPIApplication {
  // members
  @Inject
  private LBR lbr_;

  @Inject
  @Named("DefaultTool")
  private Tool tool;

  // FRI session
  private FRISession fri_session_;
  private FRIJointOverlay fri_overlay_;
  private PositionControlMode control_mode_;

  public void configure_fri() {
    // remote IP address
    String client_name = "172.31.1.150";
    getLogger().info("Remote address set to: " + client_name);

    // FRI session boilerplate
    FRIConfiguration fri_configuration = FRIConfiguration.createRemoteConfiguration(lbr_, client_name);
    fri_configuration.setSendPeriodMilliSec(10);

    getLogger().info("Creating FRI connection to " + fri_configuration.getHostName());
    getLogger().info(
        "SendPeriod: " + fri_configuration.getSendPeriodMilliSec() + "ms |" + " ReceiveMultiplier: "
            + fri_configuration.getReceiveMultiplier());

    fri_session_ = new FRISession(fri_configuration);
    fri_overlay_ = new FRIJointOverlay(fri_session_, ClientCommandMode.POSITION);

    fri_session_.addFRISessionListener(new IFRISessionListener() {
      @Override
      public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
        getLogger().info("Session State change " + friChannelInformation.getFRISessionState().toString());
      }

      @Override
      public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
        getLogger().info("Quality change signalled " + friChannelInformation.getQuality());
        getLogger().info("Jitter " + friChannelInformation.getJitter());
        getLogger().info("Latency " + friChannelInformation.getLatency());
      }
    });
    // try to connect
    try {
      fri_session_.await(1000, TimeUnit.SECONDS);
    }
    catch (final TimeoutException e) {
      getLogger().error(e.getLocalizedMessage());
      return;
    }
    getLogger().info("FRI connection established.");
  }

  @Override
  public void initialize() {
    // Gravity compensation
    tool.attachTo(lbr_.getFlange());
    // Move to mechanical zero position to avoid jerky motions
    getLogger().info("Show modal dialog and wait for user to confirm");
    int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION,
        "The robot will move to the mechanical zero position.", "OK", "Cancel");
    if (isCancel == 1) { return; }
    getLogger().info("Move to the mechanical zero position");
    PTP ptpToMechanicalZeroPosition = ptp(0, 0, 0, 0, 0, 0, 0);
    ptpToMechanicalZeroPosition.setJointVelocityRel(0.25);
    lbr_.move(ptpToMechanicalZeroPosition);

    // Initialize position control for motion overlay
    control_mode_ = new PositionControlMode();
    // Configure FRI session, which allows the client to read the robots state data
    configure_fri();
    // Reduce Acceleration
    fri_overlay_.overrideJointAcceleration(1.0);
    getLogger().info("Acceleration Override " + fri_overlay_.getAccelerationOverride());
  }

  @Override
  public void run() {
    lbr_.move(positionHold(control_mode_, -1, TimeUnit.SECONDS).addMotionOverlay(fri_overlay_));
  }

  @Override
  public void dispose() {
    // close connection
    try {
      getLogger().info("Disposing FRI session.");
      fri_session_.close();
    }
    catch (Exception e) {
      getLogger().info("Failed to dispose FRI session. Session maybe not opened?");
      e.printStackTrace();
    }
    finally {
      super.dispose();
    }
  }

  /**
   * main
   * 
   * @param args
   */
  public static void main(final String[] args) {
    FRIServerPosition app = new FRIServerPosition();
    app.runApplication();
  }
}
