from vehicle_motion_api import VehicleMotionAPI
import rospy
from std_msgs.msg import Header, String, Int32
import time
def main():
    """
    Main function
    """
    try:
        VM = VehicleMotionAPI()
        """ Set team name here """
        VM.General.vehicle_set_team_name("Unknown")
        while (not VM.General.exit_flag):
            """ Begin application code """
            # VM.General.vehicle_control_manual_override(True)
            # time.sleep(0.1)
            # VM.General.vehicle_control_manual(throttle=1, brake=0)
            # time.sleep(1)
            # VM.General.vehicle_control_manual(throttle=0, brake=1)
            # VM.General.dev_detect_door()
            print(VM.General.get_vehicle_door())
            time.sleep(1)
            # VM.General.vehicle_control_manual_override(False)
            """ End application code """
    except KeyboardInterrupt:
        print("Shutting down")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    