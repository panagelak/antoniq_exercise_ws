import sys

from rqt_gui.main import Main


def main():
    main = Main()
    sys.exit(
        main.main(
            sys.argv,
            standalone='antoniq_mission_status_rqt.mission_status_plugin.MissionStatusPlugin'))


if __name__ == '__main__':
    main()
