# Scripts
This directory contains a series of useful scripts related to working with CAN bus.

>[!TIP]
> Copy these script to home directory on Pika Spark for easy access

- `setup_can.sh`: Brings up the CAN interface on Pika Spark
- `setup_slcan.sh`: Pavel Kirienko's script to set up SLCAN. Called by `setup_slcan.sh`. Has a few command line arguments so call `--help` to learn how to call it.
    
     **Note**: This is old version of setup_slcan.sh used with UAVCAN, but it is exactly the same

- `setup_vcan.sh`: Sets up vcan on development host. Bitrate irrelevant as it is entirely virtual.
- `setup_yakut_slcan.sh`: Calls `setup_slcan.sh` internally, and exports a few environment variables needed for use with yakut. Source this script.
- `setup_yakut_vcan.sh`: Calls `setup_vcan.sh` internally, and exports a few environment variables needed for use with yakut. Source this script.
- `build.sh`: Call from root of ROS 2 Workspace, not package root! Need to add yourself to `docker` group. Execute with `--help` flag for usage.

> [!NOTE]
> CAN is using 1 Mbps bitrate right now. Change speed code in `setup_yakut_slcan.sh` or bitrate in `setup_can.sh` if decide to use something else. The ESCs will need to be updated accordingly too (restart ESC for it to be stored in EEPROM).

Technically, `slcan` and `can` are different, with the former being an ASCII based protocol that Zubax Babel uses, and the latter being a physical interface that a laptp does not have. For sake of simplicity though, the basename has been set to `can` when setting up the interface. This way, `can` can represent 'hardware' while `vcan` can represent the 'virtual' bus.