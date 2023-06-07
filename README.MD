# Pacchetto di supporto per compito AIPR 2306

[![N|Solid](https://www.unicampania.it/doc/img/logo_vanvitelli.jpg)](https://www.ingegneria.unicampania.it/roboticslab)

## Install

- Prerequisiti: Installare le dipendenze

Nella cartella src del ros workspace
```bash
rosdep install -y --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

## Avviare la demo

Avviare il rosmaster (non necessario):
```bash
roscore
```

In un nuovo terminale avviare il simulatore:
```bash
roslaunch aipr_support_2306 sim.launch
```

## License

GNU General Public License v3.0