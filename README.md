
# ilm_ros_pkg

Dies ist das ROS-Package welches im Rahmen der Arbeit "Fusion von Kamera- und Punktwolkendaten in Simulation und Realwelt" erstellt wurde.

Um das Projekt zu bauen, muss das Package in einem catkin_workspace platziert und mit dem Befehl `catkin_make` gebaut werden.

## Inhalt
Unter dem Ordner `src` ist der Source Code zu finden. Hierbei gibt es vier Executables.
 - `camInfoPublisher`: Node, der eine .yaml-Datei mit intrinsischen Parametern ausliest und kontinuierlich unter dem anegebenem Topic veröffentlicht
 - `pairPicker`: Ein Tool mit dem 3D-2D Korrespondenzen manuell abgelesen werden können
 - `extrinsiscCalibrator`: Ein Tool, welches aus den Korrespondenzen, die mit `pairPicker` abgelesen wurden, extrinsische Parameter berechnet
 - `protofuse`: ROS-Node, der die Fusion einer 3D-Punktwolke mit zwei Kamerabildern und einer Punktwolke durchführt

