# Autorenschaft
Diese Anleitung und Einführung ist geschrieben von:
Laura Bielenberg, Romol Chadda, Marcus Ebner und Markus Hessinger

# Einleitung
Im Rahmen dieses Versuchs soll das Knobelspiel [Türme von Hanoi](https://de.wikipedia.org/wiki/T%C3%BCrme_von_Hanoi "Wikipedia: Türme von Hanoi") mit einem Roboterarm mit sieben Freiheitsgraden gelöst werden. Dazu muss nicht nur ein Algorithmus zur Lösung des theoretischen Problems implementiert werden, sondern auch die Schwierigkeiten gelöst werden, die bei der Bedienung eines Roboters bei einer solchen Aufgabenstellung entstehen.

Ziel des Rätsels ist, einen Stapel Scheiben von einem Turm zu einem anderen zu verschieben und bei jedem Zug stets nur eine Scheibe zu verschieben. Weiterhin dürfen Scheiben nur auf einen leeren Stab oder auf einem mit einer größeren Scheibe abgelegt werden.

# KUKA LBR iiwa
Der eingesetzte Roboterarm ist ein Leichtbauroboter _intelligent industrial work assistant_ (LBR iiwa) der Firma KUKA genutzt. Dieser ist ein serieller siebenachsiger Knickarmroboter. Jede der Achsen verfügt über einen Drehfreiheitsgrad. Aufgrund seiner sieben Freiheitsgrade (engl. Degrees of freedom, DOF) steht dem LBR iiwa ein redundanter DOF bei der Bewegungsausführung zur Verfügung.

Das gesamte Robotersystem besteht aus mehreren Komponenten. Zu diesen zählen der Manipulator selbst, welcher die Robotermechanik und Elektronik beinhaltet, der zugehörige KUKA Sunrise Cabinet Controller, auf welchem sich die robotereigene Regelung befindet sowie die Software KUKA Sunrise.OS. Zusätzlich ermöglicht das Control Panel (smartPad) die Ansteuerung und Überwachung des Roboters. Die hierzu benötigten Statusdaten des Manipulators, wie die momentanen Achspositionen und die wirkenden Momente in den einzelnen Gelenken, werden mithilfe von Drehmoment- und Achsbereichssensoren ermittelt. Diese sind den Servos der einzelnen Gelenke nachgeschaltet. Seine Leichtbauweise in Kombination mit der Drehmomentsensorik, welche die Möglichkeit der Einstellung gewünschter Momente mit einer Toleranz von +-2 % bietet, bringt eine hohe Genauigkeit bei der Kraftregelung des Roboters auch bei sensiblen Aufgaben mit sich. Aufgrund dieser Eigenschaft ist der Roboter besonders gut für Aufgaben im Bereich der Mensch-Roboter Kollaboration geeignet.

# Umgebung
Als Roboter wird hier der _KUKA iiwa_ verwendet, der einen _Robotiq_ Dreifingergreifer montiert hat. Das Gesamtsystem ist in einer [ROS](http://www.ros.org/ "ROS-website")-Umgebung eingebunden. Dabei handelt es sich um ein Software-Framework, in dem einzelne Prozesse, sogenannte _Nodes_ bestimmte Aufgaben wie Hardwareabstraktion oder reine Datenverarbeitung übernehmen können und sich über Nachrichten austauschen können. Die einzelnen Prozesse/Aufgaben sind in Paketen organisiert. Hierbei werden [Robotiq](https://github.com/ros-industrial/robotiq)-Pakete zur Ansteuerung des Greifers über EtherCAT und Pakete des [iiwa_Stacks](https://github.com/IFL-CAMP/iiwa_stack) zur Ansteuerung des Roboters verwendet. Zur Bahnplanung des Roboters wird das [MoveIt! Motion Planning Framework](https://moveit.ros.org/) verwendet. Um die Einarbeitungszeit zu verkürzen und verschiedene Ansteuerungsarten des Roboters vereinheitlicht zu verwenden, wurden die benötigten Funktionen von uns noch weiter abstrahiert und stehen in der Klasse _RobotInterface_ im Paket [iimoveit](https://github.com/KUKAnauten/iimoveit) zur Verfügung. Eine Verknüpfung zur Dokumentation der API befindet sich im Paketordner von __hanoi\_students__ (siehe übernächster Absatz) oder ist [hier zu erreichen](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API").

In ROS arbeitet man in sogenannten _Workspaces_. Für diesen Versuch wurde bereits ein Workspace eingerichtet, in dem die oben beschriebenen Pakete bereits enthalten sind. Auch wurde ein Paket erstellt, in dem sich der _Node_ befindet, der den Roboter und den Greifer steuern soll. Der Code dieses Nodes muss von Euch vervollständigt werden, sodass der Roboter die Türme von Hanoi erfolgreich löst.

Euer Workspace heißt __hanoi\_ws__ und befindet sich unter __/home/mlab8/ROS/__. Ihr werdet nur die Datei __hanoi\_iiwa\_node.cpp__ bearbeiten, die sich im Ordner __src__ des Pakets __hanoi\_students__ befindet. Diese kann mit einem Texteditor eurer Wahl geöffnet werden. Vorinstalliert ist Sublime Text, der auch Syntax Highlighting beherrscht.

# Schreiben des Programms
## Programmstruktur
Zur Durchführung der Aufgabe wurde die Klasse `HanoiRobot` erstellt, die alle Methoden von [RobotInterface](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API") erbt.

___Achtung!___ Methoden, die mit _publish_ oder _run_ beginnen, sollen im Rahmen dieses Versuchs jedoch ignoriert werden, denn hierbei wird der Bahnplanungsschritt umgangen und kann zu unkontrollierbarem Verhalten führen! MoveIt! kann zwar Trajektorien berechnen, die Kollisionen vermeiden, jedoch müssen dazu Objekte über Sensoren erkannt werden oder per Hand in die Planungsumgebung eingefügt werden. Dies ist in unserer Umgebung noch nicht der Fall, weshalb geplante Bewegungen vor der Ausführung bestätigt werden müssen und auch zuerst im Simulator Gazebo geprüft werden sollen.
Zum Bewegen sollten momentan am besten nur die Methoden `planAndMove`, `planAndMoveToBasePose` sowie `moveAlongCartesianPathInWorldCoords` verwendet werden. Diese sind zur durchführung des Versuchs ausreichend und andere Methoden befinden sich noch in experimentellem Status. Im diesem Versuch müssen folgende Methoden implementiert werden:
- `void moveSlice(int from, int to)`
- `void moveTower(int height, int from, int to, int with)`

Die Methode `moveTower` soll den Algorithmus enthalten und dabei die Methode `moveSlice` verwenden, in der der Roboter eine Scheibe von einem zum anderen Stab bewegen soll. Zu beachten ist, dass beim Bewegen des Roboters mit `planAndMove` nur die Anfangs- und Endpose vorgegeben sind. dazwischen bewegt sich der Roboter nicht auf einer geraden Linie, sondern so, wie es von den Gelenken am besten passt - dabei gibt es natürlich unendlich viele Lösungen, jenachdem, mit welchem Gütemaßen man die Planung durchführt. Weiterhin hält der Roboter bei Erreichen der Zielpose an, anstatt bestimmte Wegpunkte einfach zu durchfahren.

## Verwendung von `planAndMove`und `moveAlongCartesianPathInWorldCoords`
Diese Methoden sollen zur Bewegungsplanung- und Durchführung verwendet werden. `planAndMove` erwartet eine `geometry_msgs::Pose` als Parameter und `moveAlongCartesianPathInWorldCoords` einen `std::vector<geometry_msgs::Pose>`, was etwa einer `List` in Java entspricht. Das ist dann nützlich, wenn man den Roboter entlang einer Bahn bewegen will, oder wenn der Roboter Wegpunkte ohne anzuhalten abfahren soll. Da der Roboter die Stäbe nicht abknicken soll, wenn er eine Scheibe gegriffen hat, oder wenn er sich zwischen zwei Stäben bewegt, sollten hier dem Roboter besser explizit Wegpunkte angegeben werden. Vor allem beim entnehmen einer Scheibe von einem Stapel sollten dem Roboter Wegpunkte entlang des Stabs in Zentimeterabständen angegeben werden, bis er genügend Abstand vom Stab hat. Die Parameter `eef_step` und `jump_threshold` sollen zu `0.01` und `0.0` gesetzt werden.

Die Klasse `HanoiRobot` enthält die Membervariablen `base_pose_` und ein Array `tower_poses_`. Darin sind die Endeffektorposen des Roboters gespeichert, bei denen sich der Greifer entweder oberhalb des mittleren Stabs oder (mit den Fingerspitzen) auf Höhe der untersten Scheibe einer der drei Türme befindet. Auf diese kann man in den Methoden zugreifen, um diese Posen anzufahren, oder ausgehend von diesen andere Posen zu berechnen. Diese sind als `geometry_msgs::PoseStamped` gespeichert. Es reicht jedoch aus, nur den Member `pose` davon zu verwenden, dieser ist vom Typ `geometry_msgs::Pose`.

Die Klasse `Pose`, ist eine von ROS-Messages abgeleitete, automatisch generiert Klasse. Sie enthält die Membervariablen `position` und `orientation`. Dabei enthält `position` wiederum die _doubles_ `x`, `y` und `z`, welche die Translation einer Pose in Weltkoordinaten darstellen. Das Weltkoordinatensystem befindet sich auf dem Boden unter der Basis des Roboters. `orientation` enthält weiterhin die Variablen `x`, `y`, `z` und `w`, die die Orientierung der Pose als [Quaternion](http://www.mathepedia.de/Quaternionen.html "Mathepedia - Quaternionen") darstellen.

# Setzen der Turmposen und der Basispose
Die Basispose, bei der sich der Endeffektor oberhalb aller Türme befindet, und die Pose des Turm 0 (wenn man auf den Roboter schaut, der linke Turm) werden bereits in der `main`-Funktion gesetzt. Die Basispose ist in Gelenkwinkeln angegeben, da diese Darstellung eindeutig ist. So ist sichergestellt, dass der Roboter aus einer Ausgangslage agiert, aus der er nicht so schnell in die Gelenkwinkelbegrenzungen gelangt. Wegen der Redundanz (der Roboter hat 7 Freiheitsgrade) kann eine Pose im kartesischen Raum (6 Freiheitsgrade) durch verschiedene Kombinationen von Gelenkwinkeln erreicht werden. Sie müssen zur ordnungsgemäßen Funktion des Codes noch die Pose der zwei weiteren Türme setzen (in der `main`-Funktion die auskommentierten Zeilen durch richtigen Code ersetzen).

# Ausführen des Codes
## Starten der ROS-Umgebung und benötigter Nodes
Um das geschriebene Programm auszuführen, müssen zuerst die benötigten Nodes gestartet werden. Dazu geht man wie folgt vor:

1. Falls mit dem echten Roboter gearbeitet werden soll, muss zuerst der Roboter hochgefahren werden
2. Im Dateibrowser zum Workspace wechseln, Rechtsklick in einen leeren Bereich -> Terminal hier öffnen
3. Als erstes wird per `roscore` der Hauptprozess von ROS gestartet
4. Mit \[STRG\]+\[SHIFT\]+\[TAB\] einen neuen Tab öffnen, dann über den Befehl `sudoros` Adminrechte für dieses Terminal erlangen und mit diesen dann per `roslaunch robotiq_s_model_control s_model_ethercat.launch` den Node zur Steuerung des Greifers starten. Hier kann es gut sein, dass die EtherCAT Verbindung nicht aufgebaut wird. Dann muss man so lange den Befehl ausführen (mit der Pfeil-nach-oben-Taste den letzten Befehl in den Eingabebereich holen und mit Return bestätigen), bis die Meldung kommt, dass ein Slave gefunden und konfiguriert wurde.
4. Neuen Tab öffnen, per `roslaunch iiwa14_s_model_moveit run_move_group.launch` alle nötigen Nodes und Konfigurierungen starten, um den Roboter anzusteuern. Fall mit dem echten Roboter gearbeitet werden soll, muss an den Befehl ein `sim:=false` angehängt werden!
6. Falls im vorherigen Schritt `sim:=false` gesetzt wurde, muss jetzt die RobotApplication "ROSSmartServo" über das SmartPad des Roboters gestartet werden. Ansonsten wurde die Simulationssoftware Gazebo gestartet, in der ihr sehen könnt, wie sich der Roboter vorraussichtlich bewegen wird.
7. Als Nächstes könnt ihr nun euren Node starten, siehe nächster Abschnitt

## Kompilieren und Starten des Hanoi Nodes
Habt ihr euren Code geändert, müsst ihr ihn erst kompilieren (sowie assemblen und linken). Dazu öffnet ihr wieder einen neuen Tab und führt darin den Befehl `catkin build hanoi_students` aus. Enthält euer Code Fehler, wird der Compiler dies anzeigen und ihr müsst diese erst beheben.

Ist der Code (syntaktisch) fehlerfrei übersetzt worden, könnt ihr ihn mit dem Befehl `roslaunch hanoi_students hanoi.launch` ausführen.

Im Fenster der Visualisierungssoftware RViz seht ihr dann die geplanten Bewegungen. Wenn für einen Befehl der Wert `approvalRequired` auf `true` gesetzt wurde, wird die Bewegung jedoch nicht ausgeführt. Ist man sich sicher, dass die Bewegung Kollisionsfrei durchgeführt werden kann, klickt man dann in RViz auf _Next_.
