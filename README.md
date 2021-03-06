# 1. Einleitung
Im Rahmen dieses Versuchs soll das Knobelspiel [Türme von Hanoi](https://de.wikipedia.org/wiki/T%C3%BCrme_von_Hanoi "Wikipedia: Türme von Hanoi") mit einem Roboterarm mit sieben Freiheitsgraden gelöst werden. Dazu muss nicht nur ein Algorithmus zur Lösung des theoretischen Problems implementiert werden, sondern auch die Schwierigkeiten gelöst werden, die bei der Bedienung eines Roboters bei einer solchen Aufgabenstellung entstehen.

Ziel des Rätsels ist, einen Stapel Scheiben von einem Turm zu einem anderen zu verschieben und bei jedem Zug stets nur eine Scheibe zu verschieben. Weiterhin dürfen Scheiben nur auf einen leeren Stab oder auf einem mit einer größeren Scheibe abgelegt werden.

## 1.1 KUKA LBR iiwa
Der eingesetzte Roboterarm ist ein Leichtbauroboter _intelligent industrial work assistant_ (LBR iiwa) der Firma KUKA. Dieser ist ein serieller siebenachsiger Knickarmroboter. Jede der Achsen verfügt über einen Drehfreiheitsgrad. Aufgrund seiner sieben Freiheitsgrade (engl. Degrees of freedom, DOF) steht dem LBR iiwa ein redundanter DOF bei der Bewegungsausführung zur Verfügung.

Das gesamte Robotersystem besteht aus mehreren Komponenten. Zu diesen zählen der Manipulator selbst, welcher die Robotermechanik und Elektronik beinhaltet, der zugehörige KUKA Sunrise Cabinet Controller, auf welchem sich die robotereigene Regelung befindet sowie die Software KUKA Sunrise.OS. Zusätzlich ermöglicht das Control Panel (smartPad) die Ansteuerung und Überwachung des Roboters. Die hierzu benötigten Statusdaten des Manipulators, wie die momentanen Achspositionen und die wirkenden Momente in den einzelnen Gelenken, werden mithilfe von Drehmoment- und Achsbereichssensoren ermittelt. Diese sind den Servos der einzelnen Gelenke nachgeschaltet. Seine Leichtbauweise in Kombination mit der Drehmomentsensorik, welche die Möglichkeit der Einstellung gewünschter Momente mit einer Toleranz von +-2 % bietet, bringt eine hohe Genauigkeit bei der Kraftregelung des Roboters auch bei sensiblen Aufgaben mit sich. Aufgrund dieser Eigenschaft ist der Roboter besonders gut für Aufgaben im Bereich der Mensch-Roboter Kollaboration geeignet.

Des Weiteren verfügt der Roboter über mehrere Betriebsmodi. Im positionsgeregelten Modus lassen sich Positionen mit gewünschten Geschwindigkeiten anfahren, während sich im kraftgeregenlten Modus die gewünschte Kontaktkrafte in einer kartesischen Richtung vorgeben lässt. Zwei impedanzgeregelte Modi ermöglichen zusätzlich die Vorgabe der Nachgiebigkeiten in den einzelnen Gelenkachsen oder in den sechs kartesischen Freiheitsgraden des Roboters durch den Nutzer.

Für den Praktikumsversuch wird als Manipulator ein Dreifingergreifer von _Robotiq_ verwendet, der für den Versuch vormontiert ist.

## 1.2 ROS-Umgebung
Das Gesamtsystem ist in einer [ROS](http://www.ros.org/ "ROS-website")-Umgebung eingebunden. Dies ist ein Open-Source Software-Framework für Roboteranwendungen. Es ermöglicht durch die Nutzung seiner Vielzahl an Software-Bibliotheken, Werkzeugen und Konventionen die Erstellung robuster und komplexer plattformunabhängiger Anwendungen für die Robotik. Durch seine freie Verfügbarkeit und Modularität können einzelne Programmteile von seinen Nutzern hinzugefügt, erweitert und flexibel für ihre Anwendungsfälle genutzt und kombiniert werden, wodurch die kollaborative Weiterentwicklung von Robotiksoftware gefördert wird. Im vorliegenden Fall wird die Distribution _Kinetic Kame_ verwendet.

### 1.2.1 Architektur
Einzelne Prozesse, sogenannte _Nodes_, können bestimmte Aufgaben, wie Hardwareabstraktion, reine Datenverarbeitung (Verarbeitung von Sensordaten) oder Bereitstellung von Statusdaten des Roboters, übernehmen und sich über Nachrichten austauschen. Die einzelnen Prozesse/Aufgaben sind in Paketen organisiert. Für die Ausführung eines Nodes ist der Start eines ROScore notwendig. Dieser registriert als Namensdienst von ROS alle laufenden Nodes und ermöglicht diesen somit das gegenseitige Auffinden und den Datenaustausch untereinander. Zusätzlich startet der Core den Parameterserver, welcher einen zentralen Speicherort für Informationen darstellt. Alle Nodes können auf dessen Inhalte zugreifen und diese gegebenenfalls verändern. Der Start des Nodes selbst erfolgt vorwiegend mithilfe sogenannter Launch-files. Diese ermöglichen sowohl das gleichzeitige Starten mehrerer Nodes also auch die Übergabe von Parametern beim Start.

Die Interprozesskommunikation findet über sogenannte Topics, Services und Actions statt. Über diese können Nodes vordefinierte Datenstrukturen, die sogenannte Messages, mit einander austauschen.

### 1.2.2 Dateisystem
Den Grundbaustein des Dateisystems stellen die sogenannten Packages dar, in welchen die in ROS genutzten Softwarebausteine organisiert sind. Sie können diverse Elemente wie Datensätze, Bibliotheken, Konfigurationsdateien oder Nodes enthalten, welche als gemeinsamer Baustein zur Verfügung gestellt werden sollen. Ihnen verdankt ROS somit seine Modularität. Mehrere zusammengehörige Packages lassen sich auch in einem Metapackage (_Stack_) zusammenfassen. Beispielsweise werden [Robotiq](https://github.com/ros-industrial/robotiq)-Pakete zur Ansteuerung des Greifers über EtherCAT und Pakete des [iiwa_Stacks](https://github.com/IFL-CAMP/iiwa_stack) zur Ansteuerung des Roboters verwendet.

### 1.2.3 Bewegungsplanung mit MoveIt! und Visualisierung mit Rviz
Zur Bahnplanung des Roboters wird das [MoveIt! Motion Planning Framework](https://moveit.ros.org/), welches aufbauend auf dem Kommunikationskonzept und Buildsystems von ROS eine Sammlung von Paketen zur Bewegungsplanung und Objektmanipulation bereitstellt, verwendet. Diese enthalten Software zur Pfad- und Trajektorienplanung und der damit einhergehenden Berechnung von Vorwärts- und Inverskinematik, sowie der Kollisionserkennung, 3D Objektwahrnehmung und Navigation.

Zentrales Element der MoveIt! Architektur ist der `move_group` Node. Dieser dient der Integration verschiedener Informationszweige. Zu diesen gehören unter anderem Informationen bezüglich der aktuellen Lage, Geschwindigkeit und Beschleunigung der  einzelnen  Gelenke  des  Roboters,  Daten  externer  Sensoren  zur  Umgebungserfassung, sowie Angaben über vorhandene Transformationen (TF). Anhand dessen stellt dieser dem Benutzer entsprechende Actions und Services für die Ansteuerung des Roboters zur Verfügung. Die kinematische Beschreibung des Roboters, sowie Informationen über dessen maximal zulässige Gelenkwinkel und Beschleunigungen bezieht der `move_group` Node über den Parameterserver,  anhand  der  dort  hinterlegten  URDF,  SRDF  (Semantic  Robbt  Description Format) und Konfigurationsdateien.

Für die Interaktion mit dem Roboter mittels MoveIt! stehen drei verschiedene Interfaces zur Auswahl. Zwei Anwendungsprogrammierschnittstellen (API) zur Programmierung in C++ oder Python sowie eine grafische Benutzeroberfläche (GUI), das ROS Visualisierungstool (Rviz). Mithilfe dieser Interfaces lassen sich Anfragen zur Planung und Ausführung von Bewegungen an den `move_group`
Node übergeben. Als reiner Integrator führt dieser Node selbst keinerlei Algorithmen aus. Stattdessen werden einzelne Funktionalitäten, wie die Pfadplanung oder die Lösung von Vorwärts- und Inverskinematik (engl. Kinematic solver), anhand zusätzlicher Plugins realisiert.

Das Rviz Plugin lässt sich ebenfalls in Kombination mit der C++ bzw. Python API zur Visualisierung der geplanten Trajektorie nutzen. Hierzu muss die entsprechende Bibliothek zur Bereitstellung der `rviz_visual_tools`, einer Auswahl von Anzeigeobjekten und Interaktionstools, eingebunden werden. Zusätzlich zu dem Planungsvorhaben lassen sich in Rviz zudem Kollisionsobjekte hinzufügen, die vorhandenen Transformationen Anzeigen und die Inhalte anderer ROS Topics visualisieren.

Um die Einarbeitungszeit zu verkürzen und verschiedene Ansteuerungsarten des Roboters vereinheitlicht zu verwenden, wurden die benötigten Funktionen noch weiter abstrahiert und stehen in der Klasse _RobotInterface_ im Paket [iimoveit](https://github.com/KUKAnauten/iimoveit) zur Verfügung. Eine Verknüpfung zur Dokumentation der API befindet sich im Paketordner von __hanoi\_students__ (siehe übernächster Absatz) oder ist [hier zu erreichen](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API").

### 1.2.4 Workspace
In ROS arbeitet man in sogenannten _Workspaces_. Für diesen Versuch wurde bereits ein Workspace eingerichtet, in dem die oben beschriebenen Pakete bereits enthalten sind. Auch wurde ein Paket erstellt, in dem sich der _Node_ befindet, der den Roboter und den Greifer steuern soll. Der Code dieses Nodes muss von Euch vervollständigt werden, sodass der Roboter die Türme von Hanoi erfolgreich löst.

Euer Workspace heißt __hanoi\_ws__ und befindet sich unter __/home/mlab8/ROS/__. Ihr werdet nur die Datei __hanoi\_iiwa\_node.cpp__ bearbeiten, die sich im Ordner __src__ des Pakets __hanoi\_students__ befindet. Diese kann mit einem Texteditor eurer Wahl geöffnet werden. Vorinstalliert ist Sublime Text, der auch Syntax Highlighting beherrscht.

## 1.3 Grundlagen
Für das Arbeiten mit und an dem Roboter sind einige Grundlagen der Roboik sowie mathematische Grundlagen notwendig. In diesem Abschnitt wird knapp auf die relevanten Grundlagen eingegangen. Für tiefergehendes Verständnis wird auf [1] verwiesen.

### 1.3.1 Koordinatentransformationen und Pose
Ein Roboter besteht aus einer definierten Anzahl Glieder. Serielle Manipulatoren zeichnen sich dadurch aus, dass zwei aufeinanderfolgende Glieder stets durch exakt ein gemeinsames Gelenk miteinander in Verbindung stehen. Es lassen sich sowohl translatorische (Lineargelenk), wie auch rotatorische (Drehgelenk) Ausführungen unterscheiden, wobei sich beim Aufbau serieller Roboter das Drehgelenk etabliert hat. So handelt es sich auch bei der Verbindung der Glieder des genutzten Roboters ausschließlich um Drehgelenke, weshalb sich im Folgenden auf deren Beschreibung beschränkt wird. Das letzte Glied der kinematischen Kette wird als Endeffektor bezeichnet. Diese Bezeichnung ist sowohl für das Endstück des Roboters, beispielsweise den Flansch, als auch für ein Werkzeug (Tool) gebräuchlich, welches gegebenenfalls zusätzlich an einem Flansch befestigt ist.

Für die Umsetzung vorgegebener Aufgaben und Bewegungsabläufe des Endeffektors ist das Wissen über dessen Pose von Bedeutung. Die Pose eines Körpers wird stets durch seine relative Lage zu einem anderen Körper beschrieben. Die Pose beinhaltet hierbei sowohl die relative Position als auch Orientierung des Körpers. Für den Anwendungsfall eines seriellen Manipulators bedeutet dies, dass die Lage des körperfesten Koordinatensystems _S_B_ (engl. frame) eines Gliedes B durch die beiden genannten Komponenten bezüglich eines Referenzsystems, des Koordinatensystems _S_A_ des vorherigen Körpers A, festgelegt ist. Die Position von _S_A_ gegenüber _S_B_ ist charakterisiert über eine Translation, darstellbar durch den Vektor _p_ (Element R^3), wobei die Indizes _A_ und _B_ für die zwei Koordinatensysteme stehen:

![translationsteil_pose](https://user-images.githubusercontent.com/38288915/38779231-c6bf8252-40c5-11e8-83d7-7b89900c703e.PNG).

Die Orientierung des körperfesten gegenüber des Referenzkoordinatensystems durch die (3x3) Rotationsmatrix _R_ beschreiben:

![rotationsteil_pose](https://user-images.githubusercontent.com/38288915/38779248-1336e12a-40c6-11e8-8271-0149b23c3c10.PNG).

Ist ein Punkt _d_ bezüglich _S_B_ gegeben, lässt sich dieser relativ zu Bezugssystem _S_A_ mithilfe
der folgenden Transformationsbeziehung ausdrücken (vergleiche Abbildung).

![transformationsbeziehung](https://user-images.githubusercontent.com/38288915/38779297-91b1b192-40c6-11e8-8d6a-add71dd671e8.PNG)

![transformationsbeispiel](https://user-images.githubusercontent.com/38288915/38779526-f8ada9c0-40c9-11e8-8dfa-9ab866b8d6d9.PNG)

### 1.3.2 Euler-Winkel
Eine weitere Möglichkeit zur Angabe der Orientierung bietet die Euler-Winkel-Darstellung. Mit dieser lässt sich die 3x3 Rotationsmatrix aus dem vorherigen Abschnitt auf eine Drehung _Theta_ (Element R^3) mit insgesamt drei Rotationen darstellen. Dabei wird als erstes das Koordinatensystem um die z-Achse um den Winkel _alpha_ gedreht. Danach wird um die y'-Achse (resultierend aus der ersten Drehung) um den Winkel _beta_ gedreht. Schließlich wird noch um die z''-Achse (resultierend aus der zweiten Drehung) um den Winkel _gamma_ gedreht. Dies wird auch als ZYZ-Euler-Winkel-Transformation bezeichnet.

Eine spezielle Variante der Euler-Winkel-Darstellung ist die Roll-Pitch-Yaw-Darstellung (Roll-Nick-Gier). Dabei wird um alle drei Achsen nacheinander um die Wikel _alpha_, _beta_ und _gamma_ gedreht.

### 1.3.3 Arbeitsräume (Joint Space und Task Space)
Der Joint Space beschreibt die Konfiguration der gesamten kinematischen Kette des Roboters anhand seiner Gelenkparameter _qi_. Diese enthalten im Falle rotatorischer Gelenke ihre jeweiligen Drehwinkel. Seine Dimension entspricht der Zahl der _n_ Freiheitsgrade des Roboters und wird durch den Vektor _q = (q1 . . . qn)T_ (T steht für transponiert) (Element R^n) gebildet.

Der Taskspace, auch Operational Space oder Arbeitsraum genannt, dient hingegen der Beschreibung der Pose des Endeffektors bezüglich der Basis des Roboters in kartesischen Koordinaten. Die Dimension des Arbeitsraumes entspricht der Anzahl der _m_ DOF, welche zur Erledigung der vorgegebenen Aufgabe zur Verfügung stehen. Bei einer Aufgabe im dreidimensionalen, wie z.B. dem Führen und Ausrichten einer Nadel, sind dies m = 6 Freiheitsgrade, je drei für Position und Orientierung. Die Konfiguration des Endeffektors _x_ee_, wird somit anhand seiner generalisierten Koordinaten bezüglich Position _p_ee_ und Orientierung _theta_ee_ beschrieben.

Die Pose im Task Space lässt sich mithilfe der nichtlinearen Funktion _f(*)_ in Abhängigkeit der Joint Space Parameter angeben als _x = f(q)_.

# 2. Vorbereitung
Die Vorbereitungsaufgaben sollen das Verständnis weiter fördern und erfordern Recherche! Dazu können sowohl [1] als auch eigene Recherchen zu den Thematiken weiterhelfen.

1. Was bedeutet Redundanz speziell im Sinne der Robotik und was kann damit bewirkt werden?

2. Was sind homogene Koordinaten und welchen Vorteil bringen sie mit sich?

3. Stellen Sie die Rotationsmatrizen für Rotationen um die x-, y- und z-Achse auf. Wie lautet die Gesamtrotationsmatrix der ZYZ-Euler-Winkel-Transformation? (Hinweis: Nacheinanderfolgende Rotationen werden durch Multiplikation der jeweiligen Rotationsmatrizen dargestellt)

4. Machen Sie sich mit dem Knobelspiel Türme von Hanoi vertraut. Hier finden Sie ein gutes Beispiel zum [Testen der Türme von Hanoi](https://www.mathematik.ch/spiele/hanoi_mit_grafik/). Verdeutlichen Sie sich das Prinzip der Rekursion und schreiben Sie ein Python-Skript zum Lösen des Knobelspiels. Zum Anzeigen kann der _print_-Befehl, der dann bei der Ausführung des Programms zeigen sollte, welches Plättchen von einem zum anderen Turm bewegt wird. Als IDE für Python wird PyCharm empfohlen (alternativ auch WinPy).

5. Da die Klasse [RobotInterface](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API") im Praktikumsversuch genutzt wird, sollten Sie sich mit den Methoden und Eigenschaften dieser Klasse vertraut machen.

# 3. Versuchsnachmittag
## 3.1 Programmstruktur
Zur Durchführung der Aufgabe wurde die Klasse `HanoiRobot` erstellt, die alle Methoden von [RobotInterface](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API") erbt.

___Achtung!___ Methoden, die mit _publish_ oder _run_ beginnen, sollen im Rahmen dieses Versuchs jedoch ignoriert werden, denn hierbei wird der Bahnplanungsschritt umgangen und kann zu unkontrollierbarem Verhalten führen! MoveIt! kann zwar Trajektorien berechnen, die Kollisionen vermeiden, jedoch müssen dazu Objekte über Sensoren erkannt werden oder per Hand in die Planungsumgebung eingefügt werden. Dies ist in unserer Umgebung noch nicht der Fall, weshalb geplante Bewegungen vor der Ausführung bestätigt werden müssen und auch zuerst im Simulator Gazebo geprüft werden sollen.
Zum Bewegen sollten momentan am besten nur die Methoden `planAndMove`, `planAndMoveToBasePose` sowie `moveAlongCartesianPathInWorldCoords` verwendet werden. Diese sind zur Durchführung des Versuchs ausreichend und andere Methoden befinden sich noch in experimentellem Status. In diesem Versuch müssen folgende Methoden implementiert werden:
- `void moveSlice(int from, int to)`
- `void moveTower(int height, int from, int to, int with)`

Die Methode `moveTower` soll den Algorithmus enthalten und dabei die Methode `moveSlice` verwenden, in der der Roboter eine Scheibe von einem zum anderen Stab bewegen soll. Zu beachten ist, dass beim Bewegen des Roboters mit `planAndMove` nur die Anfangs- und Endpose vorgegeben sind. Dazwischen bewegt sich der Roboter nicht auf einer geraden Linie, sondern so, wie es von den Gelenken am besten passt - dabei gibt es natürlich unendlich viele Lösungen, jenachdem, mit welchem Gütemaßen man die Planung durchführt. Weiterhin hält der Roboter bei Erreichen der Zielpose an, anstatt bestimmte Wegpunkte einfach zu durchfahren.

## 3.2 Verwendung von `planAndMove`und `moveAlongCartesianPathInWorldCoords`
Diese Methoden sollen zur Bewegungsplanung- und Durchführung verwendet werden.  Es reicht aus, von den überladenen Methoden `planAndMove(PoseStamped &target_pose, bool approvalRequired)` zu verwenden. Diese Methode erwartet eine `geometry_msgs::PoseStamped` als Parameter und einen `bool`, der festlegt, ob vor der Ausführung der Bewegung eine manuelle Bestätigung erforderlich ist. `moveAlongCartesianPathInWorldCoords` erwartet einen `std::vector<geometry_msgs::Pose>`, was etwa einer `List` in Java entspricht, welche einzelne Zielposen enthält. Das ist dann nützlich, wenn man den Roboter entlang einer Bahn bewegen will, oder wenn der Roboter Wegpunkte abfahren soll ohne anzuhalten. Da der Roboter die Stäbe nicht abknicken soll, wenn er eine Scheibe gegriffen hat, oder wenn er sich zwischen zwei Stäben bewegt, sollten hier dem Roboter besser explizit Wegpunkte angegeben werden. Vor allem beim Entnehmen einer Scheibe von einem Stapel sollten dem Roboter Wegpunkte entlang des Stabs in Zentimeterabständen angegeben werden, bis er genügend Abstand vom Stab hat. Die Parameter `eef_step` und `jump_threshold` sollen zu jeweils `0.01` und `0.0` gesetzt werden.

Die Klasse `HanoiRobot` enthält die Membervariablen `base_pose_` und ein Array `tower_poses_`. Darin sind die Endeffektorposen des Roboters gespeichert, bei denen sich der Greifer entweder oberhalb des mittleren Stabs oder (mit den Fingerspitzen) auf Höhe der untersten Scheibe einer der drei Türme befindet. Auf diese kann man in den Methoden zugreifen, um diese Posen anzufahren, oder ausgehend von diesen andere Posen zu berechnen. Diese sind als `geometry_msgs::PoseStamped` gespeichert. Es reicht jedoch aus, nur den Member `pose` davon zu verwenden, dieser ist vom Typ `geometry_msgs::Pose`.

Die Klasse `Pose`, ist eine von ROS-Messages abgeleitete, automatisch generiert Klasse. Sie enthält die Membervariablen `position` und `orientation`. Dabei enthält `position` wiederum die _doubles_ `x`, `y` und `z`, welche die Translation einer Pose in Weltkoordinaten darstellen. Das Weltkoordinatensystem befindet sich auf dem Boden unter der Basis des Roboters. `orientation` enthält weiterhin die Variablen `x`, `y`, `z` und `w`, die die Orientierung der Pose als [Quaternion](http://www.mathepedia.de/Quaternionen.html "Mathepedia - Quaternionen") darstellen.

## 3.3 Setzen der Turmposen und der Basispose
Die Basispose, bei der sich der Endeffektor oberhalb aller Türme befindet, und die Pose des Turm 0 (wenn man auf den Roboter schaut, der linke Turm) werden bereits in der `main`-Funktion gesetzt. Die Basispose ist in Gelenkwinkeln angegeben, da diese Darstellung eindeutig ist. So ist sichergestellt, dass der Roboter aus einer Ausgangslage agiert, aus der er nicht so schnell in die Gelenkwinkelbegrenzungen gelangt. Wegen der Redundanz (der Roboter hat 7 Freiheitsgrade) kann eine Pose im kartesischen Raum (6 Freiheitsgrade) durch verschiedene Kombinationen von Gelenkwinkeln erreicht werden. Sie müssen zur ordnungsgemäßen Funktion des Codes noch die Pose der zwei weiteren Türme setzen (in der `main`-Funktion die auskommentierten Zeilen durch richtigen Code ersetzen).

## 3.4 Ausführen des Codes
### 3.4.1 Starten der ROS-Umgebung und benötigter Nodes
Um das geschriebene Programm auszuführen, müssen zuerst die benötigten Nodes gestartet werden. Dazu geht man wie folgt vor:

1. Falls mit dem echten Roboter gearbeitet werden soll, muss zuerst der Roboter hochgefahren werden
2. Im Dateibrowser zum Workspace wechseln, Rechtsklick in einen leeren Bereich -> Terminal hier öffnen
3. Als erstes wird per `roscore` der Hauptprozess von ROS gestartet
4. Mit \[STRG\]+\[SHIFT\]+\[TAB\] einen neuen Tab öffnen, dann über den Befehl `sudoros` Adminrechte für dieses Terminal erlangen und mit diesen dann per `roslaunch robotiq_s_model_control s_model_ethercat.launch` den Node zur Steuerung des Greifers starten. Hier kann es gut sein, dass die EtherCAT Verbindung nicht aufgebaut wird. Dann muss man so lange den Befehl ausführen (mit der Pfeil-nach-oben-Taste den letzten Befehl in den Eingabebereich holen und mit Return bestätigen), bis die Meldung kommt, dass ein Slave gefunden und konfiguriert wurde.
4. Neuen Tab öffnen, per `roslaunch iiwa14_s_model_moveit run_move_group.launch` alle nötigen Nodes und Konfigurierungen starten, um den Roboter anzusteuern. Fall mit dem echten Roboter gearbeitet werden soll, muss an den Befehl ein `sim:=false` angehängt werden!
6. Falls im vorherigen Schritt `sim:=false` gesetzt wurde, muss jetzt die RobotApplication "ROSSmartServo" über das SmartPad des Roboters gestartet werden. Ansonsten wurde die Simulationssoftware Gazebo gestartet, in der ihr sehen könnt, wie sich der Roboter voraussichtlich bewegen wird.
7. Als Nächstes könnt ihr nun euren Node starten, siehe nächster Abschnitt

### 3.4.2 Kompilieren und Starten des Hanoi Nodes
Habt ihr euren Code geändert, müsst ihr ihn erst kompilieren (sowie assemblen und linken). Dazu öffnet ihr wieder einen neuen Tab und führt darin den Befehl `catkin build hanoi_students` aus. Enthält euer Code Fehler, wird der Compiler dies anzeigen und ihr müsst diese erst beheben.

Ist der Code (syntaktisch) fehlerfrei übersetzt worden, könnt ihr ihn mit dem Befehl `roslaunch hanoi_students hanoi.launch` ausführen.

Im Fenster der Visualisierungssoftware RViz seht ihr dann die geplanten Bewegungen. Wenn für einen Befehl der Wert `approvalRequired` auf `true` gesetzt wurde, wird die Bewegung jedoch nicht ausgeführt. Ist man sich sicher, dass die Bewegung kollisionsfrei durchgeführt werden kann, klickt man dann in RViz auf _Next_.

# 4. Quellen
[1] Spong, Mark W.: _Robot Modeling and Control_, 1. Edition, 2005

# 5. Autorenschaft
Diese Anleitung und Einführung ist geschrieben von Laura Bielenberg, Romol Chadda, Marcus Ebner und Markus Hessinger.
