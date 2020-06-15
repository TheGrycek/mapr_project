# mapr_project

# Dependencies:

$ sudo apt-get install ros-melodic-map-server ros-melodic-dwa-local-planner libompl-dev

$ sudo apt-get install ros-melodic-grid-map

$ pip install pathlib

# Installation

$ sudo apt-get update

$ cd ~/catkin_ws/src

$ git clone https://github.com/Kamilkim/mapr_project.git

$ cd ~/catkin_ws/

$ catkin_make_isolated

$ source devel_isolated/setup.bash

# Run example

Uruchomienie mapy:

$ roslaunch mapr_project mapr_project.launch

Uruchomienie mapy z rysowaniem sciezki

$ roslaunch mapr_project mapr_project_ompl.launch


# Tasks

1. Generowanie mapy - na podstawie informacji z repozytorium https://github.com/ANYbotics/grid_map zostal napisany skrypt elevMap_create.py, ktory generuje mape wysokosciowa (widoczna ponizej) o wymiarach 64x64 piksele. Mapa nastepnie zostala zapisana do rosbaga.

<p align="center"> 
<img src="doc/elevation_map.JPG">
</p>

2. Losowanie startu i mety - skrypt subsykrybuje mape wysokosciowa z uruchomionego rosbaga i losuje na jej powierzchn dwa punkty startowy i koncowy. Zostalo dodane ogranicze ktore powoduje, ze punkty nie moga byc wylosowane blizej siebie wiecej niz na 5 pikseli. Nastepnie punkty sa publikowane w topiku.

<p align="center"> 
<img src="doc/elevation_map_points.JPG">
</p>

3. Szukanie sciezki - do wyszukiwania sciezki zostal uzyty algotyrm RRT* z biblioteki OMPL, dodatkowo sciezka jest optymalizowana pod wzgledem kosztu, ktorym jest wysokosc na mapie w danym punktcie. Do optymalizacji kosztu uzyto rowniez elementu biblioteki OMPL - Optimization Objectives
https://ompl.kavrakilab.org/optimizationObjectivesTutorial.html. Node planera subskrybuje mape z rosbaga i losawane punkty z topika, nastepnie wyszukuje sciezke i publikuje ja w topiku.

<p align="center"> 
<img src="doc/elevation_map_path.JPG">
</p>

4. Zapis mapy - Node z planerem jest uruchamiany co sekunde i z taka czestotliwoscia m sa subskybowane nowe punkty i wyszukiwana miedzy nimi sciezka.


<p align="center"> 
<img src="doc/planning.gif">
</p>

Punkty startowy i koncowy sa zapisywane jako zdjecie. Po znalezieniu sciezki rowniez ona zapisywana jest jako zdjecie. Zbior par zdjec (punktow i sciezek) zostal wykorzystany do uczenia sieci neuronowej, ktora nasladuje uzyty algorytm trasowania sciezki.

5. Model sieci neurownej

<p align="center"> 
<img src="doc/data_point.png" width="128px" height="128px">
<img src="doc/data_path.png" width="128px" height="128px">
</p>




