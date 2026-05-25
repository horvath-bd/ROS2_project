# ROS2 és Gazebo Játék Harckocsi Szimuláció 

## Tartalomjegyzék
1. [Projekt leírása](#1-projekt-leírása)
2. [Fejlesztési lépések és architektúra](#2-fejlesztési-lépések-és-architektúra)

## 1. Projekt leírása
Játék harckocsi szimulációja Gazebóban.

## 2. Fejlesztési lépések és architektúra
### 3D Modellezés és Textúrázás (Blender)
A harckocsi modellje Blenderben készült. A Gazebo kompatibilitás érdekében a textúrák PBR (Physically Based Rendering) baking eljárással lettek kisütve 
(Albedo, Normal, Roughness, Emissive csatornák).

-Cyclone nélkül a gazebo elszáll a retekbe
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc