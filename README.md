# Visual_Servoing_UR5_RA_2022-2
En este repositorio se presentan los códigos implementados para la aplicación de Visual Servoing en el brazo robótico UR5 para tomar muestras de racimos de uvas.

## Instalación

Crear un workspace, en este caso lo llamaremos project_ws:
```
cd
 mkdir project_ws
```


Crear y entrar al directorio src.

```
cd project_ws
mkdir src
cd src
```
  
Clonar los siguientes repositorios para tener el modelo del UR5 y el gripper respectivamente. 

```
git clone https://github.com/utecrobotics/ur5
git clone https://github.com/utecrobotics/robotiq
```

Clonar y activar los archivos de la siguiente carpeta donde se encuentran los códigos a correr.

```
cd ~/project_ws/src/ur5/ur5_description
git clone 
chmod a+x *
```

Agregar el .launch que permite visualizar al robot con sliders

```
cd ~/project_ws/src/ur5/ur5_description/launch
git clone
```
  
Construir usando catkin_make.

```
cd ..
catkin_make
```

## Visualización

En caso se use RDS, cada ves que se abra el terminal deberá correr. Caso contrario, se coloca en el archivo .bashrc para que corra cada vez que se inicie un terminal.

```
source project_ws/devel/setup.bash
```

Iniciar visualizador RViz con sliders.
```
roslaunchroslaunch ur5_description display_with_gripper_sliders.launch
```

Iniciar visualizador RViz sin sliders.
```
roslaunchroslaunch ur5_description display_with_gripper.launch
```

### Cinemática directa

### Comprobación del control cinemático 

```
rosrun ur5_description test_fkine.py
```

### Control cinemático de posición y orientación

```
rosrun ur5_description control_cinematico_UR5_pos.py
```
