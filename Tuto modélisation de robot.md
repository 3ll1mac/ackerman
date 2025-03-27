<span style="color:red">

## Warning
Il est <u>**fortement**</u> recommandé d'être sur un os ubuntu, ou machine virtuelle ubuntu (version 24 de préférence), pour ce skill_book et pour votre projet. Même si vous avez réussi à bidouiller votre chemin jusqu'ici, il existe de nombreux conflits pour télécharger gazebo et les prochaines bibliothèques dont vous aurez besoin. Vous risquez juste de perdre beaucoup de temps à les installer.

</span>

Voici le <a link="https://youtu.be/DhVjgI57Ino">lien</a> d'une vidéo youtube pour installer une machine virtuelle ubuntu 24.0.

Et la <a link="https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html">documentation</a> officielle pour installer ros2.

# Modélisation du robot
# Visualisation du robot

## Warning

Si vous copiez-collez le urdf depuis le skill book directement, les caratères comme '*' et '-' sont mauvais et vous devez les réécrire.

# Simulation du robot

## Les principaux outils de simulation

## Ajout des paramètres physiques pour la simulation dans l’URDF

## Gazebo classique

### 1 -  Warning

Il y a plusieurs versions de gazebo, ici nous allons utiliser la dernière version de gazebo, que l'on retrouve sous le nom de <code>gazebo gz</code> ou juste de <code>gz</code>.

<span style="color:red"> Méfiez-vous si jamais vous voyez juste <code>gazbo</code>dans un code ou  nom d'une bibliothèque à installer. Il s'agit sûrement d'une ancien code sur la vieille version de gazebo qui n'est plus prise en charge.</span>

Pour plus d'information sur toute l'histoire des versions de gazebo, vous pouvez regarder le schéma de la <a link="https://gazebosim.org/about">documentation officielle</a>.

### 2 - Installation gazebo

Voici un <a link="https://gazebosim.org/docs/latest/ros_installation/">lien</a> de la documentation officielle pour l'installation de gazebo selon votre distribution.

### 3 - Odométrie

### 4 - Lancement


### 3 - Partir sur ros_control

Maintenant que vous avez réussi à faire bouger votre robot, cependant vous voyez vite les limites. Avec le differential on plus ou moins obligé de rester sur des robots à deux roues.

Vous pouvez aller découvrir la bibliothèque ros_control pour pouvoir controller et bouger des robots plus complexes.

#### Installation


Voici les ros_control bibliothèques à télécharger, en remplaçant \<distro\> par votre distribution de ros2.


```bash
$ sudo apt install ros-<distro>-gz-ros2-controllers
$ sudo apt install ros-<distro>-gz-ros2-control 
```

#### Documentation

Voici des documentations sur ros_control qui pourrait vous intéresser.

Ce repository github vous montre des démos sur lesquels vous pouvez vous baser : <a link="https://github.com/ros-controls/gz_ros2_control/tree/rolling/gz_ros2_control_demos/examples">ros_control_demo</a>

Vous pouvez jeter un coup d'oeil à cette documentation <a link="https://control.ros.org/rolling/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html#steering-controllers-library-userdoc" >steering_controllers_library</a>, qui est très liée à ros_control. Cette documentation explicite bien sur quel forum envoyé quels messages pouru faire bouger le robot. 

On finit cette partie avec ce <a link="https://fjp.at/posts/ros/ros-control/">website</a> qui explique bien avec des schéma le fonctionnement interne de ros_control.