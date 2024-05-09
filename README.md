# Tactile exploration of 3D surfaces/objects with an Optoforce sensor and the UR5 robotic arm

In this case the Optoforce sensor is moved and touched to the object directly with the UR5 robotic arm. The knowledge of the object’s rough boundary can be assumed, the main focus should be on the more detailed / precise surface information of the given object. The UR5’s endeffector (EE) can be directly moved to different location-orientation combinations.
This spatial data (location +orientation) has to be co-registered with the measurement of the Optoforce sensor.

Some of the estimated subtasks:
* maybe 3D printing or mechanical design to fix the Optoforce to the UR5’s EE;
* control of the UR5 arm (most probably you will roughly learn it from the infra measurement);
* preliminary trajectory generation to the EE (on the basis of the rough surface information of the object);
* data fusion to the EE data and the Optoforce data.

## Steps
* [Récupération du fichier Solidworks robot UR5 de Universal Robotics](https://grabcad.com/library/ur5-8)
* Modification du fichier Solidworks pour l'adapter a nos besoins
    * Simplification du modèle - conversion des assemblages en fichier pièces
    * Modification de la position des articulations et des limites virtuelles
* Conception du support du capteur Optoforce afin de le fixer sur le robot UR5
* Ajout du fichier dans l'assemblage Solidworks
* Conversition du nouvel assemblage [Solidworks en URDF avec les Meshes](https://github.com/ros/solidworks_urdf_exporter/releases)
* Simulation Pybullet
    * Import du fichier URDF dans pybullet
    * Récupération des informations critiques du robot
    * Déplacement indépendant des différents joints

### Simulation
* Installer l'outil pybullet
* Commencer a explorer l'outils pybullet via le [guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit)
    * Comment lancer une simulation avec un URDF en activant les différents paramètres d'initialisation de l'environnement ?
    * Comment faire bouger un joint spécifique ?
    * Comment récupérer les donnnées critiques d'un joint ?
    * Explorer les différents outils de debug (pour l'affichage des links par exemple...)
    * Explorer les différents concepts d'objet déformable (pour l'optoforce)
    * Explorer rapidement les concept de reinforcement learning (lien avec ton stage)
* Expérimenter l'inverse_kinematics automatique via pybullet avec les données issue du modèle URDF
* Expérimenter l'inverse_dynamics automatique via pybullet avec les données issue du modèle URDF
* Voir comment récupérer les données de colisions du end effector issue de la simulation
* Générer une trajectoire sur pybullet
* Générer des graphs pour chaque joints et pour la trajctoire du robot d'un temps t1 donné à un temps t2 donné (on peut aussi imaginer des fonction du type "start_reccord" et "end_reccord" qui change la valeur d'un boolen (passe de True à False ou inversement) et permet d'enregister ou de de ne plus enregistrer les données)
* Imaginer un algorithme pour détecter un objet et l'implémenter

### Réel 
* Voir comment récuperer les données de l'optoforce via python et créer une classe pour faciliter son utilisation
* Voir comment commander le robot réel en python (voir [lib ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html)) et créer une classe pour faciliter son utilisation
