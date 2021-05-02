# EI8IT246 Introduction à la robotique

Cours d'introduction à la robotique du semestre 8 de la filière informatique à l'ENSEIRB-MATMECA. Réalisation d'un
contrôle d'un hexapode.

## Équipe

+ Clément Barbara
+ Nathan Decou
+ Sébastien Delpeuch
+ Marc Duclusaud
+ Antoine Pringalle

## Théorie

Les éléments théoriques permettant de mettre en place le modèle direct et le modèle indirect pour une patte d'un robot (
hexapode ou quadrupède) sont disponibles [ici](https://sdelpeuch.github.io/assets/md/robotique/1/)

## Quadrupède

Les comportements de base sur le quadrupède permettant de nous familiariser
avec `pybullet` (`direct, inverse, draw, step, legs, walk`) sont executables depuis le dossier `quadruped/` avec la
commande `python3 simulator.py <name>`

## Projet : Hexapode

L'objectif du projet est de passer ce que nous avons fait sur un robot hexapode en simulation puis en réel. Par manque
de temps les fonctions sur l'hexapode sont disponibles uniquement en simulation.

### Fonctions de base

- Moving an arbitrary leg to an arbitrary (x, y, z) position ✅ 
- Moving the center of the robot to an arbitrary (x, y, z) position (the 6 legs staying on the floor) ✅ `python3 sim_hexa.py -m robot-ik-keyboard`
- Walking in a straight line (any direction) ✅ `python3 sim_hexa.py -m walk`
- Rotating without moving the center of the robot (the tip of the legs should never slide on the floor!) ✅ `python3 sim_hexa.py -m rotate`
- Combining the 2 previous behaviors to create an holonomic walk

### Fonctions avancées

- Controlling the robot with a keyboard/mouse/whatever ✅ `python3 sim_hexa.py -m holo`
- Odometry (the robot tracks his position based on the movements he does)
- Go to X, Y, Theta.
- Configurable walk (speed, frequency, length, height of the steps. Default body position, etc) ✅ `python3 sim_hexa.py -m walk-configurable`
- Legs move smoothly (ask me about this) 
- The robot can change its direction on the go