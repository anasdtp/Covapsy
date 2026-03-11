# CoVAPSy - Architecture Webots

## 📁 Fichiers adaptés pour le simulateur Webots

### Fichiers principaux

| Fichier | Description |
|---------|-------------|
| `controller_jaune.py` | **Point d'entrée Webots** - Lance la simulation et gère le contrôle clavier |
| `robot_base_webots.py` | Adaptateurs pour `Actionneurs` et `CapteurLidar` compatible Webots |
| `conduite_reactive_webots.py` | Logique de conduite réactive (niveau FACILE) sans thread |
| `perception_webots.py` | Analyse du lidar avec paramètres Webots |
| `config_webots.py` | Configuration des paramètres pour le simulateur |

### Fichiers pour Raspberry Pi (ne pas modifier)

| Fichier | Description |
|---------|-------------|
| `main.py` | Point d'entrée pour le Raspberry Pi réel |
| `robot_base.py` | Abstraction hardware pour RPi (PWM + lidar USB) |
| `conduite_reactive.py` | Version multi-thread pour RPi |
| `perception.py` | Version avec paramètres hardware réels |
| `config.py` | Calibration PWM pour la voiture physique |
| `state_machine.py` | Machine à états (pour développement futur) |

---

## 🚀 Utilisation dans Webots

### 1. Lancer la simulation

1. Ouvrir Webots
2. Charger le monde CoVAPSy
3. La simulation démarre avec la voiture à l'arrêt

### 2. Contrôles clavier

| Touche | Action |
|--------|--------|
| **A** | Activer le mode autonome (la voiture démarre) |
| **N** | Désactiver le mode autonome (la voiture s'arrête) |

> ⚠️ **Important** : Cliquez sur la vue 3D avant d'appuyer sur les touches pour que Webots capture les événements clavier.

### 3. Comportement du robot

Quand le mode autonome est activé :

- **Suivi de couloir** : La voiture suit le centre du couloir en comparant les distances à ±60°
- **Ralentissement adaptatif** : La vitesse diminue quand un obstacle est détecté devant
- **Évitement d'obstacles** :
  - Obstacle avant-gauche → correction vers la droite
  - Obstacle avant-droit → correction vers la gauche
  - Obstacle de front → séquence de recul automatique

---

## ⚙️ Configuration

### Paramètres de conduite (config_webots.py)

```python
VITESSE_CROISIERE_M_S  = 0.5   # Vitesse normale (m/s)
DISTANCE_MUR_AVANT_MM  = 800   # Seuil de détection frontal (mm)
DISTANCE_MUR_COTE_MM   = 500   # Seuil de détection latéral (mm)
K_SUIVI_COULOIR        = 0.02  # Gain du correcteur de direction
```

Pour ajuster le comportement, modifier ces valeurs dans `config_webots.py`.

---

## 🔧 Architecture technique

### Flux de données

```
Webots Simulator
    ↓
controller_jaune.py [boucle principale]
    ↓
robot_base_webots.py [lecture lidar + commande moteurs]
    ↓
perception_webots.py [analyse lidar]
    ↓
conduite_reactive_webots.py [décision de commande]
    ↓
robot_base_webots.py [application des commandes]
    ↓
Webots Simulator
```

### Différences clés avec la version Raspberry Pi

| Aspect | Raspberry Pi | Webots |
|--------|--------------|--------|
| **Boucle** | Multi-thread | Synchrone (driver.step()) |
| **Lidar** | RPLidar A2 USB | Simulé (controller.Lidar) |
| **Moteurs** | HardwarePWM | vehicle.Driver |
| **Entrée** | Socket TCP | Clavier Webots |
| **Config** | config.py | config_webots.py |

---

## 🐛 Dépannage

### Le robot ne bouge pas
- ✅ Vérifiez que vous avez appuyé sur **A** pour activer le mode autonome
- ✅ Cliquez sur la vue 3D avant d'appuyer sur les touches
- ✅ Vérifiez les logs dans la console Webots

### Le robot tourne en rond
- Ajuster `K_SUIVI_COULOIR` dans `config_webots.py` (diminuer si trop sensible)
- Vérifier que le lidar détecte bien les murs (logs de distances)

### Le robot recule trop souvent
- Augmenter `DISTANCE_MUR_AVANT_MM` dans `config_webots.py`
- Augmenter `DISTANCE_MUR_COTE_MM` pour les obstacles latéraux

### Erreurs d'import Python
- Les imports `vehicle` et `controller` sont fournis par Webots → normaux en dehors de Webots
- Ne pas exécuter `controller_jaune.py` directement avec Python (utiliser Webots)

---

## 📝 Logs et debug

Les logs sont affichés dans la console Webots avec le format :
```
HH:MM:SS INFO module: message
```

Niveaux de log :
- **INFO** : Démarrage/arrêt, commandes clavier
- **DEBUG** : Détails de la perception et des décisions
- **WARNING** : Problèmes temporaires

Pour activer le mode DEBUG, modifier dans `controller_jaune.py` :
```python
logging.basicConfig(level=logging.DEBUG, ...)
```

---

## 🎯 Prochaines étapes

1. ✅ **Niveau FACILE** : Conduite réactive (implémenté)
2. ⏳ **Niveau MOYEN** : Cartographie SLAM
3. ⏳ **Niveau DIFFICILE** : Planification A* et suivi de trajectoire

Les modules `state_machine.py` et autres fichiers du dossier `base/` sont prêts pour les développements futurs.

---

## 📞 Aide

Pour des questions ou problèmes :
- Consulter le fichier `base/prompt_agent_covapsy.md` pour les spécifications complètes
- Vérifier les logs dans la console Webots
- Tester les paramètres dans `config_webots.py`
