# CoVAPSy — Voiture Autonome

Projet de voiture autonome pour la compétition **CoVAPSy** (Paris-Saclay).
Châssis Tamiya TT02, piloté par un Raspberry Pi 4 avec un lidar RPLidar A2M12.

---

## Structure du projet

```
Covapsy/
├── # ── Version Raspberry Pi (voiture réelle) ──────────────────
├── main.py                      # Point d'entrée — commandes GO/STOP/QUIT dans le terminal
├── config.py                    # Tous les paramètres calibrés (PWM, vitesses, seuils)
├── robot_base.py                # Abstraction matérielle : PWM + acquisition lidar (thread)
├── perception.py                # Analyse du tableau lidar (détection obstacles, suivi couloir)
├── conduite_reactive.py         # Logique de conduite autonome (thread)
├── state_machine.py             # Machine à états de conduite
│
├── # ── Version Webots (simulateur) ────────────────────────────
├── controller_jaune.py          # Point d'entrée Webots — touches A / N
├── config_webots.py             # Paramètres adaptés au simulateur
├── robot_base_webots.py         # Adaptateurs Webots (Driver + Lidar simulé)
├── perception_webots.py         # Perception adaptée à l'API lidar Webots
├── conduite_reactive_webots.py  # Conduite synchrone (sans thread)
│
├── # ── Scripts de test et calibration ─────────────────────────
└── base/
    ├── test_lidar.py            # Affichage polaire des données lidar (matplotlib)
    ├── test_pwm_direction.py    # Calibration interactive des butées de direction
    ├── test_pwm_propulsion.py   # Calibration interactive du variateur
    ├── raz_lidar.py             # Réinitialisation du lidar (résout Descriptor mismatch)
    └── commande_PS4.py          # Contrôle manuel via manette PS4
```

---

## Matériel

| Composant | Détails |
|---|---|
| Châssis | Tamiya TT02 |
| Ordinateur | Raspberry Pi 4 Model B |
| Lidar | RPLidar A2M12 — `/dev/ttyUSB0` — baudrate 256000 |
| Propulsion | HardwarePWM channel 0 — 50 Hz |
| Direction | HardwarePWM channel 1 — 50 Hz |

---

## Lancer la voiture réelle

Se connecter en ssh sur la raspberry pi.

### Prérequis
```bash
pip install rplidar rpi-hardware-pwm
```

### Démarrage
```bash
python main.py
```

Commandes disponibles dans le terminal :

| Commande | Action |
|---|---|
| `GO` | Démarre la conduite autonome |
| `STOP` | Arrête la voiture |
| `QUIT` | Arrête et quitte le programme |

> La voiture ne bouge pas avant la commande `GO` (conformité règlement CoVAPSy).

---

## Lancer la simulation Webots

### Prérequis
- [Webots R2023b](https://cyberbotics.com/) installé
- Monde CoVAPSy chargé dans Webots
- `controller_jaune.py` défini comme contrôleur du robot TT02 jaune

### Démarrage
1. Ouvrir Webots et charger le monde CoVAPSy
2. Lancer la simulation
3. **Cliquer sur la vue 3D** pour activer la capture clavier

| Touche | Action |
|---|---|
| `A` | Active le mode autonome |
| `N` | Désactive le mode autonome |

---

## Architecture logicielle

### Comportement de conduite

La logique de conduite est identique entre la version réelle et Webots :

```
Nouveau scan lidar disponible
    │
    ├─[obstacle frontal < DISTANCE_MUR_AVANT_MM]──→ Séquence de recul
    │     Choisir le côté avec le plus d'espace
    │     Reculer + braquer + repartir tout droit
    │
    ├─[obstacle avant-droit < DISTANCE_MUR_COTE_MM]──→ Correction gauche
    ├─[obstacle avant-gauche < DISTANCE_MUR_COTE_MM]─→ Correction droite
    │
    └─[couloir libre]──→ Suivi de couloir
          angle = K_SUIVI_COULOIR × (lidar[+60°] − lidar[−60°])
          vitesse = VITESSE_CROISIERE (réduite si obstacle proche)
```

### Différences Raspberry Pi vs Webots

| Aspect | Raspberry Pi | Webots |
|---|---|---|
| Boucle principale | Multi-thread (lidar + conduite) | Synchrone (`driver.step()`) |
| Lidar | RPLidar A2M12 USB | Lidar simulé (`controller.Lidar`) |
| Moteurs | `rpi_hardware_pwm` | `vehicle.Driver` |
| Démarrage | `input()` dans le terminal | Touche clavier dans la vue 3D |
| Config | `config.py` | `config_webots.py` |

### Convention d'orientation du lidar

Le 0° physique du lidar est monté pointant vers **l'arrière** de la voiture.

```
        AVANT (tableau[0])
           ↑
  gauche   │   droite
tableau[90]│tableau[-90]
           │
        ARRIÈRE (zone ignorée ±90°)
```

Les points entre **−90° et +90° physiques** (zone arrière + intérieur carrosserie) sont ignorés.

---

## Calibration

Tous les paramètres sont centralisés dans `config.py` (voiture réelle) et `config_webots.py` (simulation).

### Paramètres clés

```python
# Vitesses
VITESSE_CROISIERE_M_S  = 0.2    # m/s — augmenter progressivement
VITESSE_MAX_M_S_SOFT   = 0.3    # plafond logiciel de sécurité

# Seuils de détection
DISTANCE_MUR_AVANT_MM  = 400    # mm — obstacle frontal → recul
DISTANCE_MUR_COTE_MM   = 250    # mm — obstacle latéral → correction

# Suivi de couloir
K_SUIVI_COULOIR        = 0.02   # °/mm — augmenter si la voiture suit mal
```

### Recalibrer les moteurs

```bash
python base/test_pwm_propulsion.py   # calibration variateur
python base/test_pwm_direction.py    # calibration direction
```

Les nouvelles valeurs obtenues sont à reporter dans `config.py`.

---

## Règlement CoVAPSy — Points clés implémentés

| Règle | Implémentation |
|---|---|
| Pas de démarrage sans signal | La voiture attend `GO` dans le terminal |
| Marche arrière si blocage | `SEQUENCE_RECUL` dans la machine à états |
| Évitement obstacles (~400×200 mm) | Seuils `DISTANCE_MUR_AVANT_MM` / `DISTANCE_MUR_COTE_MM` |
| Fonctionnement double sens | Symétrie du correcteur de suivi de couloir |
