# config.py — Paramètres de configuration centralisés CoVAPSy
# Valeurs calibrées depuis conduite_autonome_basique.py (validé sur la voiture réelle)
# Pour recalibrer : utiliser test_pwm_propulsion.py et test_pwm_direction.py

# ============================================================
# LIDAR
# ============================================================
LIDAR_PORT     = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 256000

# ============================================================
# PROPULSION — HardwarePWM channel 0, 50 Hz
# ============================================================
DIRECTION_PROP      = 1     # 1 = variateur normal, -1 = variateur inversé
PWM_STOP_PROP       = 7.36  # duty cycle correspondant à l'arrêt (1.5 ms)
POINT_MORT_PROP     = 0.41  # seuil minimal en dessous duquel la voiture ne bouge pas
DELTA_PWM_MAX_PROP  = 1.0   # plage PWM entre l'arrêt et la vitesse maximale
VITESSE_MAX_M_S_HARD = 8.0  # vitesse physique maximale de la voiture (m/s)
VITESSE_MAX_M_S_SOFT = 0.3  # vitesse logicielle maximale en autonome (m/s) — MODE TEST

# ============================================================
# DIRECTION — HardwarePWM channel 1, 50 Hz
# ============================================================
DIRECTION_DIR    = -1   # -1 = angle_pwm_min à droite, +1 = angle_pwm_min à gauche
ANGLE_PWM_MIN    = 5.5  # butée physique droite (duty cycle)
ANGLE_PWM_MAX    = 9.3  # butée physique gauche (duty cycle)
ANGLE_PWM_CENTRE = 7.4  # centre (roues droites)
ANGLE_DEGRE_MAX  = 18   # angle max en degrés (vers la gauche)

# ============================================================
# CONDUITE AUTONOME
# ============================================================
BOUCLE_PERIODE_S       = 0.01   # période de la boucle de contrôle (10 ms)
DISTANCE_MUR_AVANT_MM  = 400    # seuil obstacle FRONTAL (mm) — uniquement devant
DISTANCE_MUR_COTE_MM   = 250    # seuil obstacle LATÉRAL avant-gauche/avant-droit (mm)
DISTANCE_STOP_MM       = 200    # distance d'arrêt d'urgence (mm)
VITESSE_CROISIERE_M_S  = 0.2    # vitesse de croisière en couloir libre (m/s) — MODE TEST
K_SUIVI_COULOIR        = 0.02   # gain du correcteur de suivi de couloir (°/mm)
TIMEOUT_BLOCAGE_S      = 2.0    # délai avant déclenchement séquence de recul (s)

# ============================================================
# SÉQUENCE DE RECUL
# ============================================================
VITESSE_RECUL_M_S  = -0.5   # vitesse lors du recul — MODE TEST
DUREE_RECUL_S      = 0.4    # durée du recul
