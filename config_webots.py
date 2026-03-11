# config_webots.py — Paramètres de configuration pour le simulateur Webots
# Adapté depuis config.py pour fonctionner avec Webots au lieu du hardware Raspberry Pi

# ============================================================
# DIRECTION — Paramètres Webots
# ============================================================
ANGLE_DEGRE_MAX  = 16   # angle max en degrés (vers la gauche) — identique au simulateur

# ============================================================
# VITESSE — Paramètres Webots
# ============================================================
VITESSE_MAX_M_S_HARD = 8.0   # vitesse physique maximale (m/s)
VITESSE_MAX_M_S_SOFT = 0.5   # vitesse logicielle maximale en autonome (m/s) — augmenté pour simulation
VITESSE_CROISIERE_M_S  = 0.5  # vitesse de croisière en couloir libre (m/s)
VITESSE_RECUL_M_S = -4.0     # vitesse lors du recul (m/s)

# ============================================================
# CONDUITE AUTONOME
# ============================================================
BOUCLE_PERIODE_S       = 0.01   # période de la boucle de contrôle (10 ms)
DISTANCE_MUR_AVANT_MM  = 500    # seuil obstacle FRONTAL (mm) — réduit pour éviter reculs prématurés
DISTANCE_MUR_COTE_MM   = 350    # seuil obstacle LATÉRAL avant-gauche/avant-droit (mm)
DISTANCE_STOP_MM       = 200    # distance d'arrêt d'urgence (mm)
K_SUIVI_COULOIR        = 0.02   # gain du correcteur de suivi de couloir (°/mm)
TIMEOUT_BLOCAGE_S      = 2.0    # délai avant déclenchement séquence de recul (s)

# ============================================================
# SÉQUENCE DE RECUL
# ============================================================
DUREE_RECUL_S      = 0.6    # durée du recul (s)
PAUSE_APRES_RECUL_S = 0.3   # pause après recul avant de repartir (s)
