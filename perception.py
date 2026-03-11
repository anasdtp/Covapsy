# perception.py — Analyse du tableau lidar et détection d'obstacles
import config


def distance_secteur(tableau_mm: list, angle_centre: int, demi_angle: int = 15) -> float:
    """Retourne la distance minimale non nulle dans un secteur angulaire.

    angle_centre : angle en degrés, 0 = devant, +gauche, -droite.
    Retourne 0 si aucune mesure valide dans le secteur.
    """
    n = len(tableau_mm)
    valeurs = []
    for da in range(-demi_angle, demi_angle + 1):
        idx = angle_centre + da
        # Accès circulaire avec indices négatifs nativement supportés par Python
        val = tableau_mm[idx % n]
        if val > 0:
            valeurs.append(val)
    return min(valeurs) if valeurs else 0


def filtrer_tableau(tableau_mm: list, fenetre: int = 3) -> list:
    """Filtre médian glissant sur le tableau lidar.

    Les mesures à 0 (pas de retour lidar) sont ignorées lors du filtre
    mais conservées si aucun voisin valide n'est disponible.
    """
    n = len(tableau_mm)
    resultat = list(tableau_mm)
    demi = fenetre // 2
    for i in range(n):
        voisins = []
        for k in range(-demi, demi + 1):
            v = tableau_mm[(i + k) % n]
            if v > 0:
                voisins.append(v)
        if voisins:
            resultat[i] = sorted(voisins)[len(voisins) // 2]
    return resultat


def analyser(tableau_mm: list) -> dict:
    """Analyse complète du tableau lidar.

    Retourne un dictionnaire avec :
    - d_avant, d_avant_gauche, d_avant_droit, d_gauche, d_droit : distances (mm)
    - mur_avant, mur_gauche, mur_droit                          : bool obstacle proche
    - obstacle_detecte                                           : bool (union des murs)
    - angle_couloir                                              : angle de correction (°)
    - espace_gauche, espace_droit                               : espace libre estimé (mm)

    Convention angles : 0° = devant, +° = gauche, -° = droite.
    """
    # --- Distances par secteur ---
    d_avant         = distance_secteur(tableau_mm,   0, demi_angle=20)
    d_avant_gauche  = distance_secteur(tableau_mm,  45, demi_angle=15)
    d_avant_droit   = distance_secteur(tableau_mm, -45, demi_angle=15)
    d_gauche        = distance_secteur(tableau_mm,  90, demi_angle=20)
    d_droit         = distance_secteur(tableau_mm, -90, demi_angle=20)

    # --- Détection de blocage (obstacle dans la zone de danger) ---
    mur_avant       = 0 < d_avant        < config.DISTANCE_MUR_MM
    mur_avant_gauche = 0 < d_avant_gauche < config.DISTANCE_MUR_MM
    mur_avant_droit  = 0 < d_avant_droit  < config.DISTANCE_MUR_MM

    # --- Calcul de l'angle de suivi de couloir ---
    # Différence entre les distances à ±60° : si symétrique → angle=0
    # La saturation évite des commandes excessives
    raw_angle = config.K_SUIVI_COULOIR * (tableau_mm[60] - tableau_mm[-60])
    angle_couloir = max(-config.ANGLE_DEGRE_MAX, min(config.ANGLE_DEGRE_MAX, raw_angle))

    # --- Vitesse adaptative selon distance avant ---
    # Vitesse entre 0 et VITESSE_CROISIERE selon la distance disponible devant
    if d_avant <= 0:
        vitesse_consigne = config.VITESSE_CROISIERE_M_S
    elif d_avant < config.DISTANCE_STOP_MM:
        vitesse_consigne = 0.0
    elif d_avant < config.DISTANCE_MUR_MM:
        # Proportionnel : ralentissement progressif
        ratio = (d_avant - config.DISTANCE_STOP_MM) / (config.DISTANCE_MUR_MM - config.DISTANCE_STOP_MM)
        vitesse_consigne = ratio * config.VITESSE_CROISIERE_M_S
    else:
        vitesse_consigne = config.VITESSE_CROISIERE_M_S

    return {
        "d_avant":          d_avant,
        "d_avant_gauche":   d_avant_gauche,
        "d_avant_droit":    d_avant_droit,
        "d_gauche":         d_gauche,
        "d_droit":          d_droit,
        "mur_avant":        mur_avant,
        "mur_avant_gauche": mur_avant_gauche,
        "mur_avant_droit":  mur_avant_droit,
        "obstacle_detecte": mur_avant or mur_avant_gauche or mur_avant_droit,
        "angle_couloir":    angle_couloir,
        "vitesse_consigne": vitesse_consigne,
    }


def choisir_angle_recul(tableau_mm: list) -> float:
    """Détermine l'angle de braquage optimal pour la manœuvre de recul.

    Tourne dans le sens qui offre le plus d'espace latéral.
    """
    d_gauche = distance_secteur(tableau_mm,  60, demi_angle=20)
    d_droit  = distance_secteur(tableau_mm, -60, demi_angle=20)

    if d_gauche >= d_droit:
        # Plus d'espace à gauche → braquer à droite pour reculer
        return -config.ANGLE_DEGRE_MAX
    else:
        return +config.ANGLE_DEGRE_MAX


# ============================================================
# Test standalone
# ============================================================
if __name__ == "__main__":
    # Simule un tableau lidar avec un obstacle à 150mm devant
    tab = [1000] * 360
    tab[0]   = 150   # obstacle droit devant
    tab[60]  = 800   # plus loin à gauche
    tab[-60] = 400   # plus proche à droite

    infos = analyser(tab)
    print("=== Test perception.py ===")
    for cle, val in infos.items():
        print(f"  {cle:20s} : {val}")
    print(f"\n  Angle recul conseillé : {choisir_angle_recul(tab)}°")
