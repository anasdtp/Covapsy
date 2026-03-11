# conduite_reactive.py — Boucle de conduite autonome réactive (niveau FACILE)
# Architecture multi-thread reprise de conduite_autonome_avec_threads.py
import logging
import threading
import time

from robot_base import Actionneurs, CapteurLidar
import perception
import config

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------
# Drapeaux partagés entre threads (protégés par un Event threading)
# --------------------------------------------------------------------------
_run_event = threading.Event()


def boucle_conduite(actionneurs: Actionneurs, lidar: CapteurLidar):
    """Thread de conduite réactive.

    Lit les données lidar et applique la loi de commande :
      - Suivi de couloir par différence de distance à ±60°
      - Ralentissement progressif si obstacle proche devant
      - Séquence de recul si obstacle trop proche (DISTANCE_MUR_MM)

    Tourne à ~10 ms tant que _run_event est activé.
    """
    logger.info("Thread conduite démarré")
    actionneurs.demarrer()

    while _run_event.is_set():
        if not lidar.lire():
            time.sleep(config.BOUCLE_PERIODE_S)
            continue

        infos = perception.analyser(lidar.tableau_mm)

        # --- Obstacle bloquant ---
        if infos["mur_avant"]:
            logger.info("Obstacle devant (%.0f mm) — recul", infos["d_avant"])
            actionneurs.set_vitesse_m_s(0)
            angle_recul = perception.choisir_angle_recul(lidar.tableau_mm)
            actionneurs.set_direction_degre(angle_recul)
            actionneurs.recule()
            actionneurs.set_direction_degre(0)
            time.sleep(0.3)  # pause après recul avant de repartir

        elif infos["mur_avant_droit"]:
            logger.info("Obstacle avant-droit (%.0f mm) — correction gauche", infos["d_avant_droit"])
            actionneurs.set_direction_degre(+config.ANGLE_DEGRE_MAX)
            actionneurs.set_vitesse_m_s(infos["vitesse_consigne"])

        elif infos["mur_avant_gauche"]:
            logger.info("Obstacle avant-gauche (%.0f mm) — correction droite", infos["d_avant_gauche"])
            actionneurs.set_direction_degre(-config.ANGLE_DEGRE_MAX)
            actionneurs.set_vitesse_m_s(infos["vitesse_consigne"])

        else:
            # --- Couloir libre : suivi par différence ±60° ---
            actionneurs.set_direction_degre(infos["angle_couloir"])
            actionneurs.set_vitesse_m_s(infos["vitesse_consigne"])

        time.sleep(config.BOUCLE_PERIODE_S)

    # Arrêt propre à la sortie du thread
    actionneurs.set_vitesse_m_s(0)
    actionneurs.set_direction_degre(0)
    logger.info("Thread conduite arrêté")


def demarrer(actionneurs: Actionneurs, lidar: CapteurLidar) -> threading.Thread:
    """Lance le thread de conduite réactive.

    Retourne l'objet Thread pour pouvoir attendre sa fin avec join().
    """
    _run_event.set()
    t = threading.Thread(
        target=boucle_conduite,
        args=(actionneurs, lidar),
        daemon=True,
        name="conduite_reactive"
    )
    t.start()
    return t


def arreter():
    """Signale au thread de conduite de s'arrêter."""
    _run_event.clear()


# ============================================================
# Test standalone — nécessite le matériel Raspberry Pi
# ============================================================
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S"
    )

    lidar = CapteurLidar()
    act   = Actionneurs()

    try:
        lidar.connecter()
        lidar.demarrer()
        time.sleep(1)  # attendre le premier scan complet

        print("conduite_reactive.py — Ctrl+C pour arrêter")
        t_conduite = demarrer(act, lidar)
        t_conduite.join()

    except KeyboardInterrupt:
        print("\nArrêt demandé")
    finally:
        arreter()
        time.sleep(0.5)
        act.arreter()
        lidar.arreter()
        print("Arrêt propre")
