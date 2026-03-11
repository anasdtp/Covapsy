# main_autonomous.py — Programme principal de conduite autonome CoVAPSy
#
# Démarrage : python main_autonomous.py
# Commandes disponibles dans le terminal :
#   GO    → démarre la conduite autonome
#   STOP  → arrête la voiture
#   QUIT  → arrête la voiture et quitte le programme
#
# Conformité règlement CoVAPSy 2026 :
#   - La voiture n'avance pas avant réception de la commande GO
#   - La voiture s'arrête immédiatement sur commande STOP
#   - Marche arrière automatique en cas de blocage
import logging
import sys
import threading
import time

import conduite_reactive
from robot_base import Actionneurs, CapteurLidar

# ============================================================
# Configuration du logging
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("covapsy.log", encoding="utf-8"),
    ]
)
logger = logging.getLogger(__name__)


def afficher_aide():
    print("=" * 50)
    print("  CoVAPSy — Conduite Autonome")
    print("=" * 50)
    print("  GO    → Démarrer la voiture")
    print("  STOP  → Arrêter la voiture")
    print("  QUIT  → Quitter le programme")
    print("=" * 50)


def boucle_commandes(actionneurs: Actionneurs, lidar: CapteurLidar):
    """Thread principal de traitement des commandes utilisateur via input()."""
    t_conduite = None

    while True:
        try:
            cmd = input("\nCommande > ").strip().upper()
        except (EOFError, KeyboardInterrupt):
            cmd = "QUIT"

        if cmd == "GO":
            if conduite_reactive._run_event.is_set():
                print("  Déjà en marche.")
            else:
                logger.info("Commande GO reçue — démarrage de la conduite")
                print("  Démarrage de la conduite autonome...")
                t_conduite = conduite_reactive.demarrer(actionneurs, lidar)

        elif cmd == "STOP":
            if not conduite_reactive._run_event.is_set():
                print("  Déjà à l'arrêt.")
            else:
                logger.info("Commande STOP reçue — arrêt de la conduite")
                conduite_reactive.arreter()
                if t_conduite and t_conduite.is_alive():
                    t_conduite.join(timeout=2)
                actionneurs.set_vitesse_m_s(0)
                actionneurs.set_direction_degre(0)
                print("  Voiture arrêtée.")

        elif cmd == "QUIT":
            logger.info("Commande QUIT reçue — arrêt du programme")
            conduite_reactive.arreter()
            if t_conduite and t_conduite.is_alive():
                t_conduite.join(timeout=2)
            break

        else:
            print(f"  Commande inconnue : '{cmd}' — utilisez GO, STOP ou QUIT")


# ============================================================
# Point d'entrée
# ============================================================
def main():
    afficher_aide()

    lidar = CapteurLidar()
    act   = Actionneurs()

    try:
        # Initialisation du matériel
        logger.info("Connexion au lidar...")
        lidar.connecter()
        lidar.demarrer()

        logger.info("Initialisation des actionneurs (PWM désactivées jusqu'au GO)...")
        # Les PWM sont démarrées uniquement dans conduite_reactive.demarrer()
        # La voiture ne bouge donc pas avant la commande GO (règlement CoVAPSy)

        print("\n  Lidar connecté. En attente de la commande GO.")
        boucle_commandes(act, lidar)

    except Exception as e:
        logger.error("Erreur critique : %s", e, exc_info=True)

    finally:
        logger.info("Arrêt propre en cours...")
        conduite_reactive.arreter()
        time.sleep(0.5)
        try:
            act.arreter()
        except Exception:
            pass
        lidar.arreter()
        logger.info("Programme terminé.")
        print("Au revoir.")


if __name__ == "__main__":
    main()
