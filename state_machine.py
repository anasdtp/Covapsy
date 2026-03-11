# state_machine.py — Machine à états de conduite autonome CoVAPSy
import logging
import time
from enum import Enum, auto

import config
import perception

logger = logging.getLogger(__name__)


class Etat(Enum):
    """États de la machine à états de conduite autonome."""
    INIT            = auto()  # Initialisation, PWM à l'arrêt
    ATTENTE         = auto()  # Attente d'un nouveau scan lidar (10 ms)
    CALC_COMMANDE   = auto()  # Calcul de la direction et vitesse depuis le lidar
    EVAL_OBSTACLE   = auto()  # Évaluation de la nature de l'obstacle détecté
    SEQUENCE_RECUL  = auto()  # Exécution de la manœuvre de recul
    ARRET           = auto()  # Arrêt d'urgence ou arrêt demandé


class MachinaEtats:
    """Implémente la machine à états de conduite autonome.

    Diagramme de transitions :
        INIT → ATTENTE
        ATTENTE --[nouveau scan, pas d'obstacle]--→ CALC_COMMANDE → ATTENTE
        ATTENTE --[nouveau scan, obstacle]--------→ EVAL_OBSTACLE
        EVAL_OBSTACLE --[mur détecté]-------------→ SEQUENCE_RECUL → ATTENTE
        EVAL_OBSTACLE --[faux positif / passage]--→ CALC_COMMANDE → ATTENTE
        ARRET ← (signal externe ou timeout blocage)

    Args :
        actionneurs : instance de robot_base.Actionneurs
        lidar       : instance de robot_base.CapteurLidar
    """

    def __init__(self, actionneurs, lidar):
        self._act   = actionneurs
        self._lidar = lidar
        self._etat  = Etat.INIT

        # Chronomètre pour détecter un blocage prolongé
        self._t_dernier_mouvement = time.monotonic()
        # Compteur consécutif d'obstacles pour éviter les faux positifs
        self._compteur_obstacle   = 0
        # Seuil : N scans consécutifs avec obstacle avant de déclencher le recul
        self._seuil_obstacle      = 3

    # ----------------------------------------------------------
    # Propriété publique
    # ----------------------------------------------------------

    @property
    def etat(self) -> Etat:
        return self._etat

    # ----------------------------------------------------------
    # Boucle principale
    # ----------------------------------------------------------

    def step(self) -> bool:
        """Exécute un cycle de la machine à états.

        Retourne False si l'état ARRET est atteint (signal d'arrêt).
        À appeler en boucle toutes les ~10 ms.
        """
        if self._etat == Etat.INIT:
            self._transition(Etat.ATTENTE)

        elif self._etat == Etat.ATTENTE:
            if self._lidar.lire():
                infos = perception.analyser(self._lidar.tableau_mm)
                if infos["obstacle_detecte"]:
                    self._compteur_obstacle += 1
                    if self._compteur_obstacle >= self._seuil_obstacle:
                        self._transition(Etat.EVAL_OBSTACLE, infos=infos)
                    # Obstacle isolé (< seuil) : on continue mais on ralentit
                    else:
                        self._act.set_vitesse_m_s(infos["vitesse_consigne"])
                else:
                    self._compteur_obstacle = 0
                    self._transition(Etat.CALC_COMMANDE, infos=infos)
            else:
                time.sleep(config.BOUCLE_PERIODE_S)

        elif self._etat == Etat.CALC_COMMANDE:
            # Les infos ont déjà été calculées lors du passage en ATTENTE
            # On les récupère depuis le tableau courant
            infos = perception.analyser(self._lidar.tableau_mm)
            self._act.set_direction_degre(infos["angle_couloir"])
            self._act.set_vitesse_m_s(infos["vitesse_consigne"])
            if infos["vitesse_consigne"] > 0:
                self._t_dernier_mouvement = time.monotonic()
            # Vérification du timeout de blocage
            if time.monotonic() - self._t_dernier_mouvement > config.TIMEOUT_BLOCAGE_S:
                logger.warning("Timeout blocage détecté — déclenchement recul d'urgence")
                self._transition(Etat.SEQUENCE_RECUL)
            else:
                self._transition(Etat.ATTENTE)

        elif self._etat == Etat.EVAL_OBSTACLE:
            infos = perception.analyser(self._lidar.tableau_mm)
            # Si l'obstacle est devant ou sur le côté avant → recul
            if infos["mur_avant"] or infos["mur_avant_gauche"] or infos["mur_avant_droit"]:
                self._transition(Etat.SEQUENCE_RECUL)
            else:
                # L'obstacle n'est plus critique (passage latéral possible)
                self._compteur_obstacle = 0
                self._transition(Etat.CALC_COMMANDE)

        elif self._etat == Etat.SEQUENCE_RECUL:
            # Choisir le sens de braquage selon l'espace disponible
            angle_recul = perception.choisir_angle_recul(self._lidar.tableau_mm)
            logger.info("Recul — braquage %.0f°", angle_recul)
            self._act.set_direction_degre(angle_recul)
            self._act.recule()
            self._act.set_direction_degre(0)
            self._compteur_obstacle   = 0
            self._t_dernier_mouvement = time.monotonic()
            self._transition(Etat.ATTENTE)

        elif self._etat == Etat.ARRET:
            self._act.set_vitesse_m_s(0)
            self._act.set_direction_degre(0)
            return False

        return True

    def arreter(self):
        """Force la transition vers l'état ARRET."""
        self._transition(Etat.ARRET)

    # ----------------------------------------------------------
    # Helpers internes
    # ----------------------------------------------------------

    def _transition(self, nouvel_etat: Etat, infos: dict = None):
        if nouvel_etat != self._etat:
            logger.debug("Transition %s → %s", self._etat.name, nouvel_etat.name)
        self._etat = nouvel_etat


# ============================================================
# Test standalone (mock lidar sans matériel)
# ============================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s %(name)s: %(message)s")

    class MockActionneurs:
        def set_vitesse_m_s(self, v):   print(f"  → vitesse {v:.2f} m/s")
        def set_direction_degre(self, a): print(f"  → direction {a:.1f}°")
        def recule(self):               print("  → RECUL")

    class MockLidar:
        def __init__(self):
            self.tableau_mm = [800] * 360
            self._appels    = 0
        def lire(self):
            self._appels += 1
            # Simule un obstacle après 5 appels
            if self._appels == 5:
                self.tableau_mm[0] = 150  # obstacle droit devant
            return True

    sm = MachinaEtats(MockActionneurs(), MockLidar())

    print("=== Test machine à états (10 cycles) ===")
    for i in range(10):
        print(f"\nCycle {i+1} — état courant : {sm.etat.name}")
        continuer = sm.step()
        if not continuer:
            print("→ ARRET")
            break
