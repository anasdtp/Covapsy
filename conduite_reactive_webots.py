# conduite_reactive_webots.py — Conduite autonome réactive adaptée pour Webots
# Version sans thread, appel synchrone depuis la boucle principale Webots
import logging

import config_webots as config

logger = logging.getLogger(__name__)


class ConduiteReactive:
    """Logique de conduite réactive (niveau FACILE) adaptée pour Webots.
    
    Cette classe implémente la même logique que conduite_reactive.py mais
    sans thread, car Webots utilise une boucle synchrone avec driver.step().
    """

    def __init__(self, actionneurs, lidar):
        """
        Args:
            actionneurs: instance de robot_base_webots.Actionneurs
            lidar: instance de robot_base_webots.CapteurLidar
        """
        self._act = actionneurs
        self._lidar = lidar
        self._actif = False
        self._en_recul = False  # Flag pour éviter les reculs en boucle
        self._cycles_depuis_recul = 0  # Compteur pour forcer une pause après recul

        # Import de la version Webots de perception
        import perception_webots
        import time
        self._perception = perception_webots
        self._time = time

    def demarrer(self):
        """Active la conduite autonome."""
        self._actif = True
        self._act.demarrer()
        logger.info("Conduite réactive démarrée (Webots)")

    def arreter(self):
        """Désactive la conduite autonome."""
        self._actif = False
        self._act.set_vitesse_m_s(0)
        self._act.set_direction_degre(0)
        logger.info("Conduite réactive arrêtée (Webots)")

    def step(self):
        """Exécute un cycle de la logique de conduite.
        
        À appeler à chaque itération de la boucle Webots (driver.step()).
        """
        if not self._actif:
            self._act.set_vitesse_m_s(0)
            self._act.set_direction_degre(0)
            return

        # Lecture des données lidar
        if not self._lidar.lire():
            logger.warning("Échec lecture lidar")
            return

        # Si on vient de reculer, on attend quelques cycles avant de réanalyser
        if self._cycles_depuis_recul > 0:
            self._cycles_depuis_recul -= 1
            if self._cycles_depuis_recul > 0:
                # Avance doucement pendant la pause
                self._act.set_vitesse_m_s(0.3)
                return

        # Analyse du tableau lidar
        infos = self._perception.analyser(self._lidar.tableau_mm)

        # --- Obstacle bloquant devant ---
        if infos["mur_avant"] and not self._en_recul:
            logger.info("Obstacle devant (%.0f mm) — recul", infos["d_avant"])
            self._en_recul = True
            self._act.set_vitesse_m_s(0)
            self._time.sleep(0.1)
            
            # Choisir l'angle de recul selon l'espace disponible
            angle_recul = self._perception.choisir_angle_recul(self._lidar.tableau_mm)
            self._act.set_direction_degre(angle_recul)
            self._act.recule()
            self._act.set_direction_degre(0)
            
            # Forcer une pause de 30 cycles (~0.3s à 100ms/cycle) après le recul
            self._cycles_depuis_recul = 30
            self._en_recul = False

        # --- Obstacle avant-droit ---
        elif infos["mur_avant_droit"]:
            logger.debug("Obstacle avant-droit (%.0f mm) — correction gauche", infos["d_avant_droit"])
            self._act.set_direction_degre(+config.ANGLE_DEGRE_MAX)
            self._act.set_vitesse_m_s(infos["vitesse_consigne"])

        # --- Obstacle avant-gauche ---
        elif infos["mur_avant_gauche"]:
            logger.debug("Obstacle avant-gauche (%.0f mm) — correction droite", infos["d_avant_gauche"])
            self._act.set_direction_degre(-config.ANGLE_DEGRE_MAX)
            self._act.set_vitesse_m_s(infos["vitesse_consigne"])

        # --- Couloir libre : suivi par différence ±60° ---
        else:
            self._act.set_direction_degre(infos["angle_couloir"])
            self._act.set_vitesse_m_s(infos["vitesse_consigne"])

    @property
    def actif(self) -> bool:
        """Retourne True si la conduite autonome est active."""
        return self._actif


# ============================================================
# Test standalone (ne fonctionne pas sans simulateur Webots)
# ============================================================
if __name__ == "__main__":
    print("Ce module nécessite le simulateur Webots pour fonctionner.")
    print("Utilisez controller_jaune.py comme point d'entrée.")
