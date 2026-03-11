# robot_base_webots.py — Adaptateurs Webots pour les classes Actionneurs et CapteurLidar
# Implémente les mêmes interfaces que robot_base.py mais pour le simulateur Webots
import logging
import threading
import time

import config_webots as config

logger = logging.getLogger(__name__)


class Actionneurs:
    """Pilote la propulsion et la direction via l'API Webots Driver."""

    def __init__(self, driver):
        """
        Args:
            driver: instance de vehicle.Driver de Webots
        """
        self._driver = driver
        self._actif = False

    def demarrer(self):
        """Active les actionneurs (met la voiture à l'arrêt, roues droites)."""
        self.set_vitesse_m_s(0)
        self.set_direction_degre(0)
        self._actif = True
        logger.info("Actionneurs démarrés (Webots)")

    def arreter(self):
        """Remet à l'arrêt."""
        if self._actif:
            try:
                self.set_vitesse_m_s(0)
                self.set_direction_degre(0)
            except Exception as e:
                logger.warning("Erreur lors de l'arrêt des actionneurs : %s", e)
        self._actif = False
        logger.info("Actionneurs arrêtés (Webots)")

    # ----------------------------------------------------------
    # Commandes de base
    # ----------------------------------------------------------

    def set_vitesse_m_s(self, vitesse_m_s: float):
        """Commande la vitesse en m/s. Positif = avant, négatif = arrière."""
        # Saturation logicielle
        vitesse_m_s = max(-config.VITESSE_MAX_M_S_HARD,
                          min(config.VITESSE_MAX_M_S_SOFT, vitesse_m_s))
        
        # Conversion m/s → km/h pour Webots
        vitesse_kmh = vitesse_m_s * 3.6
        self._driver.setCruisingSpeed(vitesse_kmh)

    def set_direction_degre(self, angle_degre: float):
        """Commande l'angle de braquage en degrés. 0 = tout droit.
        +angle_degre_max = gauche, -angle_degre_max = droite.
        """
        # Saturation sur les butées physiques
        angle_degre = max(-config.ANGLE_DEGRE_MAX, min(config.ANGLE_DEGRE_MAX, angle_degre))
        
        # Inversion du signe pour Webots (convention différente)
        angle_rad = -angle_degre * 3.14159 / 180.0
        self._driver.setSteeringAngle(angle_rad)

    def recule(self):
        """Séquence de recul : impulsion courte arrière, pause, puis recul lent."""
        logger.info("Séquence de recul déclenchée (Webots)")
        # Recul simple pour Webots
        self.set_vitesse_m_s(config.VITESSE_RECUL_M_S)
        time.sleep(config.DUREE_RECUL_S)
        self.set_vitesse_m_s(0)


class CapteurLidar:
    """Acquisition lidar depuis Webots (pas de thread nécessaire, lecture synchrone)."""

    def __init__(self, lidar_webots):
        """
        Args:
            lidar_webots: instance de controller.Lidar de Webots
        """
        self._lidar = lidar_webots
        self.tableau_mm = [0] * 360   # tableau public lu par la logique de conduite
        self._nouveau_scan = False

    def connecter(self):
        """Active le lidar Webots."""
        # Le lidar est déjà activé dans controller_jaune.py via enable()
        logger.info("Lidar Webots connecté")

    def demarrer(self):
        """Démarre l'acquisition (pas de thread nécessaire pour Webots)."""
        logger.info("Lidar Webots démarré")

    def arreter(self):
        """Stoppe l'acquisition."""
        logger.info("Lidar Webots arrêté")

    def lire(self) -> bool:
        """Lit les données lidar depuis Webots et met à jour tableau_mm.

        Retourne True si de nouvelles données sont disponibles.
        """
        try:
            # Lecture des données brutes du lidar Webots
            donnees_brutes = self._lidar.getRangeImage()
            
            # Conversion au format attendu (identique à controller_jaune.py)
            for i in range(360):
                if (donnees_brutes[-i] > 0) and (donnees_brutes[-i] < 20):
                    self.tableau_mm[i-180] = 1000 * donnees_brutes[-i]
                else:
                    self.tableau_mm[i-180] = 0
            
            return True
        except Exception as e:
            logger.warning("Erreur lecture lidar Webots : %s", e)
            return False


# ============================================================
# Test standalone (ne fonctionne pas sans simulateur Webots)
# ============================================================
if __name__ == "__main__":
    print("Ce module nécessite le simulateur Webots pour fonctionner.")
    print("Utilisez controller_jaune.py comme point d'entrée.")
