# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Architecture modulaire adaptée du projet CoVAPSy
# Utilise les modules: robot_base_webots, conduite_reactive_webots, perception_webots
#
# Commandes clavier:
#   A → Activer le mode autonome
#   N → Désactiver le mode autonome (arrêt)

from vehicle import Driver
from controller import Lidar
import logging
import sys

# Import des modules CoVAPSy adaptés pour Webots
from robot_base_webots import Actionneurs, CapteurLidar
from conduite_reactive_webots import ConduiteReactive

# ============================================================
# Configuration du logging
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger(__name__)

# ============================================================
# Initialisation Webots
# ============================================================
driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Lidar Webots
lidar_webots = Lidar("RpLidarA2")
lidar_webots.enable(sensorTimeStep)
lidar_webots.enablePointCloud() 

# Clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# Mise à zéro de la vitesse et de la direction
driver.setSteeringAngle(0)
driver.setCruisingSpeed(0)

# ============================================================
# Initialisation des modules CoVAPSy
# ============================================================
actionneurs = Actionneurs(driver)
capteur_lidar = CapteurLidar(lidar_webots)
conduite = ConduiteReactive(actionneurs, capteur_lidar)

# Connexion et démarrage des modules
capteur_lidar.connecter()
capteur_lidar.demarrer()

logger.info("=== CoVAPSy - Simulateur Webots ===")
logger.info("Appuyez sur 'A' pour activer le mode autonome")
logger.info("Appuyez sur 'N' pour désactiver le mode autonome")

print("=" * 50)
print("  CoVAPSy — Conduite Autonome (Webots)")
print("=" * 50)
print("  Cliquez sur la vue 3D pour commencer")
print("  A → Activer le mode autonome")
print("  N → Désactiver le mode autonome")
print("=" * 50)

# ============================================================
# Boucle principale Webots
# ============================================================
while driver.step() != -1:
    # Récupération des touches clavier
    while True:
        currentKey = keyboard.getKey()
        
        if currentKey == -1:
            break
        
        elif currentKey == ord('n') or currentKey == ord('N'):
            if conduite.actif:
                logger.info("Commande STOP reçue — arrêt de la conduite")
                conduite.arreter()
                print("-------- Mode Auto TT-02 jaune Désactivé -------")
        
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not conduite.actif:
                logger.info("Commande GO reçue — démarrage de la conduite")
                conduite.demarrer()
                print("-------- Mode Auto TT-02 jaune Activé ---------")
    
    # Exécution d'un cycle de la conduite autonome
    # (gère automatiquement le cas où le mode est désactivé)
    conduite.step()

# ============================================================
# Arrêt propre
# ============================================================
logger.info("Arrêt de la simulation")
conduite.arreter()
actionneurs.arreter()
capteur_lidar.arreter()

