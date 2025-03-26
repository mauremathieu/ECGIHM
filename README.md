# STM32CubeIDE : Dispositif ECG-IHM salle blanche

## Description
Ce projet implémente un dispositif d'électrocardiogramme (ECG) avec interface homme-machine (IHM) avec les composants principales conceptionné en salle blanche. Le système utilise un microcontrôleur STM32 pour acquérir, filtrer et analyser un signal cardiaque, puis afficher les mesures de fréquence cardiaque (BPM) sur une matrice de LED.

## Fonctionnalités
- **Acquisition du signal ECG** :
  - Échantillonnage à 1500 Hz via ADC
  - Moyenne glissante sur K échantillons pour réduire le bruit
  - Saturation des valeurs extrêmes pour éviter les artefacts

- **Traitement du signal** :
  - **Filtrage passe-bas FIR** : Filtre à réponse impulsionnelle finie d'ordre 64 avec fréquence de coupure à 45 Hz et fenêtre de Hamming pour éliminer les hautes fréquences parasites
  - **Filtrage de Kalman** : Réduction du bruit et extraction des caractéristiques importantes du signal avec détection des variations significatives
  - **Élévation au carré** : Amplification des pics QRS pour faciliter leur détection

- **Algorithme de détection des battements cardiaques** :
  - Détection des pics QRS par seuillage adaptatif
  - Mécanisme anti-rebond (200ms) pour éviter les fausses détections
  - Calcul du BPM basé sur la moyenne des N derniers intervalles entre pics

- **Interface utilisateur** :
  - Affichage en temps réel du BPM sur matrice LED DOT_MATRIX
  - Indication visuelle par LED clignotante pendant l'acquisition
  - Format d'affichage optimisé pour la lisibilité

- **Communication et débogage** :
  - Transmission série UART des données brutes et filtrées
  - Format de sortie compatible avec l'outil Serial Plotter pour visualisation graphique
  - Envoi simultané des signaux à différentes étapes du traitement (signal brut, filtré, BPM)

- **Optimisations** :
  - Gestion efficace de la mémoire avec buffers circulaires
  - Calibration automatique de l'ADC pour une précision optimale
  - Utilisation de timers matériels pour garantir la précision temporelle

## Technologies utilisées
- STM32L476RG (Nucleo)
- STM32CubeIDE
- Capteurs ECG
- Affichage matriciel LED
- Communication UART

## Configuration matérielle requise
- Carte Nucleo STM32L476RG
- Capteur ECG compatible
- Matrice LED DOT_MATRIX
- Interface série pour visualisation des données

## Paramètres du système
- Fréquence d'échantillonnage: 1500 Hz
- Filtre passe-bas: 45 Hz
- Détection de pics avec seuil adaptatif

## Authors

- MAURE Mathieu
- BONNET Pierre-François