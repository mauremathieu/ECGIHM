# STM32CubeIDE : Dispositif ECG-IHM salle blanche

## Description
Ce projet implémente un dispositif d'électrocardiogramme (ECG) avec interface homme-machine (IHM) avec les composants principales conceptionné en salle blanche. Le système utilise un microcontrôleur STM32 pour acquérir, filtrer et analyser un signal cardiaque, puis afficher les mesures de fréquence cardiaque (BPM) sur une matrice de LED.

## Fonctionnalités
- Acquisition du signal ECG via ADC
- Traitement du signal avec différents filtres (passe-bas FIR, Kalman)
- Détection des pics QRS et calcul du BPM (battements par minute)
- Affichage du BPM sur matrice à points
- Transmission des données via UART pour visualisation externe

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