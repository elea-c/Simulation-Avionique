# Simulation-Avionique
Mini projet en C

HL-01 – Gestion d’engagement
Le logiciel doit permettre au pilote d’engager/désengager le système.
HL-02 – Validation capteurs
Le logiciel doit détecter une invalidité de capteurs et signaler la faute.
HL-03 – Gestion des modes
Le logiciel doit sélectionner un mode de fonctionnement (NORMAL/DEGRADED/FAILSAFE) selon l’état des capteurs et conditions de sécurité.
HL-04 – Protection overspeed
Le logiciel doit limiter l’action de commande lors d’une condition de survitesse.
HL-05 – Comportement en faute critique
Le logiciel doit passer en mode FAILSAFE quand une faute critique survient.
HL-06 – Traçabilité & testabilité
Chaque exigence bas niveau doit être vérifiable par test.


Pour HL-01 (Engagement)
LLR-01.1 Si pilot_engage = true et sensor_ok = true, alors le système doit passer de OFF à NORMAL au prochain cycle.
LLR-01.2 Si pilot_disengage = true, alors le système doit passer à OFF au prochain cycle, peu importe le mode courant.
LLR-01.3 Si pilot_engage = true alors que sensor_ok = false, le système doit rester OFF et lever fault_flags.SENSOR_INVALID.
Pour HL-02 (Validation capteurs)
LLR-02.1 Une mesure airspeed doit être considérée invalide si airspeed < 0 ou airspeed > 250 (valeurs arbitraires pour le mini-projet).
LLR-02.2 Une mesure altitude doit être considérée invalide si altitude < -100 ou altitude > 50000.
LLR-02.3 Si une mesure invalide est détectée, fault_flags.SENSOR_INVALID doit être mis à 1 au même cycle.
(Note : tu peux remplacer sensor_ok par une logique interne de validation basée sur les plages ci-dessus.)
Pour HL-03 (Modes)
LLR-03.1 Si le système est engagé et aucune faute capteur n’est active, le mode doit être NORMAL.
LLR-03.2 Si le système est engagé et fault_flags.SENSOR_INVALID = 1, alors le mode doit devenir DEGRADED au prochain cycle.
LLR-03.3 Si le système est en DEGRADED et que fault_flags.SENSOR_INVALID reste actif pendant N cycles consécutifs (ex: N=50 → 1s à 20ms), alors le mode doit devenir FAILSAFE.
Pour HL-04 (Overspeed)
LLR-04.1 Si airspeed >= 200, alors fault_flags.OVERSPEED doit être mis à 1 au même cycle.
LLR-04.2 Si fault_flags.OVERSPEED = 1, la sortie cmd doit être saturée à ±0.2 (au lieu de ±1.0) tant que la survitesse persiste.
LLR-04.3 Si airspeed < 195 (hystérésis), alors fault_flags.OVERSPEED doit revenir à 0.
Pour HL-05 (Failsafe)
LLR-05.1 En mode FAILSAFE, cmd doit être forcée à 0 à chaque cycle.
LLR-05.2 Le mode FAILSAFE ne peut être quitté que par pilot_disengage = true (retour à OFF).
Pour HL-06 (Traçabilité/tests)
LLR-06.1 Chaque LLR doit avoir au minimum un test unitaire qui vérifie le comportement nominal ou le cas limite correspondant.
LLR-06.2 Pour chaque HL, il doit exister une matrice de traçabilité listant les LLR associés.
