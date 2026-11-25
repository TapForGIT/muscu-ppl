// =====================================================================
//  Fiches exercices (texte rédigé pour l'appli, à partir des consignes
//  techniques de référence — sources indiquées dans "src").
//  Modifiable : ajoute une entrée avec le nom EXACT de l'exercice.
// =====================================================================
window.FICHES = {
"Développé couché Smith":{
  exec:["Allonge-toi sur le banc plat, la barre au-dessus du bas de la poitrine.","Prends la barre un peu plus large que les épaules, décroche-la en la faisant pivoter.","Descends-la en contrôle jusqu'à effleurer le bas des pecs.","Pousse jusqu'à bras presque tendus, sans verrouiller les coudes."],
  cles:["Omoplates serrées et basses, pieds bien à plat.","Coudes à environ 45-60° du buste en bas (pas à 90°).","Poignets droits, alignés avec les avant-bras."],
  err:["Banc mal placé : la barre tombe sur le cou ou le ventre.","Rebondir sur la poitrine.","Décoller les fesses du banc."],
  src:"SmartWorkout — Développé couché à la Smith machine"},
"Développé incliné Smith":{
  exec:["Règle le banc entre 30 et 45°, placé pour que la barre tombe sur le haut de la poitrine.","Prise un peu plus large que les épaules, décroche la barre en la faisant pivoter.","Descends lentement jusqu'au haut des pecs, sous les clavicules.","Marque un temps sans rebond, puis pousse jusqu'en haut sans verrouiller."],
  cles:["Coudes à environ 45° du buste.","Trajectoire alignée avec le haut de la poitrine.","Poignets droits au-dessus des coudes."],
  err:["Barre qui descend vers le cou.","Banc trop incliné (l'exercice devient un développé épaules).","Écarter les coudes à 90°."],
  src:"SmartWorkout — Développé incliné à la machine Smith"},
"Développé incliné haltères":{
  exec:["Dossier entre 30 et 45°, pieds à plat, genoux à environ 90°.","Haltères de chaque côté du haut de la poitrine, avant-bras verticaux.","Pousse vers le haut et légèrement vers l'intérieur jusqu'à bras tendus au-dessus des épaules.","Redescends en 2 secondes sur la même trajectoire jusqu'à sentir l'étirement sous les clavicules."],
  cles:["Fessiers, haut du dos et tête restent sur le banc.","En bas, coudes juste sous la ligne du banc, à 45-60° du buste.","Les haltères se rapprochent en haut sans se cogner."],
  err:["Descendre les coudes trop bas (épaules en souffrance).","Cambrer pour soulever plus lourd.","Laisser tomber les haltères à la fin : ramène-les sur les cuisses."],
  src:"SmartWorkout — Développé incliné avec haltères"},
"Écartés poulie haute vers basse":{
  exec:["Poulies réglées en haut, une poignée dans chaque main.","Avance d'un pas pour mettre les câbles en tension, un pied devant l'autre si besoin.","Bras ouverts sur les côtés, coudes légèrement fléchis.","Ramène les mains devant toi, vers le bas, en grand arc de cercle, puis reviens lentement."],
  cles:["Garde la même légère flexion des coudes tout le long.","Buste stable, gainé, à peine penché.","Serre les pecs une seconde quand les mains se rejoignent."],
  err:["Plier les coudes pendant le mouvement (ça devient un développé).","Se pencher trop en avant.","Laisser les poignées tirer les bras trop loin en arrière au retour."],
  src:"SmartWorkout — Écartés à la poulie"},
"Écartés poulie basse vers haute":{
  exec:["Poulies réglées en bas, une poignée dans chaque main.","Debout au milieu, un léger pas en avant, bras tendus vers le bas sur les côtés.","Monte les mains vers l'avant et vers le haut en arc de cercle jusqu'à hauteur des épaules.","Serre le haut des pecs, puis redescends lentement."],
  cles:["Coudes légèrement fléchis et fixes.","Les mains finissent devant, à hauteur d'épaules, sans dépasser.","Buste droit, épaules basses."],
  err:["Monter trop haut (les épaules prennent le relais).","Tirer avec les bras en pliant les coudes.","Se cambrer."],
  src:"SmartWorkout — Écarté à un bras de bas en haut avec câble"},
"Élévations latérales poulie":{
  exec:["Poulie en bas, poignée dans la main la plus éloignée de la machine.","Le câble passe devant toi, la main part devant la hanche opposée.","Monte le bras sur le côté jusqu'à hauteur d'épaule, coude légèrement fléchi.","Redescends lentement sans laisser la charge se poser."],
  cles:["Monte légèrement en avant du corps (plan de l'omoplate).","Épaule basse : ne hausse pas les épaules.","Tempo lent, surtout à la descente."],
  err:["Lancer le bras avec le buste.","Monter au-dessus de l'horizontale.","Plier fortement le coude."],
  src:"SmartWorkout — Élévation latérale à un bras avec câble"},
"Élévations latérales haltères":{
  exec:["Debout, un haltère dans chaque main le long du corps.","Coudes légèrement fléchis, monte les bras sur les côtés.","Arrête-toi à hauteur d'épaules.","Redescends lentement."],
  cles:["Bras un peu en avant du corps.","Le coude mène le mouvement, la main ne passe pas au-dessus du coude.","Épaules basses, buste immobile."],
  err:["Se balancer pour lancer les haltères.","Hausser les épaules.","Monter trop haut."],
  src:"SmartWorkout — Élévation latérale avec haltères"},
"Extension triceps overhead poulie":{
  exec:["Corde à la poulie haute, dos à la machine.","Grand pas en avant (fente), buste penché vers l'avant, dos plat.","Corde derrière la tête, bras le long des oreilles, coudes fléchis vers l'avant.","Tends les bras vers l'avant jusqu'à les aligner avec le buste, puis reviens lentement."],
  cles:["Seuls les avant-bras bougent, les coudes restent près de la tête.","Poings serrés ensemble sur la corde.","Bonne amplitude derrière la tête pour étirer la longue portion."],
  err:["Écarter les coudes.","Bouger les épaules pour aider.","Se redresser pendant la série."],
  src:"SmartWorkout — Extension triceps au-dessus de la tête à la poulie haute"},
"Pushdown corde":{
  exec:["Corde à la poulie haute, face à la machine.","Pieds largeur d'épaules, genoux souples, buste légèrement penché.","Coudes collés au corps, fléchis à environ 90°.","Pousse vers le bas jusqu'à bras tendus en écartant un peu la corde, puis remonte lentement."],
  cles:["Coudes immobiles contre les côtes.","Poignets neutres.","Écarte les extrémités de la corde en bas."],
  err:["Laisser les coudes avancer ou reculer.","Se pencher pour pousser avec le poids du corps.","Remonter trop haut (les coudes décollent)."],
  src:"SmartWorkout — Extension des triceps à la corde"},
"Pushdown unilatéral prise supination":{
  exec:["Poignée simple à la poulie haute, prise paume vers le haut.","Debout, droit, coude collé au corps.","Pars avant-bras parallèle au sol.","Tends le bras vers le bas jusqu'à extension complète, puis remonte lentement."],
  cles:["Seul l'avant-bras bouge.","Marque une pause bras tendu.","Termine un bras puis change."],
  err:["Coude qui s'écarte du corps.","Se pencher sur le côté.","Poignet cassé."],
  src:"SmartWorkout — Extension triceps unilatérale en prise inversée à la poulie"},
"Développé épaules machine":{
  exec:["Règle le siège pour avoir les poignées à hauteur d'épaules.","Dos plaqué au dossier, pieds à plat.","Prise paumes vers l'avant, coudes légèrement devant le corps.","Pousse jusqu'à bras tendus sans verrouiller, puis redescends lentement."],
  cles:["Épaules basses, pas haussées.","Poitrine sortie, colonne neutre.","Descente contrôlée jusqu'à hauteur d'épaules."],
  err:["Décoller le dos du dossier en cambrant.","Descendre trop bas.","Verrouiller les coudes en haut."],
  src:"SmartWorkout — Presse à épaules à la machine"},
"Rowing T poitrine appuyée prise large":{
  exec:["Pieds sur le repose-pieds, genoux à environ 90°.","Poitrine sur le support, le haut du support sous les clavicules, épaules qui dépassent.","Poignées larges, prise paumes vers le bas, bras tendus.","Serre les omoplates puis tire jusqu'au bord du support, coudes à hauteur du dos, et redescends en 2-3 secondes."],
  cles:["Coudes écartés à 60-80° du buste.","La poitrine reste collée au support.","Ne pose pas les disques entre les répétitions."],
  err:["Décoller la poitrine pour tricher.","Tirer avec les biceps sans serrer les omoplates.","Hausser les épaules."],
  src:"SmartWorkout — Rowing T-bar avec appui poitrine"},
"Squat barre":{
  exec:["Barre sur le haut du dos, pieds largeur d'épaules, pointes légèrement ouvertes.","Inspire, gaine le ventre.","Descends en envoyant la hanche en arrière et les genoux vers l'avant, jusqu'à la parallèle.","Remonte en poussant dans tout le pied."],
  cles:["La barre reste au-dessus du milieu du pied.","Les genoux suivent la direction des pointes de pieds.","Dos neutre, regard droit devant."],
  err:["Genoux qui rentrent vers l'intérieur.","Talons qui décollent.","Dos qui s'arrondit en bas."],
  src:"SmartWorkout — Squat avec barre"},
"Tirage vertical prise neutre serrée":{
  exec:["Cuisses bloquées sous le boudin, poignée en V (prise neutre serrée).","Bras tendus, buste légèrement penché en arrière.","Tire la poignée vers le haut de la poitrine en descendant les coudes le long du corps.","Remonte lentement jusqu'à bras tendus."],
  cles:["Pense à ramener les coudes vers les hanches.","Poitrine sortie, omoplates basses.","La poignée passe devant le visage."],
  err:["Se balancer en arrière.","Tirer derrière la nuque.","Arrondir le dos en bas du mouvement."],
  src:"SmartWorkout — Tirage vertical (tirage poitrine)"}
};
