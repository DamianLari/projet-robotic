import time
dateheure = time.time() - time.timezone
print(dateheure)
nbjour = int(dateheure/86400)
seconde = dateheure - 86400 * nbjour
heure = int(seconde / 3600)
seconde = seconde - 3600 * heure
minute = int(seconde / 60)
seconde = seconde - minute *60
milliseconde = (seconde - int(seconde)) * 1000
seconde = int(seconde)
nbannebis = int((nbjour + 365)/(365.25 * 4))
nbannee = int((nbjour - nbannebis) / 365)
nbjourannee = nbjour - nbannebis - 365 * nbannee
isbisextile = int((nbjour + 365 + 365)/(365.25 * 4)) - nbannebis
annee = 1970 + nbannee
nbjourmois = [0, 31, 59+isbisextile, 90+isbisextile, 120+isbisextile, 151+isbisextile, 181+isbisextile, 212+isbisextile, 243+isbisextile, 273+isbisextile, 304+isbisextile, 334+isbisextile, 365+isbisextile]
ismois = [nbjourannee - aaaa for aaaa in nbjourmois]
mois = [aaaa < 0 for aaaa in ismois].index(True) - 1
jour = nbjourannee - nbjourmois[mois] + 1
print("{:02}/{:02}/{:04} {:02}:{:02}:{:02}.{:03}".format(jour,mois,annee,heure,minute,seconde,int(milliseconde)))
