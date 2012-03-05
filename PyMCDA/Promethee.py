#!/usr/bin/env python
from math import exp

class PreferenceType2:
        q = 0
        def __init__(self, valQ):
                self.q = valQ
        def valeur(self, diff):
                if (diff <= self.q):
                        return 0
                return 1

class PreferenceType5:
        q = 0
        p = 1
        def __init__(self, valQ, valP):
                self.q = valQ
                self.p = valP
        def valeur(self, diff):
                if (diff <= self.q):
                        return 0
                if (diff <= self.p):
                        return (diff - self.q) / (self.p - self.q)
                return 1

class PreferenceType6:
        s = 0.5
        valSquare = 0.5
        def __init__(self, valS):
                self.s = valS
                self.valSquare = -1 * (2 * valS * valS)
        def valeur(self, diff):
                if (diff <= 0):
                        return 0
                return 1 - exp(diff * diff / self.valSquare)

        
def decision(candidats, criteres, poidsCrit, fctPrefCrit):
        if (len(candidats) == 1):
                return candidats[0]
        first = True
        outRankingMax = 0
        for cand1 in candidats:
                outRankingPlus = 0
                outRankingMoins = 0
                for cand2 in candidats:
                        if cand1 == cand2:
                                continue
                        PiXA = 0
                        PiAX = 0
                        for crit in criteres:
                                poids = poidsCrit[crit]
                                fctPref = fctPrefCrit[crit]
                                valCand1 = cand1[crit]
                                valCand2 = cand2[crit]
                                val1 = valCand1 - valCand2
                                val2 = valCand2 - valCand1
                                PiXA = PiXA + poids * fctPref.valeur(val1)
                                PiAX = PiAX + poids * fctPref.valeur(val2)
                        outRankingPlus = outRankingPlus + PiXA
                        outRankingMoins = outRankingMoins + PiAX
                outRankingPlus = outRankingPlus / (len(candidats) - 1)
                outRankingMoins = outRankingMoins / (len(candidats) - 1)
                outRanking = outRankingPlus - outRankingMoins
                if ( first or outRanking > outRankingMax ):
                        outRankingMax = outRanking
                        meilleurCand = cand1
                        first = False
        return meilleurCand

def filtragePareto (candidats, criteres):
        candidatsFilt = []
        for cand1 in candidats:
                paretoFront = True
                for cand2 in candidats:
                        if cand1 == cand2:
                                continue
                        if paretoInf(cand1, cand2, criteres):
                                paretoFront = False
                                break
                if (paretoFront):
                        candidatsFilt.append(cand1)
        return candidatsFilt

def paretoInf(cand1, cand2, criteres):
        equals = 0
        for crit in criteres:
                valCand1 = cand1[crit]
                valCand2 = cand2[crit]
                if (valCand1 > valCand2):
                        return False
                if (valCand1 == valCand2):
                        equals = equals + 1
        return equals < len(criteres)
                
        

print("Decision par la methode PROMETHEE")

criteres = ['distance', 'quantiteInfo'] #noms des criteres utilises
poids = {'distance': 0.6, 'quantiteInfo' : 0.4} #poids des criteres utilises
fctPrefCrit = {'distance': PreferenceType6(10), 'quantiteInfo' : PreferenceType5(60,10)} #fonction de preference utilisee (voir article pour typologie)
candidats = [] #val des candidats pour chacun des criteres utilises (distance negative, car plus c'est faible mieux c'est)
candidats.append({'distance': -50, 'quantiteInfo' : 50})
candidats.append({'distance': -100, 'quantiteInfo' : 20})
candidats.append({'distance': -20, 'quantiteInfo' : 5})
candidats.append({'distance': -75, 'quantiteInfo' : 100})
candidats.append({'distance': -30, 'quantiteInfo' : 5})

candidatsFiltres = filtragePareto(candidats,criteres) #permet de supprimer les candidats ne se trouvant pas sur le front de Pareto
print('filtrage : ', candidatsFiltres)

print('decision : ', decision(candidatsFiltres,criteres,poids, fctPrefCrit))
