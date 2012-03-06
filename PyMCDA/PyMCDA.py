#!/usr/bin/env python

import unittest
import math

class PreferenceFunction(object):
	"""PROMETHEE abstract preference function"""
	def value(self, difference):
		abstract

class UsualPreferenceFunction(PreferenceFunction):
	"""PROMETHEE Type I preference function"""
	def value(self, difference):
		if difference < 0:
			return 0
		else:
			return 1
		
class UShapePreferenceFunction(PreferenceFunction):
	"""PROMETHEE Type II preference function"""
	def __init__(self, threshold):
		self.threshold = threshold

	def value(self, difference):
		if difference < self.threshold:
			return 0
		else:
			return 1

class VShapePreferenceFunction(PreferenceFunction):
	"""PROMETHEE Type III preference function"""
	def __init__(self, threshold):
		self.threshold = threshold

	def value(self, difference):
		if difference < 0:
			return 0
		elif difference < self.threshold:
			return difference/self.threshold
		else:
			return 1

class LevelPreferenceFunction(PreferenceFunction):
	"""PROMETHEE Type IV preference function"""
	def __init__(self, threshold1, threshold2):
		self.threshold1 = threshold1
		self.threshold2 = threshold2

	def value(self, difference):
		if difference < self.threshold1:
			return 0
		elif difference < self.threshold2:
			return 0.5
		else:
			return 1

class LinearPreferenceFunction(PreferenceFunction):
	"""PROMETHEE Type V preference function"""
	def __init__(self, threshold1, threshold2):
		self.threshold1 = threshold1
		self.threshold2 = threshold2

	def value(self, difference):
		v = VShapePreferenceFunction(self.threshold2-self.threshold1)
		return v.value(difference)

class GaussianPreferenceFunction(PreferenceFunction):
	"""PROMETHEE Type VI preference function"""
	def __init__(self, s):
		self.s = s
		self.sSquare = -1 * (2 * s * s)
	def value(self, difference):
		if difference < 0:
			return 0
		else:
			return 1 - math.exp((difference*difference)/self.sSquare)

class PreferenceFunctionTest(unittest.TestCase):

	def test_preferenceFunctions_returns_zero_when_negative_difference(self):
		f = UsualPreferenceFunction()
		negativeDifference = -1
		self.assertEquals(0, f.value(negativeDifference))
		g = UShapePreferenceFunction(0.5)
		self.assertEquals(0, g.value(negativeDifference))
		h = VShapePreferenceFunction(0.5)
		self.assertEquals(0, h.value(negativeDifference))
		l = LevelPreferenceFunction(0.1, 0.2)
		self.assertEquals(0, l.value(negativeDifference))
		k = LinearPreferenceFunction(0.1, 0.2)
		self.assertEquals(0, k.value(negativeDifference))
		h = GaussianPreferenceFunction(0.5)
		self.assertEquals(0, h.value(negativeDifference))

	def test_UsualPreferenceFonction_returns_1_when_non_negative_difference(self):
		f = UsualPreferenceFunction()
		positiveDifference = 1
		self.assertEquals(1, f.value(positiveDifference))

	def test_UShapePreferenceFunction_returns_0_when_diff_is_less_than_threshold(self):
		threshold = 0.2
		f = UShapePreferenceFunction(threshold)
		self.assertEquals(0, f.value(0.1))

	def test_UShapePreferenceFunction_returns_1_when_diff_is_higher_than_threshold(self):
		threshold = 0.2
		f = UShapePreferenceFunction(threshold)
		self.assertEquals(1, f.value(0.3))
		
	def test_VShapePreferenceFunction_returns_1_when_higher_than_threshold(self):
		threshold = 0.2
		f = VShapePreferenceFunction(threshold)
		self.assertEquals(1, f.value(0.3))

	def test_VShapePreferenceFunction_returns_dotfive_when_middle_of_threshold(self):
		threshold = 0.2
		f = VShapePreferenceFunction(threshold)
		self.assertEquals(0.5, f.value(0.1))
		
	def test_LevelPreferenceFunction_is_0_when_diff_below_first_threshold(self):
		threshold1 = 0.3
		threshold2 = 0.5
		f = LevelPreferenceFunction(threshold1, threshold2)
		self.assertEquals(0, f.value(0.1))

	def test_LevelPreferenceFunction_is_dotfive_when_diff_between_first_and_second_threshold(self):
		threshold1 = 0.3
		threshold2 = 0.5
		f = LevelPreferenceFunction(threshold1, threshold2)
		self.assertEquals(0.5, f.value(0.4))

	def test_LevelPreferenceFunction_is_one_when_diff_is_greather_than_second_threshold(self):
		threshold1 = 0.3
		threshold2 = 0.5
		value = 0.6
		f = LevelPreferenceFunction(threshold1, threshold2)
		self.assertEquals(1, f.value(value))
	
	def test_LinearPreferenceFunction_minus_threshold_is_equal_to_VShapePreferenceFunction(self):
		threshold1 = 0.3
		threshold2 = 0.5
		f = LinearPreferenceFunction(threshold1, threshold2)
		g = VShapePreferenceFunction(threshold2-threshold1)
		value = 0.0
		self.assertEquals(g.value(value), f.value(value-threshold1))
		value = threshold2
		self.assertEquals(g.value(value), f.value(value-threshold1))

def test_GaussianPreferenceFunction_returns_one_minus_exp_minus_over_2_at_s(self):
	s = 10
	f = GaussianPreferenceFunction(s)
	self.assertEquals(1-math.exp(-1/2), f.value(s))
	
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
				PiXA = PiXA + poids * fctPref.value(val1)
				PiAX = PiAX + poids * fctPref.value(val2)
			outRankingPlus = outRankingPlus + PiXA
			outRankingMoins = outRankingMoins + PiAX
		outRankingPlus = outRankingPlus / (len(candidats) - 1)
		outRankingMoins = outRankingMoins / (len(candidats) - 1)
		outRanking = outRankingPlus - outRankingMoins
		if ( first or outRanking > outRankingMax ):
			outRankingMax = outRanking
			bestCandidate = cand1
			first = False
	return bestCandidate

def paretoFilter (candidates, criteria):
	"""http://en.wikipedia.org/wiki/Pareto_efficiency#Pareto_frontier"""
	candidatesList = []
	for cand1 in candidates:
		paretoFront = True
		for cand2 in candidates:
			if cand1 == cand2:
				continue
			if paretoInf(cand1, cand2, criteria):
				paretoFront = False
				break
		if (paretoFront):
			candidatesList.append(cand1)
	return candidatesList
			
def paretoInf(candidate1, candidate2, criteria):
	equals = 0
	for item in criteria:
		criteriaValueForCandidate1 = candidate1[item]
		criteriaValueForCandidate2 = candidate2[item]
		if (criteriaValueForCandidate1 > criteriaValueForCandidate2):
			return False
		if (criteriaValueForCandidate1 == criteriaValueForCandidate2):
			equals = equals + 1
	return equals < len(criteria)
		
#if __name__ == "__main__":
#	unittest.main()

print("Decision done by PROMETHEE method")

#Noms des criteres utilises
criteria = ['distance', 'quantiteInfo']

#poids des criteres utilises
weights = {'distance': 0.6, 'quantiteInfo' : 0.4}

#fonction de preference utilisee (voir article pour typologie)
fctPrefCrit = {'distance': GaussianPreferenceFunction(10), 'quantiteInfo' : LinearPreferenceFunction(60,10)} 

#eval des candidats pour chacun des criteres utilises (distance negative, car plus c'est faible mieux c'est)
candidates = []
candidates.append({'distance': -50, 'quantiteInfo' : 50})
candidates.append({'distance': -100, 'quantiteInfo' : 20})
candidates.append({'distance': -20, 'quantiteInfo' : 5})
candidates.append({'distance': -75, 'quantiteInfo' : 100})
candidates.append({'distance': -30, 'quantiteInfo' : 5})

#permet de supprimer les candidats ne se trouvant pas sur le front de Pareto
filteredCandidates = paretoFilter(candidates,criteria) 
print('Filter: ', filteredCandidates)

print('Decision: ', decision(filteredCandidates, criteria, weights, fctPrefCrit))
