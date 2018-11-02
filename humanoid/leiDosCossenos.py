#!/usr/bin/env python
# license removed for brevity

'''powered by marcos'''
import math

a = 8
c = 7.5
altura = 14.
pos_inicial_pelves = [0., 0., altura]
pos_inicial_foot = [0., 0., -altura]

deslocamentoXpes = 4
deslocamentoYpelves = 2
deslocamentoZpes = 3
nEstados = 125
tempoPasso = 2

def footToHip(pointHip):
	angulos = []
	x,y,z = pointHip

	#ankle roll
	theta = math.atan(y/z)
	angulos.append(theta)

	#ankle pitch
	b = math.sqrt(x**2+y**2+z**2)
	anguloA = math.acos((a**2-(b**2+c**2))/(-2*b*c))
	betha = math.atan(x/z)
	anguloA = betha + anguloA
	angulos.append(anguloA)

	#knee
	anguloB = math.acos((b**2-(a**2+c**2))/(-2*a*c))
	anguloB = anguloB - math.pi
	angulos.append(anguloB)

	#hip pitch
	anguloC = math.acos((c**2-(a**2+b**2))/(-2*a*b))
	anguloC = anguloC - betha
	angulos.append(anguloC)

	#hip roll
	angulos.append(-theta)

	#hip yall
	angulos.append(0)

	return angulos

def hipToFoot(pointFoot):
	angulos = []
	x,y,z = pointFoot
	theta = math.atan(y/x)
	angulos.append(theta)
	#print('hipToFoot',x,y,theta)
	b = math.sqrt(x**2+y**2+z**2)
	anguloA = math.acos((a**2-(b**2+c**2))/(-2*b*c))
	angulos.append(anguloA)
	anguloB = math.acos((b**2-(a**2+c**2))/(-2*a*c))
	angulos.append(anguloB)
	anguloC = math.acos((c**2-(a**2+b**2))/(-2*a*b))
	angulos.append(anguloC)
	angulos.append(-1*theta)
	angulos.append(0)
	return angulos

def getTragectoryPoint(x):
	pos_pelves = pos_inicial_pelves[:]
	p1 = (deslocamentoXpes/2)*((math.exp((2*(x-nEstados/2))/50) - math.exp((2*(x-nEstados/2))/-50))/(math.exp((2*(x-nEstados/2))/50)+math.exp((2*(x-nEstados/2))/-50)))
	pos_pelves[0] = p1
	pos_pelves[1] = -deslocamentoYpelves*math.sin(x*math.pi/nEstados)

	pos_foot = pos_inicial_pelves[:]
	p2 = (-deslocamentoXpes/2)*((math.exp((2*(x-nEstados/2))/50) - math.exp((2*(x-nEstados/2))/-50))/(math.exp((2*(x-nEstados/2))/50)+math.exp((2*(x-nEstados/2))/-50)))
	pos_foot[0] = p2
	pos_foot[2] = altura - deslocamentoZpes*math.exp(-((x-nEstados/2)**2)/600)
	return pos_pelves, pos_foot
