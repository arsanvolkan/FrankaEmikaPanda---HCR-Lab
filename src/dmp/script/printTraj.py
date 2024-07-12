#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d  # Fonction pour la 3D

def getTraj(file_name):
	head = np.loadtxt(file_name, delimiter=' ', max_rows=1, dtype=np.str)
	# Lecture des données au format str
	data = np.loadtxt(file_name, delimiter=' ', skiprows=1, dtype=np.str)
	# data = np.char.replace(data, ',', '.')

	# Affichage des en-têtes
	print(head)

	# Sélections des données en fonction de l'en-tête et conversion en flottant
	X = np.asarray(data[:, np.where(head == 'x')],
				   dtype=np.float, order='C').flatten()
	Y = np.asarray(
		data[:, np.where(head == 'y')], dtype=np.float, order='C').flatten()
	Z = np.asarray(
		data[:, np.where(head == 'z')], dtype=np.float, order='C').flatten()
	return [X, Y, Z]

def printTraj(trajectory):
	x = trajectory[0]
	y = trajectory[1]
	z = trajectory[2]

	# Affichage des données
	print(x, '\n', y, '\n', z)

	fig = plt.figure()
	ax = fig.gca(projection='3d')  # Affichage en 3D
	ax.plot(x, y, z, label='Courbe')  # Tracé de la courbe 3D
	plt.title("Courbe 3D")
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.tight_layout()
	plt.show()


if __name__ == '__main__':
	# lines = []
	# with open("plan2.txt", "r") as f:
	# 	lines = f.readlines()
	# for line in lines:
	# 	print(line[0])
	traj = getTraj('planDMP.txt')
	printTraj(traj)
	#file_name="plan2.txt"

	print(X,Y,Z)


