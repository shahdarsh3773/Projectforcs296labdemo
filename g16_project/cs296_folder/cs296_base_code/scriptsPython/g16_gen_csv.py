#! /usr/bin/env python3.2
import subprocess
import os
heading = 'iteration-value, rerun-number, step-time, collision-time, velocity-time, position-time, loop-time\n'
iteration=10
rerun=10
for i in range(1,iteration+1):
	for j in range(1,rerun+1):
		execute = ['./mybins/cs296_16_exe', str(i)]
		p1 = subprocess.Popen(execute, stdout=subprocess.PIPE)
		output1 = p1.stdout.readline().decode('utf-8')
		output1 = p1.stdout.readline().decode('utf-8')
		stime = output1[25:-3]
		output1 = p1.stdout.readline().decode('utf-8')
		ctime = output1[31:-3]
		output1 = p1.stdout.readline().decode('utf-8')
		vtime = output1[37:-3]
		output1 = p1.stdout.readline().decode('utf-8')
		ptime = output1[37:-3]
		output1 = p1.stdout.readline().decode('utf-8')
		ltime = output1[19:-3]
		heading = heading + str(i) + ", " + str(j) + ", " + stime + ", " + ctime + ", " + vtime + ", " + ptime + ", " + ltime + "\n"


if not os.path.exists('./data'):
	os.makedirs('./data')

f = open('./data/g16_lab05data_01.csv','w')
f.write(heading)
