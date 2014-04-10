import csv
from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

itn_no=10
rr_no=10
roll_number=10#Darsh's roll. Give 61 when you have more number of iteration numbers
ran_no=5#number of random values to be chosen
X=np.arange(itn_no)
step_arr=[]
loop_arr=[]
vel_arr=[]
col_arr=[]
pos_arr=[]
tot_arr=[]
cumuX = np.arange(rr_no)
loop=0
f=open("./data/g16_lab05data_01.csv","r")

f_csv = csv.reader(f)
vel=0
pos=0
col=0
count = 0
step=0
iteration_step= []
positive_step = []
negative_step = []
temp_count=0
value_roll=[]
for line in f_csv:
	if count != 0:#the first line is removed since it has the headings
		iteration_step.append(float(line[2].strip()))
		step =step+ float(line[2].strip())
		loop =loop+ float(line[6].strip())
		vel =vel+ float(line[4].strip())
		pos =pos+ float(line[5].strip())
		col =col + float(line[3].strip())
		temp_count += 1

		
		if rr_no*(roll_number-1)+1 <= temp_count <= (rr_no*(roll_number-1) +  rr_no) :
			#print(line[2].strip()[0:3])
			value_roll.append( float(line[2].strip()[0:3]))
			#print(line)


		if count%rr_no==0:
			step_arr.append(step/rr_no)
			loop_arr.append(loop/rr_no)
			vel_arr.append(vel/rr_no)
			pos_arr.append(pos/rr_no)
			col_arr.append(col/rr_no)
			temp=(col/rr_no)+ (vel/rr_no) + (pos/rr_no)
			tot_arr.append(temp)
			positive_step.append( max(iteration_step) - (step/rr_no) )
			negative_step.append(  -min(iteration_step)  + (step/rr_no))
			step = 0
			loop=0
			vel=0
			col=0
			pos=0
			iteration_step = []	
	count += 1

count=0
stepran=0
stepranY=[]
randarr=[]

f.close()
f=open("./data/g16_lab05data_01.csv","r")
f_csv2 = csv.reader(f)
for line2 in f_csv2:
	if count != 0:
		tepz=str(line2[1]).strip()
		if count%rr_no==1:
			if count!=1:
				stepranY.append(stepran/ran_no)
			stepran=0
			randarr=[]
			for j in range(ran_no):
				from random import randint
				randarr.append(str(randint(1,rr_no)).strip())
		if tepz in randarr:
			occ = randarr.count(tepz)
			stepran += occ*float(line2[2].strip())
		if count==itn_no*rr_no:
			stepranY.append(stepran/ran_no)
	count+=1

stepg=plt.bar(X, step_arr, width = 1, color = 'green', edgecolor='white',label="AVERAGE STEP TIME")
loopg=plt.plot(X, loop_arr, color = 'pink',label="LOOP TIME")
maxiz=max(loop_arr)
minz=min(loop_arr)
plt.xlabel("ITERATION NUMBERS")
plt.ylabel("AVERAGE STEP TIME AND LOOP TIME (ms)")
maxzz=plt.axhline(y=maxiz, color = 'blue', xmin=0.0,label="LOOPTIME MAX")
minzz=plt.axhline(y=minz, color = 'red', xmin=0.0,label="LOOPTIME MIN")
plt.title("GRAPH-I")
plt.legend(loc='best')
ylim(0.0,maxiz+5.0)
plt.savefig('./plots/g16_lab09_plot1.png')
plt.clf()

stepg=plt.plot(X, step_arr, color = 'green', label="step-time")
velg=plt.plot(X, vel_arr, color = 'blue', label="velocity-time")
posg=plt.plot(X, pos_arr, color = 'pink', label="position-time")
colg=plt.plot(X, col_arr, color = 'orange', label="collision-time")
totg=plt.plot(X, tot_arr, color = 'red', label="total-time")
plt.xlabel("ITERATION NUMBERS")
plt.ylabel("AVERAGE TIMES (ms)")
plt.title("GRAPH_II")
plt.legend(loc='best')
plt.savefig('./plots/g16_lab09_plot2.png')
plt.clf()

graph_freq = plt.hist(value_roll, bins = rr_no//3, label = "HISTOGRAM")
value_roll.sort()
graph_cum = plt.plot( value_roll, cumuX + 1, label = "CUMULATIVE FREQUENCY" ) 
plt.xlabel('TIME IN ms')
plt.ylabel('NUMBER OF RERUNS')
plt.title('ROLL NUMBER FREQUENCY')
plt.legend(loc='best')	
plt.savefig('./plots/g16_lab09_plot04.png')
plt.clf()

coefs = np.lib.polyfit(X, step_arr, 1)
fit_y = np.lib.polyval(coefs, X) 
plt.scatter(X, step_arr, color = 'red', label="STEP_TIME FOR ALL VALUES")
plt.plot(X, fit_y, 'b--', label="BEST FIT FOR ALL VALUES")
coefsran = np.lib.polyfit(X, stepranY, 1) 
fit_y_ran = np.lib.polyval(coefsran, X) 
plt.scatter(X, stepranY, color = 'green', label="STEP_TIME FOR RANDOM VALUES")
plt.plot(X, fit_y_ran, 'b--', color= 'brown', label="BEST FIT FOR RANDOM VALUES")
xlim(0.0,itn_no)
plt.xlabel("ITERATION NUMBERS")
plt.ylabel("AVERAGE STEP_TIME (ms)")
plt.title("GRAPH-V")
plt.legend(loc='best')
plt.savefig('./plots/g16_lab09_plot5.png')
plt.clf()
			
graph_step = plt.plot(X + 1, step_arr, label = "AVG STEP TIME", color = 'red')
graph_error = plt.errorbar(X + 1, step_arr, yerr=[negative_step, positive_step], color = 'r', ecolor='g', capthick=0.5, label = "ERROR IN AVG STEP TIME" )
plt.xlabel('NUMBER OF ITERATIONS')
plt.ylabel('AVG STEP TIME WITH ERROR (ms)')
plt.title('AVG STEP TIME WITH ERROR Vs ITERATIONS')
plt.legend(loc='best')
plt.savefig('./plots/g16_lab09_plot03.png')
plt.clf()

#for i in value_roll:
	#print(i)
#print(rr_no//3)








