import re
pageTemplate = '''
<!DOCTYPE html PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN">
<html>
<head>
  <meta content="text/html; charset=ISO-8859-1"
 http-equiv="content-type">
  <title>g16_lab09_report</title>
</head>
<body>
<body style="background-color:yellow;">
<h1> {Introduction} </h1>
<p>This webpage has been made by Group 16 for CS-296</p>
<center>{la}</center>
<h1> {G[0]} </h1>
{Gbody[0]}<br><br>
<center>{eq11}=<span>&Sigma;</span>{eq12}</center><br>
<center>{eq21}=<sup>{eq22}</sup>&frasl;<sub>{eq23}</sub></center>
<img src={image[0]}>
<h1> {G[1]} </h1>
{Gbody[1]}<br>
<center>{equation[1]}</center>
<img src={image[1]}>
<h1> {G[2]} </h1>
{Gbody[2]}
<img src={image[2]}>
<h1> {G[3]} </h1>
{Gbody[3]}
<img src={image[3]}>
<h1> {G[4]} </h1>
{Gbody[4]}
<img src={image[4]}>
</body>
</html>''' # NEW note '{person}' two lines up

def main(): 
	f1=open("./doc/cs296_project_report.tex","r")
	s=f1.read()
	la=(re.match('^.*\\\\author(.*)\\\\date',repr(s)).group(1))
	la=la.replace("\\\\","  ")
	la=la.replace("\\n","<tr>")
	la=la.replace("{","")
	la=la.replace("}","")
	la=la.replace("\\","")
	la=la.replace(" and","<br>")
	la=la.replace("texttt","email : ")
	G=["","","","",""," "]
	Gbody=["","","","","","","","","","",""]
	image=["./images/plot1.png","./images/plot2.png","./images/plot3.png","./images/plot4.png","./images/plot5.png"]
	equation=["","","","","","","","","","","","","",""]
	Introduction = "Analysis of Graphs from Lab05"
	f=open("./doc/cs296_project_report.tex","r")
	eq1num=""
	eq1den=""
	lines = f.readlines()
	i=0
	check=0
	j=0
	k=0
	l=0
	eq1found=0
	eq1stored=0
	eq2found=0
	eq2stored=0
	eq3found=0
	eq3stored=0
	eq1statement=""
	eq2statement=""
	eq3statement=""
	flag1=0
	flag2=0
	#for line in lines:
		#if line.find("author")!=-1 or flag2==1:
			#flag2=1
			#if line.find("{")!=-1 and flag2==1:
				#flag1=flag1+1
				#print(line)
			#elif line.find("}")!=-1 and flag2==1:
				#flag1=flag1-1
				#print(line)
				#if flag1==0:
					#flag2=0
					#break
	for line in lines:
		lines[j]=lines[j].replace("{\\it x}","<I> x</I>")
		lines[j]=lines[j].replace("{\\it top}","<I> top</I>")
		lines[j]=lines[j].replace("{\\it experimental errors}","<I> experimental errors</I>")
		lines[j]=lines[j].replace("\\%","%")
		j=j+1
		
	j=0
	for line in lines:
		lines[j]=lines[j].replace("\\newline","<br>")
		j=j+1	
	for line in lines:
		if (line.find("begin{equation}") != -1 and eq1found==0)or (eq1found==1 and eq1stored==0):
			if eq1found==1:
				eq1statement=line.strip()
				eq1stored=1
				eq1found=0			
			eq1found=1
			l=l+1
		elif (line.find("begin{equation}") != -1 and eq2found==0) or (eq2found==1 and eq2stored==0):
			if eq2found==1:
				eq2statement=line.strip()
				eq2stored=1
				eq2found=0			
			eq2found=1
			l=l+1
		elif (line.find("begin{equation}") != -1 and eq3found==0) or (eq3found==1 and eq3stored==0):
			if eq3found==1:
				eq3statement=line.strip()
				eq3statement="<br>"+"<center>"+eq3statement+"</center>"+"<br>"
				eq3stored=1
				eq3found=0			
			eq3found=1
			l=l+1
	lines2=[]
	counts=0
	for line in lines:
		if line.find("\\section")!=-1 and counts==0:
			counts =1
			continue
		if line.find("\\section")!=-1 and counts==1:
			counts =2
			continue
		if counts ==2:
			lines2.append(line)
	for line in lines2:
		if line.find("gettimeofday") != -1:
			break
		else:
			if line.find("\\begin") != -1:
					continue
		
			else :
				if line.find("\\subsection")!=-1:
					str1=line.find('{') + 1
					str2=line.find('}')
					G[i]=line[str1:str2]
					#print(G[i])
					check = 1
				else:
					if check == 1:
						if line.find("X=P+C+V") != -1:
							Gbody[i]=Gbody[i]+eq3statement
						if  line.find("X=P+C+V") == -1 and line.find("end{equation}") == -1 and line.find("figure") == -1 and line.find("minipage") == -1 and line.find("graphics") == -1 and line.find("caption") == -1 and line.find("subsubsection") == -1  and line.find("displaystyle") == -1  and line.find("S=L/N") == -1: #and line.find("cs296") == -1 and line.find("90") == -1 and line.find("100") == -1:
							Gbody[i]=Gbody[i]+line
						if lines2[lines2.index(line)+1].find("\\subsection")!=-1 :
							check=0
							i=i+1
						else:
							if lines2[lines2.index(line)+1].find("\\begin")!=-1 and lines2[lines2.index(line)+2].find("\\subsection")!=-1: 
								check=0
								i=i+1
	eq11=eq1statement[0]
	eq12=eq1statement[-1]
	eq13=eq1statement[-3]
	eq21=eq2statement[0]
	eq22=eq2statement[2]
	eq23=eq2statement[-1]
	browseLocal(pageTemplate.format(**locals()))
	#print("Equation 1 :"+eq11+" "+eq12+" "+eq13)
	#print("Equation 2 :"+eq21+" "+eq22+" "+eq23)
	#print("Equation 3 :"+eq3statement)

def strToFile(text, filename):
	"""Write a file with the given name and the given text."""
	output = open(filename,"w")
	output.write(text)
	output.close()

def browseLocal(webpageText, filename='./doc/g16_lab09_report.html'):
	'''Start your webbrowser on a local file containing the text
	with given filename.'''
	import webbrowser, os.path
	strToFile(webpageText, filename)
	webbrowser.open("file:///" + os.path.abspath(filename)) #elaborated for Mac

main()
