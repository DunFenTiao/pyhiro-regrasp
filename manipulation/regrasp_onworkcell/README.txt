The following executable programs are used:
Import xxxx.sql as into the DBMS system first.

#Executing the program for Drop on workcell simulation and get the motion sequence
1. Compute the stable pose by dropworkcell.py(under manipulation/regrasp_onworkcell/dropsimulation)
2. Compute the grasps using freegrip_air.py (under manipulation/regrasp_onworkcell/grip)
3. Compute the ik-feasible grasps all over a workcell and tablbe surface using dropworkcellgrip.py (under manipulation/regrasp_onworkcell/grip)
4. Plot and compute motion Sequence by nxtglplot(under manipulation/regrasp_onworkcell/examples),build the regrasp graph is in regriptpp.py (under manipulation/regrasp_onworkcell)

Delete the data((in MySQL workbench) 

SET FOREIGN_KEY_CHECKS=0;

truncate table freegrip.freeairgrip;

truncate table freegrip.ik_drop;

truncate table freegrip.dropfreegrip;

truncate table freegrip.dropstablepos;

truncate table freegrip.dropworkcellgrip;

SET FOREIGN_KEY_CHECKS=1;

#majiayao20170828


- pay attention to the database(right and clean),objpath,workcellpath, the stl model(check size and position in blender)


in dropsimulation:
dropworcell_t2tubeBox.py
(parameter ntimes,def updateODE limittime,def addobj setpos,height,def checkrange minxminy...,
,todo checkdiff,check repeat)


in example:
nxtsglplotreal.py one task a time(parameter:id-path idnumber)
startgoalgenerate.py (parameter:grids-x,y,discretesize in def savetoDB)


in grip:
dropworkcell grip is for generate grasp for object-fixture task
freegrip_air generate freeairgrip(parameter:reduceradius-lower more airgrips,torqueresist,lower less grip)
freetableplacement generate placement(parameter:dhover-0.15,0.12 higher less placement )

regriptpp.py (compute ik for start goal each time)
regriptpp_quick.py(prepare ik using taskik.py)(also in loadgripstobuildgraph, there is choice to control the num of droptablepos to show the time effect results)

