# pyhiro
python scripts to control the hiro/nxo/hrp5p

The following libraries are used

1. Panda3D
2. Networkx
3. Numpy, Scipy
4. MySQLdb

The following executable programs are used:

1. MySQL set (including server, bench, dump, etc.)
    * Import 20170113freegrip.sql as into the DBMS system first.

The program follows these common sense:

1. A 3d point or nd vector is represented using one row of np.array list
2. A n-by-3 matrix is represented by n rows of np.array list

##Executing the program for Fixture Regrasp 

- fixture
1. prepare object,fixture stl model.(mentionthat the fixture model contains its postion,the object model is near (0,0,0))
2. Compute the grasps using freegrip_air.py (under manipulation/regrasp_onworkcell/grip)
3. Compute the drop stable pose by dropworkcell.py(under manipulation/regrasp_onworkcell/dropsimulation)
4. Compute the ik-feasible grasps all over a workcell  using dropworkcellgrip.py (under manipulation/regrasp_onworkcell/grip)
5. Plot and compute motion Sequence by nxtglplot(under manipulation/regrasp_onworkcell/examples),
build the regrasp graph is in regriptpp.py (under manipulation/regrasp_onworkcell)

- table
2. Compute the grasps using freegrip_air.py (under manipulation/regrasp_onworkcell/grip)(if it has been compute in fixture program,there is no need to compute again)
2. Compute the stable placements using freetabletopplacement.py (under manipulation/regrasp_onworkcell/grip)
3. Compute the stable placements and ik-feasible grasps all over a table surface using tableplacements.py (under manipulation/regrasp)
4. Build the regrasp graph using regriptpp.py (under manipulation/regrasp)
Step 4 is integrated in regrasp/examples/nxtsglplot.py, see the final results by executing this file.

- experiments: compute mant task
1. generate startgoal first using startgoalgenerate,
2. then prepare task ik first using taskik.py 
3. then build regrasp map using regriptpp_quick.py,so the build regrasp will be very fast about 0.02s each task.(regripp.py version time is 1.6s)
4. in examples.the nxtglplot_experiment_quick will use regripp_quick.nxtglplotreal is for real robot,pathplot is for plot regrasp length and task sucess rate.(tbc:fixture table basket camera)

- real robot: Nextage
1. table regrasp:nxtsglplotreal(regrasp/examples)
2. fixture regrasp:nxtsglplotreal(regrasp_onworkcell/examples)

##database
- pay attention to the database(right and clean)

Delete the grasp data (in MySQL workbench):
```sql
SET FOREIGN_KEY_CHECKS=0;
truncate table freegrip.tabletopplacements;
truncate table freegrip.tabletopgrips;
truncate table freegrip.ik;
truncate table freegrip.freeairgrip;
truncate table freegrip.freetabletopplacement;
truncate table freegrip.freetabletopgrip;
SET FOREIGN_KEY_CHECKS=1;
```

Delete the tabletop regrasp data (in MySQL workbench):
```sql
SET FOREIGN_KEY_CHECKS=0;
truncate table tabletopplacements;
truncate table tabletopgrips;
truncate table ik;
SET FOREIGN_KEY_CHECKS=1;
```

Delete the fixture regrasp  data (in MySQL workbench):
``` sql
SET FOREIGN_KEY_CHECKS=0;
truncate table freegrip.freeairgrip;
truncate table freegrip.ik_drop;
truncate table freegrip.dropfreegrip;
truncate table freegrip.dropstablepos;
truncate table freegrip.dropworkcellgrip;
SET FOREIGN_KEY_CHECKS=1;
``` 
## models
- pay attention to objpath,workcellpath, the stl model(check size and position in blender)
## parameters:
###in dropsimulation:
dropworcell.py
- experiment time: parameter ntimes,def updateODE limittime,
- setup drop pos：def addobj setpos,height,
- setup check stable:def checkrange minxminy，checkdiff,check repeat)
###in example:
    
nxtsglplot.py,nxtsglplotreal.py 
- determine which path to show : id
 
regriptpp_quick.py
- loadgripstobuildgraph, there is choice to control the num of droptablepos to show the time effect results)
   
startgoalgenerate.py 
- tasks position range: grids-x,y
- tasks angle range: discretesize in def savetoDB
###in grip:
freegrip_air.py
- reduceradius-lower more airgrips,
- torqueresist-lower less grip

freetableplacement.py
- dhover-0.15,0.12 higher less placement
  
