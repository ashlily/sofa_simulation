import sys
import Sofa
import socket
import math
import numpy as np

class TutorialForceFieldLiverFEM (Sofa.PythonScriptController):
    count=0 
       
    #communication
    HOST = ''                 # Symbolic name meaning the local host
    PORT = 50007              # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn = 0;
    data = 0;
    c = "0";
    
    #parameter initialization
    mesh_size = 181
    visual_size = 0
    #warning: node index in SOFA start from 0, however in the raw file "liver.msh", node index start from 1, SOFA convert the form to be starting from 0
    #tp_index = [100]
    tp_index = [85]
    fp_index = [88, 92, 93, 94, 96, 97, 101, 104, 105, 110] #10p
    #fp_index = [88, 91,92, 93, 94, 96, 97, 98,99,101, 104, 105, 110] #13p
    manipulation_flag = 0
    stable_flag = 0
    desired_mesh = []
	
    def __init__(self, node, commandLineArguments) : 
        self.count = 0
        self.sign = -1
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)

        return

    def createCube(self, parentNode, name, x, y, z, color):
	size = 0.1

        # rootNode/Spoon3
        node = parentNode.createChild(name)
        object = node.createObject('MechanicalObject',name='MecaObject',template='Rigid',scale3d=str(size)+' '+str(size)+' '+str(size))
        object.findData('position').value=str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

        # rootNode/Spoon3/Visu
        VisuNode = node.createChild('Visu')
        VisuNode.createObject('OglModel',name='Visual',fileMesh='mesh/PokeCube.obj',color=color,scale3d=str(size)+' '+str(size)+' '+str(size))
        VisuNode.createObject('RigidMapping',input='@..',template='Rigid3d,ExtVec3f',output='@Visual') 
        
        return node

    def createGraph(self,rootNode):
	gravity = '0 0 0'

        # rootNode
        rootNode.createObject('EulerImplicitSolver', rayleighStiffness='0.1', name='Implicit Euler Solver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1e-5', tolerance='1e-5', name='Conjugate Gradient', iterations='25', template='GraphScattered')
        rootNode.createObject('VisualStyle', displayFlags='showBehavior')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DiscreteIntersection')

        # rootNode/LiverFEM
        LiverFEM = rootNode.createChild('LiverFEM')
        self.LiverFEM = LiverFEM
        LiverFEM.gravity = gravity
        LiverFEM.createObject('MeshTopology', name='mesh', fileTopology='mesh/liver.msh')
        LiverFEM.createObject('MechanicalObject', name='dofs', template='Vec3d')
        #LiverFEM.createObject('TetrahedronFEMForceField', youngModulus='500', name='FEM', poissonRatio='0.45', template='Vec3d')
        LiverFEM.createObject('TetrahedronFEMForceField', youngModulus='1000', name='FEM', poissonRatio='0.45', template='Vec3d')
        ##change
        #LiverFEM.createObject('TetrahedronFEMForceField', youngModulus='30', name='FEM', poissonRatio='0.25', template='Vec3d')
        LiverFEM.createObject('UniformMass', name='mass', totalmass='1')
        LiverFEM.createObject('FixedConstraint', indices='3 39 64', name='FixedConstraint')
        #LiverFEM.createObject('PartialLinearMovementConstraint', indices='100 105 110', keyTimes='0 5 10 15 20', template='Vec3d', movements='0 0 0 -1 1 1 0.2 -0.2 -0.2 -0.5 0.5 0.5 0 0 0')
        #LiverFEM.createObject('PartialLinearMovementConstraint', indices='100 105 110', keyTimes='0 10', template='Vec3d', movements='0 0 0  0.1 -0.1 -0.1 -0.1 0.1 0.1')
        #LiverFEM.createObject('PartialLinearMovementConstraint', indices='100', keyTimes='0', template='Vec3d', movements='0 0 0')
        LiverFEM.createObject('PartialLinearMovementConstraint', indices='85', keyTimes='0', template='Vec3d', movements='0 0 0')

        # rootNode/LiverFEM/Visu
        Visu = LiverFEM.createChild('Visu')
        self.Visu = Visu
        Visu.gravity = gravity
        Visu.createObject('OglModel', name='VisualModel', fileMesh='mesh/liver-smooth.obj')
        Visu.createObject('BarycentricMapping', input='@../dofs', name='visual mapping', output='@VisualModel')

        # rootNode/LiverFEM/Surf
        Surf = LiverFEM.createChild('Surf')
        self.Surf = Surf
        Surf.gravity = gravity
        Surf.createObject('SphereLoader', name='SphereLoader', filename='mesh/liver.sph')
        Surf.createObject('MechanicalObject', position='@[-1].position', name='mappedMS')
        Surf.createObject('SphereModel', listRadius='@[-2].listRadius', name='CollisionModel')
        Surf.createObject('BarycentricMapping', input='@../dofs', name='sphere mapping', output='@mappedMS')
	'''
        # Spring1
        Spring1 = rootNode.createChild('Spring1')
        self.Spring1 = Spring1
        Spring1.gravity = '0 0 0'
        Spring1.createObject('MechanicalObject', position='-4 4.5 2', name='Particles', template='Vec3d', restScale='0.1')
        Spring1.createObject('UniformMass', name='Mass', template='Vec3d')
        Spring1.createObject('FixedConstraint', indices='0', name='FixedConstraint', template='Vec3d')
        rootNode.createObject('StiffSpringForceField', object1='@LiverFEM', object2='@Spring1', name='Interaction Spring', template='Vec3d', spring='110 0 1000000 0.1 0')
        
        # Spring2
        Spring2 = rootNode.createChild('Spring2')
        self.Spring2 = Spring2
        Spring2.gravity = '0 0 0'
        Spring2.createObject('MechanicalObject', position='-2 4.5 2', name='Particles', template='Vec3d', restScale='0.1')
        Spring2.createObject('UniformMass', name='Mass', template='Vec3d')
        Spring2.createObject('FixedConstraint', indices='0', name='FixedConstraint', template='Vec3d')
        rootNode.createObject('StiffSpringForceField', object1='@LiverFEM', object2='@Spring2', name='Interaction Spring', template='Vec3d', spring='92 0 1000000 0.1 0')
        '''

        # show cubes at the t points
	size = len(self.tp_index)
	for i in range(0, size):
            i_temp = self.tp_index[i]-1
	    self.createCube(self.LiverFEM, 'CubeMT' + str(i), 0, 0, 0, 'green')

        '''
        # show cubes at the f points
	size = len(self.fp_index)
	for i in range(0, size):
            i_temp = self.fp_index[i]-1
	    self.createCube(self.LiverFEM, 'CubeMF' + str(i), 0, 0, 0, 'blue')
        '''
        
        #communication
	self.s.bind((self.HOST, self.PORT))
	self.s.listen(1)
	self.conn, self.addr = self.s.accept()
	print 'Connected by', self.addr
       
	return 0

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onKeyReleased(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You released a"
        return 0;

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onKeyPressed(self, c):
        ## usage e.g.
        if c=="A" :
            print "You pressed control+a"
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        ## usage e.g.
        #if isPressed : 
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0;

    def storeResetState(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        #self.conn.close()
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print 'onGUIEvent'
        return 0;

    def onEndAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print 'onScriptEvent'
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
	
        try:
            
            #communication
            #receive
            self.data = self.conn.recv(2048)
            print "Client Says: "+self.data
            
              

            cmd_f =[0 for i in range(15)]
            if self.data == 'Hello, world':
               cmd_f =[0 for i in range(3)]#need to change according to the manipulation_point number
            else:
               cmd_split = self.data.split(",")
               cmd_f = np.float64(cmd_split)
               #cmd_f =[0 for i in range(3)]



            self.manipulation_flag += 1
            if self.manipulation_flag >= 5:
               self.manipulation_flag = 5
               cT = self.LiverFEM.getTime()
               movements = self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value
               keyTimes = self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value
               self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value = [[cT], [cT + 100]] #the movement start at cT (cT+100 no effect)
               #the movement cmd is the displacement with respect to the rest shape
               self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [1, 1, 0.8]
               #self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [1*math.sin(cT), 1*math.cos(cT), 0.8*math.cos(cT)]
               #self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [cmd_f[0], cmd_f[1], cmd_f[2]]
               
               
            #record position information of all mesh nodes when stable
            if self.manipulation_flag >= 5:
                 self.stable_flag += 1
                 if self.stable_flag == 200:
                      print "record desired mesh:"
                      print "\n"
                      file = open("/home/ashily/sofa_codes/shape_simulation/deformation_simulation/desired_shapes.txt","w")
                      for i in range(0,self.mesh_size):
                          x = np.float64(self.LiverFEM.getObject('dofs').position[i][0])
	                  y = np.float64(self.LiverFEM.getObject('dofs').position[i][1])
	                  z = np.float64(self.LiverFEM.getObject('dofs').position[i][2])
                          file.write(str(x)+"\n"+str(y)+"\n"+str(z)+"\n")
                      file.close() 
           
           

            #position information
            tp_position = []
            fp_position = []
            fp_tp_position = []

            #update scene information
	    size_fp = len(self.fp_index)
	    for i in range(0, size_fp):
                i_fp = self.fp_index[i]
                x = np.float64(self.LiverFEM.getObject('dofs').position[i_fp][0])
	        y = np.float64(self.LiverFEM.getObject('dofs').position[i_fp][1])
	        z = np.float64(self.LiverFEM.getObject('dofs').position[i_fp][2])
	        fp_position.append([x,y,z])
                fp_tp_position.append(x)
                fp_tp_position.append(y)
                fp_tp_position.append(z)
                #self.LiverFEM.getObject('CubeMF' + str(i)).findData('position').value = str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

            size_tp = len(self.tp_index)
	    for i in range(0, size_tp):
                i_tp = self.tp_index[i]
                x = np.float64(self.LiverFEM.getObject('dofs').position[i_tp][0])
	        y = np.float64(self.LiverFEM.getObject('dofs').position[i_tp][1])
	        z = np.float64(self.LiverFEM.getObject('dofs').position[i_tp][2])
	        tp_position.append([x,y,z])
                fp_tp_position.append(x)
                fp_tp_position.append(y)
                fp_tp_position.append(z)
                self.LiverFEM.getObject('CubeMT' + str(i)).findData('position').value = str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

            #print "len tp is:"
            #print len(tp_position)
            #print "tp is:"
            #print tp_position

            #print "len fp_tp_position is:"
            #print len(fp_tp_position)
            #print "fp_tp_position is:"
            #print fp_tp_position

            str_fp_tp_position_long = str(fp_tp_position)
            #print "str_fp_tp_position_long is:"
            #print str_fp_tp_position_long

            str_fp_tp_position = str_fp_tp_position_long[1:-1]
            #print "len str_fp_tp_position is:"
            #print len(str_fp_tp_position)
            #print "str_fp_tp_position is:"
            #print str_fp_tp_position

            #publish position information
            self.conn.sendall(str_fp_tp_position)

            #split data
            data_split = str_fp_tp_position.split(",")
            #print "len data_split is:"
            #print len(data_split)

            #convert the split data to float64
            data_f =[]
            N_data = len(data_split)
            for i in range(0,N_data):
                 data_f.append(np.float64(data_split[i]))
            #print "len data_f is:"
            #print len(data_f)
            #print "data_f is:"
            #print data_f

            #test direct convert
            data_f_d = np.float64(data_split)
            #print "len data_f_d is:"
            #print len(data_f_d)
            #print "data_f_d is:"
            #print data_f_d
            '''
            #publish position information
            #convert the float list to string list
            str_fp_tp_position_long = str(fp_tp_position)
            #str_fp_tp_position = [format(flt) for flt in fp_tp_position] 
            #print  str_fp_tp_position_long
            str_fp_tp_position = str_fp_tp_position_long[1:-1]
            #print str_fp_tp_position 
            '''

            

	   
        except socket.error:
            print "Error Occured."
        
        return 0


def createScene(rootNode):
    rootNode.findData('dt').value = '0.02'
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    myTutorialForceFieldLiverFEM = TutorialForceFieldLiverFEM(rootNode,commandLineArguments)
    return 0

