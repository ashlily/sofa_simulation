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
    #to compute tarray
    tcount = 0;

    #parameter initialization
    visual_size = 0
    mesh_size = 181
    #for record mesh error
    desired_mesh = []
    #read the desired shapes 
    file = open("/home/ashily/sofa_codes/shape_simulation/deformation_simulation/desired_shapes.txt","r")
    for i in range(0,mesh_size):
        x = np.float64(file.readline())
	y = np.float64(file.readline())
	z = np.float64(file.readline())
        desired_mesh.append(x)
        desired_mesh.append(y)
        desired_mesh.append(z)
    file.close() 
    print "read the desired_mesh \n"
    print "desired_mesh[0] is:"
    print desired_mesh[0]
    print "\n"
    print "size of read desired_mesh is:"
    print len(desired_mesh)
    print "\n"
    
    
    #read index from files
    fp_size = 15
    fp_index = []
    #read fp index
    file = open("/home/ashily/catkin_ws/src/shape_control/src/surface_test/15nodes/s1.txt","r")
    for i in range(0,fp_size):
            x = np.long(file.readline())
            fp_index.append(x)
    file.close() 
    print "read the feature_node_index \n"
    print "fp_index[0] is:"
    print fp_index[0]
    print "\n"
    print "size of read feature_node_index is:"
    print len(fp_index)
    print "\n"

    #warning: node index in SOFA start from 0, however in the raw file "liver.msh", node index start from 1, SOFA convert the form to be starting from 0
    #tp_index = [100]
    tp_index = [85]
    #fp_index = [88, 92, 93, 94, 96, 97, 101, 104, 105, 110] #10p 
    #fp_index = [88, 91,92, 93, 94, 96, 97, 98,99,101, 104, 105, 110] #13p
    #fp_index = [88,91,96,97,98,99,101,104,105,110] #10p2
    manipulation_flag = 0
	
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
        LiverFEM.createObject('TetrahedronFEMForceField', youngModulus='250', name='FEM', poissonRatio='0.45', template='Vec3d')
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

        # show cubes at the t points
	size = len(self.tp_index)
	for i in range(0, size):
            i_temp = self.tp_index[i]-1
	    self.createCube(self.LiverFEM, 'CubeMT' + str(i), 0, 0, 0, 'green')

        # show cubes at the f points
	size = len(self.fp_index)
	for i in range(0, size):
            i_temp = self.fp_index[i]-1
	    self.createCube(self.LiverFEM, 'CubeMF' + str(i), 0, 0, 0, 'blue')
        
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
            file = open("/home/ashily/sofa/v17.06/src/applications/plugins/SofaPython/examples/deformation_simulation/revise/test_key.txt","a")
            file.write("You pressed control+a"+"\n")
            file.close()
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
            self.data = self.conn.recv(16000)
            print "Client Says: "+self.data
            cmd_f =[0 for i in range(3)]
            if self.data == 'Hello, world':
               cmd_f =[0 for i in range(3)]#need to change according to the manipulation_point number
            else:
               cmd_split = self.data.split(",")
               #print "cmd_split is:"
               #print cmd_split
               cmd_f = np.float64(cmd_split)
               #cmd_f =[0 for i in range(3)]
                  
            #print "cmd_f is:"
            #print cmd_f


            self.manipulation_flag += 1
            if self.manipulation_flag >= 10:
               self.manipulation_flag = 10
               cT = self.LiverFEM.getTime()
               #print 'cT = ', cT, "deltaTime = ", deltaTime
               #print 'position', self.LiverFEM.getObject('dofs').position[100]
               movements = self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value
               keyTimes = self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value
               self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('keyTimes').value = [[cT], [cT + 100]] #the movement start at cT (cT+100 no effect)
               #the movement cmd is the displacement with respect to the rest shape
               #self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [1, 1, 0.8]
               #self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [0.5*math.sin(cT), 0.5*math.cos(cT), 0*math.cos(cT)]
               #self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [1*math.sin(0.5*cT), 1*math.cos(0.5*cT), 0.8*math.cos(0.5*cT)]
               self.LiverFEM.getObject('PartialLinearMovementConstraint').findData('movements').value = [cmd_f[0], cmd_f[1], cmd_f[2]]
            
            #compute and record mesh error
            error_sum = 0
            mesh_error = []
            for i in range(0,self.mesh_size):
                dx = np.float64(self.desired_mesh[3*i])
                dy = np.float64(self.desired_mesh[3*i+1])
                dz = np.float64(self.desired_mesh[3*i+2])
                mx = np.float64(self.LiverFEM.getObject('dofs').position[i][0])
	        my = np.float64(self.LiverFEM.getObject('dofs').position[i][1])
	        mz = np.float64(self.LiverFEM.getObject('dofs').position[i][2])
                ex = mx - dx
                ey = my - dy
                ez = mz - dz
                mesh_error.append(ex)
                mesh_error.append(ey)
                mesh_error.append(ez)
                error_sum += (ex*ex + ey*ey + ez*ez)
            #record mesh_error
            '''
            file = open("/home/ashily/sofa_codes/shape_simulation/deformation_simulation/data_record/mesh_error.txt","a")
            for i in range(0,self.mesh_size):
                ex = mesh_error[3*i]
	        ey = mesh_error[3*i+1]
	        ez = mesh_error[3*i+2]
                file.write(str(ex)+"   ,"+str(ey)+"   ,"+str(ez)+"\n")
            file.write("\n")
            file.close() 
            '''
            file = open("/home/ashily/sofa_codes/shape_simulation/deformation_simulation/data_record/error_sum.txt","a")
            file.write(str(error_sum)+"\n")
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
                self.LiverFEM.getObject('CubeMF' + str(i)).findData('position').value = str(x)+' '+str(y)+' '+str(z)+' 0 0 0 1'

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

